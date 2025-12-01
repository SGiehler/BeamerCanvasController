#include "MotionController.h"

MotionController motion;

// Pin Definitions
#define X_STEP_PIN    25
#define X_DIR_PIN     5
#define X_LIMIT_PIN   39
#define X_ADDRESS     0b00

#define Y_STEP_PIN    27
#define Y_DIR_PIN     23
#define Y_LIMIT_PIN   36
#define Y_ADDRESS     0b01

#define Z_STEP_PIN    14
#define Z_DIR_PIN     19
#define Z_LIMIT_PIN   34
#define Z_ADDRESS     0b10

#define A_STEP_PIN    26
#define A_DIR_PIN     18
#define A_LIMIT_PIN   35
#define A_ADDRESS     0b11

#define R_SENSE       0.11f

// Motion Constants (Can be tuned or moved to settings later)
// Original: MAX_SPEED = MICROSTEPS * 300 = 16 * 300 = 4800 steps/sec
// Original: MAX_ACCEL = MICROSTEPS * 150 = 16 * 150 = 2400 steps/sec^2
// 4800 steps/sec / 400 steps/mm = 12 mm/sec.
const float MAX_SPEED_MM_S = 12.0;
const float MAX_ACCEL_MM_S2 = 6.0;

MotionController::MotionController() {
    // X
    axisConfigs[0] = {X_STEP_PIN, X_DIR_PIN, X_LIMIT_PIN, X_ADDRESS, R_SENSE};
    // Y
    axisConfigs[1] = {Y_STEP_PIN, Y_DIR_PIN, Y_LIMIT_PIN, Y_ADDRESS, R_SENSE};
    // Z
    axisConfigs[2] = {Z_STEP_PIN, Z_DIR_PIN, Z_LIMIT_PIN, Z_ADDRESS, R_SENSE};
    // A
    axisConfigs[3] = {A_STEP_PIN, A_DIR_PIN, A_LIMIT_PIN, A_ADDRESS, R_SENSE};
}

void MotionController::begin() {
    SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); // Disable initially

    for(int i=0; i<4; i++) {
        // Initialize Steppers
        steppers[i].connectToPins(axisConfigs[i].stepPin, axisConfigs[i].dirPin);

        // Initialize Drivers
        drivers[i] = new TMC2209Stepper(&SERIAL_PORT, axisConfigs[i].r_sense, axisConfigs[i].address);

        setupDriver(i);

        // Setup initial speed/accel based on settings
        StepperSettings s = settings.getStepperSettings(i);
        float stepsPerMm = (float)(25 * s.microsteps);
        steppers[i].setStepsPerMillimeter(stepsPerMm);
        steppers[i].setSpeedInStepsPerSecond(MAX_SPEED_MM_S * stepsPerMm);
        steppers[i].setAccelerationInStepsPerSecondPerSecond(MAX_ACCEL_MM_S2 * stepsPerMm);
    }
}

void MotionController::setupDriver(int i) {
    drivers[i]->begin();
    drivers[i]->toff(4);
    drivers[i]->blank_time(24);

    StepperSettings s = settings.getStepperSettings(i);
    drivers[i]->rms_current(s.current);
    drivers[i]->microsteps(s.microsteps);

    // StallGuard configuration
    drivers[i]->TCOOLTHRS(0xFFFFF); // Enable coolstep/stallguard for all speeds
    drivers[i]->SGTHRS(s.stallGuardThreshold);
}

void MotionController::requestUpdateSettings() {
    settingsUpdateRequested = true;
}

void MotionController::updateSettings() {
    if(isMoving()) return; // Don't update while moving

    for(int i=0; i<4; i++) {
        setupDriver(i);
        StepperSettings s = settings.getStepperSettings(i);
        float stepsPerMm = (float)(25 * s.microsteps);
        steppers[i].setStepsPerMillimeter(stepsPerMm);
        steppers[i].setSpeedInStepsPerSecond(MAX_SPEED_MM_S * stepsPerMm);
        steppers[i].setAccelerationInStepsPerSecondPerSecond(MAX_ACCEL_MM_S2 * stepsPerMm);
    }
    settingsUpdateRequested = false;
}

int MotionController::getStepsPerMm(int microsteps) {
    return 25 * microsteps;
}

void MotionController::loop() {
    // Handle settings update
    if(settingsUpdateRequested && !isMoving()) {
        updateSettings();
    }

    // Handle motion
    if(moving) {
        bool allComplete = true;
        for(int i=0; i<4; i++) {
            steppers[i].processMovement();
            if(!steppers[i].motionComplete()) allComplete = false;
        }

        if(allComplete) {
            moving = false;
            digitalWrite(ENABLE_PIN, HIGH); // Disable
        }
    }
}

bool MotionController::homeAll() {
    homed = false;

    digitalWrite(ENABLE_PIN, LOW); // Enable drivers

    // 1. Move fast towards home (assuming negative direction is home) until stall
    long direction = -1;
    float homingDist = 300.0; // mm, sufficient to reach end

    for(int i=0; i<4; i++) {
        steppers[i].setCurrentPositionInSteps(0);
        steppers[i].setupRelativeMoveInMillimeters(homingDist * direction);
        steppers[i].setSpeedInStepsPerSecond(steppers[i].getStepsPerSecond() / 2); // Homing speed

        // StallGuard Masking: We need to ignore DIAG for a bit.
        // We'll handle masking in the loop by checking distance moved or time.
    }

    for(int i=0; i<4; i++) {
        pinMode(axisConfigs[i].limitPin, INPUT);
    }

    bool anyMoving = true;
    bool stalled[4] = {false, false, false, false};
    unsigned long startTime = millis();

    while(anyMoving) {
        anyMoving = false;
        yield(); // Feed Watchdog
        for(int i=0; i<4; i++) {
            if(!stalled[i]) {
                // Masking: Ignore first 200ms of motion
                bool isMasked = (millis() - startTime < 200);

                if(!isMasked && digitalRead(axisConfigs[i].limitPin) == HIGH) {
                    stalled[i] = true;
                } else {
                    if(!steppers[i].processMovement()) {
                        stalled[i] = true;
                    } else {
                        anyMoving = true;
                    }
                }
            }
        }
    }

    // 2. Back off
    for(int i=0; i<4; i++) {
        steppers[i].setCurrentPositionInSteps(0);
        steppers[i].setupRelativeMoveInMillimeters(10.0); // Back off 10mm
    }

    anyMoving = true;
    while(anyMoving) {
        anyMoving = false;
        yield(); // Feed Watchdog
        for(int i=0; i<4; i++) {
            if(steppers[i].processMovement()) {
                anyMoving = true;
            }
        }
    }

    // 3. Move slow towards home
    startTime = millis();
    for(int i=0; i<4; i++) {
        stalled[i] = false;
        steppers[i].setCurrentPositionInSteps(0);
        steppers[i].setupRelativeMoveInMillimeters(20.0 * direction);
        steppers[i].setSpeedInStepsPerSecond(steppers[i].getStepsPerSecond() / 4); // Slower
    }

    anyMoving = true;
    while(anyMoving) {
        anyMoving = false;
        yield(); // Feed Watchdog
        for(int i=0; i<4; i++) {
            if(!stalled[i]) {
                bool isMasked = (millis() - startTime < 100);

                if(!isMasked && digitalRead(axisConfigs[i].limitPin) == HIGH) {
                    stalled[i] = true;
                } else {
                    if(!steppers[i].processMovement()) {
                        stalled[i] = true;
                    } else {
                        anyMoving = true;
                    }
                }
            }
        }
    }

    // Zero positions
    for(int i=0; i<4; i++) {
        steppers[i].setCurrentPositionInSteps(0);
        StepperSettings s = settings.getStepperSettings(i);
        float stepsPerMm = (float)(25 * s.microsteps);
        steppers[i].setSpeedInStepsPerSecond(MAX_SPEED_MM_S * stepsPerMm);
    }

    homed = true;
    currentPosition = 0.0;

    digitalWrite(ENABLE_PIN, HIGH); // Disable
    return true;
}

void MotionController::moveAllTo(float position_mm) {
    if(!homed) return;

    float maxTravel = settings.getMaxTravel();
    if(position_mm < 0) position_mm = 0;
    if(position_mm > maxTravel) position_mm = maxTravel;

    digitalWrite(ENABLE_PIN, LOW);

    for(int i=0; i<4; i++) {
        steppers[i].setupMoveInMillimeters(position_mm);
    }

    currentPosition = position_mm;
    moving = true;
}

void MotionController::moveAllTo(String waypointName) {
    float pos = settings.getWaypointPosition(waypointName);
    if(pos >= 0) {
        moveAllTo(pos);
    }
}

void MotionController::stop() {
    // Hard stop
     for(int i=0; i<4; i++) {
         // SpeedyStepper doesn't have a clean stop() during motion easily without re-setup
         // But we can just disable enable pin
     }
     digitalWrite(ENABLE_PIN, HIGH);
}

bool MotionController::isHomed() { return homed; }
float MotionController::getCurrentPosition() { return currentPosition; }
bool MotionController::isMoving() { return !digitalRead(ENABLE_PIN); } // Simplified check

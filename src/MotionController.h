#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include <SpeedyStepper.h>
#include <TMCStepper.h>
#include "Settings.h"

// Define constants from user request
#define SERIAL_PORT   Serial2
#define RX_PIN        16
#define TX_PIN        17
#define ENABLE_PIN    4

struct AxisConfig {
    int stepPin;
    int dirPin;
    int limitPin; // Connected to DIAG
    uint8_t address;
    float r_sense;
};

class MotionController {
public:
    MotionController();
    void begin();
    void loop(); // If needed

    // Commands
    bool homeAll(); // Blocking homing with StallGuard
    void moveAllTo(float position_mm); // Non-blocking trigger
    void moveAllTo(String waypointName);
    void stop();

    // Status
    bool isHomed();
    float getCurrentPosition();
    bool isMoving();

    // Reconfigure (e.g. after settings change)
    void updateSettings();
    void requestUpdateSettings(); // Flag to update settings in loop

private:
    void setupDriver(int axisIndex);

    // 4 axes: X, Y, Z, A
    SpeedyStepper steppers[4];
    TMC2209Stepper* drivers[4]; // Dynamic allocation because we need to pass Serial

    AxisConfig axisConfigs[4];

    bool homed = false;
    float currentPosition = 0.0;

    bool settingsUpdateRequested = false;
    bool moving = false;

    const int STEPS_PER_MM = 64 * 25; // Default assumption 64 microsteps * 25 steps/mm (from original code 16 * 25, so scaled)
    // Original code: MICROSTEPS = 16, STEPS_PER_MM = 16 * 25 = 400.
    // If we change to 64 microsteps: STEPS_PER_MM = 64 * 25 = 1600.
    // However, the physical mechanics define steps/mm. If 16 microsteps gave 400 steps/mm,
    // then 1 full step = 400 / 16 = 25 steps/mm.
    // So if we use 64 microsteps, steps/mm = 25 * 64 = 1600.
    // This needs to be calculated dynamically based on configured microsteps.

    int getStepsPerMm(int microsteps);
};

extern MotionController motion;

#endif

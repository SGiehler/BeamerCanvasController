#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <MQTT.h>
#include <../lib/MultiHomeSpeedyStepper/MultiHomeSpeedyStepper.h>

#pragma region Constants
const char* ssid      = "xxx";
const char* pass      = "xxx";
const char* hostname  = "xxx";

const char* mqtt_host     = "xxx";
const char* mqtt_clientid = "xxx";
const char* mqtt_user     = "xxx";
const char* mqtt_pw       = "xxx";

const int MICROSTEPS = 16;
const int MAX_SPEED = MICROSTEPS * 300;
const int MAX_ACCELERATION = MICROSTEPS * 150;
const int STEPS_PER_MM = MICROSTEPS * 25;
const int AXIS_LENGTH = 180;

const int ENABLE_PIN = 12;

const int X_STEP_PIN = 26;
const int X_DIR_PIN = 16;
const int X_LIMIT_PIN = 13;

const int Y_STEP_PIN = 25;
const int Y_DIR_PIN = 27;
const int Y_LIMIT_PIN = 5;

const int Z_STEP_PIN = 17;
const int Z_DIR_PIN = 14;
const int Z_LIMIT_PIN = 23;

const int A_STEP_PIN = 19;
const int A_DIR_PIN = 18;
const int A_LIMIT_PIN = 4; // Feed Hold

#pragma endregion

WiFiClient net;
MQTTClient client;

MultiHomeSpeedyStepper stepperX;
MultiHomeSpeedyStepper stepperY;
MultiHomeSpeedyStepper stepperZ;
MultiHomeSpeedyStepper stepperA;

MultiHomeSpeedyStepper* steppers[4] = {&stepperX, &stepperY, &stepperZ, &stepperA};

StaticJsonDocument<200> doc;
char output[128];

unsigned long lastMillis = 0;
bool axis_homed = false;
float current_position = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: "); 
  Serial.println(WiFi.dnsIP());
  
  Serial.print("\nconnecting...");
  while (!client.connect(mqtt_clientid, mqtt_user, mqtt_pw)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");
    
  client.subscribe("/beamershutter/move");   
  client.subscribe("/beamershutter/home");
}

void reportWifiStatus() {
  doc.clear();
  doc["ip"]      = WiFi.localIP().toString();
  doc["mac"]     = WiFi.macAddress();
  doc["gateway"] = WiFi.gatewayIP().toString();
  doc["dns"]     = WiFi.dnsIP().toString();
  serializeJson(doc, output);

  client.publish("/beamershutter/wifi_status", output);
}

bool homeAll () {
  axis_homed = false;
  long directionTowardHome = -1;
  bool did_operation;

  for(MultiHomeSpeedyStepper* stepper: steppers) {
    // Set homing Speed
    stepper->setSpeedInStepsPerSecond(MAX_SPEED / 2);

    // Set AXIS_LENGTH for homing
    stepper->setupRelativeMoveInMillimeters(AXIS_LENGTH * directionTowardHome);
  }

  // Drive each axis into limit switch
  digitalWrite(ENABLE_PIN, LOW);
  do {

    did_operation = false;

    for(MultiHomeSpeedyStepper* stepper: steppers) {
      if(!stepper->limitSwitchActivated()) {
        if(!stepper->processMovement()) {
          did_operation = true;
        }
      }
    }

  } while(did_operation);
  digitalWrite(ENABLE_PIN, HIGH);  
  //
  // check if switch never detected
  //
  if (!stepperX.limitSwitchActivated() || !stepperY.limitSwitchActivated()
  || !stepperZ.limitSwitchActivated() || !stepperA.limitSwitchActivated()) {
    Serial.println("homing failed: towards limit switch");
    return(false);
  }
 
  delay(100);


  // Back off the limit switch
  for(MultiHomeSpeedyStepper* stepper: steppers) {
    // Set AXIS_LENGTH for homing
    stepper->setupRelativeMoveInMillimeters(AXIS_LENGTH * directionTowardHome * -1);
  }

  // Drive each axis into limit switch
  digitalWrite(ENABLE_PIN, LOW);
  do {

    did_operation = false;

    for(MultiHomeSpeedyStepper* stepper: steppers) {
      if(stepper->limitSwitchActivated()) {
        if(!stepper->processMovement()) {
          did_operation = true;
        }
      }
    }

  } while(did_operation);
  digitalWrite(ENABLE_PIN, HIGH);

  //
  // check if switch never detected
  //
  if (stepperX.limitSwitchActivated() || stepperY.limitSwitchActivated()
  || stepperZ.limitSwitchActivated() || stepperA.limitSwitchActivated()) {
    Serial.println("homing failed: back-off limit switch");
    return(false);
  }

  delay(100);


  for(MultiHomeSpeedyStepper* stepper: steppers) {
    // Set homing Speed
    stepper->setSpeedInStepsPerSecond(MAX_SPEED / 4);

    // Set AXIS_LENGTH for homing
    stepper->setupRelativeMoveInMillimeters(AXIS_LENGTH * directionTowardHome);
  }

  // Drive each axis into limit switch but slow
  digitalWrite(ENABLE_PIN, LOW);
  do {

    did_operation = false;

    for(MultiHomeSpeedyStepper* stepper: steppers) {
      if(!stepper->limitSwitchActivated()) {
        if(!stepper->processMovement()) {
          did_operation = true;
        }
      }
    }

  } while(did_operation);
  digitalWrite(ENABLE_PIN, HIGH);

  //
  // check if switch never detected
  //
  if (!stepperX.limitSwitchActivated() || !stepperY.limitSwitchActivated()
  || !stepperZ.limitSwitchActivated() || !stepperA.limitSwitchActivated()) {
    Serial.println("homing failed: towards limit switch (slower)");
    return(false);
  }
 
  delay(25);

  for(MultiHomeSpeedyStepper* stepper: steppers) {
    //
    // successfully homed, set the current position to 0
    //    
    stepper->setCurrentPositionInSteps(0L);
    stepper->setSpeedInStepsPerSecond(MAX_SPEED);
  }  

  axis_homed = true;
  return(true);
}

void moveAll(float requested_location) {
    digitalWrite(ENABLE_PIN, LOW);
    float location = requested_location;
    if(requested_location > AXIS_LENGTH) location = AXIS_LENGTH;
    if(requested_location < 0) location = 0;

    for(MultiHomeSpeedyStepper* stepper: steppers) {
      stepper->setupMoveInMillimeters(location);
    }

    while(!stepperX.motionComplete() || !stepperY.motionComplete() || !stepperZ.motionComplete() || !stepperA.motionComplete())
    {
      stepperX.processMovement();
      stepperY.processMovement();
      stepperZ.processMovement();
      stepperA.processMovement();      
    }
    digitalWrite(ENABLE_PIN, HIGH);
    current_position = location;
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  if(topic.startsWith("/beamershutter/move") && axis_homed) {
    moveAll(payload.toFloat());
  }
  else if(topic.startsWith("/beamershutter/home")) {
    while(!homeAll()){ 
      delay(500); 
    };
  }
}

void setup() {
  Serial.begin(115200);

  // Disable Motors
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  WiFi.setHostname(hostname);

  client.begin(mqtt_host, net);
  client.onMessage(messageReceived);

  stepperX.connectToPins(X_STEP_PIN, X_DIR_PIN, X_LIMIT_PIN);
  stepperX.setSpeedInStepsPerSecond(MAX_SPEED);
  stepperX.setStepsPerMillimeter(STEPS_PER_MM);
  stepperX.setAccelerationInStepsPerSecondPerSecond(MAX_ACCELERATION);

  stepperY.connectToPins(Y_STEP_PIN, Y_DIR_PIN, Y_LIMIT_PIN);
  stepperY.setSpeedInStepsPerSecond(MAX_SPEED);
  stepperY.setStepsPerMillimeter(STEPS_PER_MM);
  stepperY.setAccelerationInStepsPerSecondPerSecond(MAX_ACCELERATION);

  stepperZ.connectToPins(Z_STEP_PIN, Z_DIR_PIN, Z_LIMIT_PIN);
  stepperZ.setSpeedInStepsPerSecond(MAX_SPEED);
  stepperZ.setStepsPerMillimeter(STEPS_PER_MM);
  stepperZ.setAccelerationInStepsPerSecondPerSecond(MAX_ACCELERATION);

  stepperA.connectToPins(A_STEP_PIN, A_DIR_PIN, A_LIMIT_PIN);
  stepperA.setSpeedInStepsPerSecond(MAX_SPEED);
  stepperA.setStepsPerMillimeter(STEPS_PER_MM);
  stepperA.setAccelerationInStepsPerSecondPerSecond(MAX_ACCELERATION);
}

void loop() {
  client.loop(); // MQTT Loop
  delay(10);  // <- fixes some issues with WiFi stability

  // Reconnect of connection was dropped
  if (!client.connected()) {
    if(WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);
    }
    connect();

    reportWifiStatus();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    doc.clear();
    doc["current_position"] = current_position;
    doc["axis_homed"]     = axis_homed;
    serializeJson(doc, output);

    client.publish("/beamershutter/motion_status", output);    
  }

}
 
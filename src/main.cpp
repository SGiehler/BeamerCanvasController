#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "Settings.h"
#include "MotionController.h"
#include "WebHandler.h"

// Command Queue
enum CommandType { CMD_NONE, CMD_HOME, CMD_MOVE, CMD_STOP };
struct Command {
    CommandType type;
    float value; // for move
};

QueueHandle_t commandQueue;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastStatusTime = 0;

void setup() {
    Serial.begin(115200);

    commandQueue = xQueueCreate(10, sizeof(Command));

    settings.begin();
    motion.begin();
    webHandler.begin();

    // MQTT Setup
    if(settings.getMqttHost().length() > 0) {
        mqttClient.setServer(settings.getMqttHost().c_str(), settings.getMqttPort());
        // Callbacks...
        mqttClient.setCallback([](char* topic, byte* payload, unsigned int length) {
            String msg;
            for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
            Serial.print("MQTT: "); Serial.println(topic);

            Command cmd = {CMD_NONE, 0};
            if(String(topic).endsWith("/move")) {
                cmd = {CMD_MOVE, msg.toFloat()};
            } else if (String(topic).endsWith("/home")) {
                cmd = {CMD_HOME, 0};
            } else if (String(topic).endsWith("/stop")) {
                cmd = {CMD_STOP, 0};
            }
            if(cmd.type != CMD_NONE) xQueueSend(commandQueue, &cmd, 0);
        });
    }
}

void handleMqtt() {
    if(settings.getMqttHost().length() == 0) return;

    if (!mqttClient.connected()) {
        static unsigned long lastReconnectAttempt = 0;
        unsigned long now = millis();
        if (now - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = now;
            if(WiFi.status() == WL_CONNECTED) {
                if (mqttClient.connect(settings.getMqttClientId().c_str(), settings.getMqttUser().c_str(), settings.getMqttPass().c_str())) {
                    Serial.println("MQTT Connected");
                    mqttClient.subscribe("beamershutter/move");
                    mqttClient.subscribe("beamershutter/home");
                    mqttClient.subscribe("beamershutter/stop");
                }
            }
        }
    } else {
        mqttClient.loop();
    }
}

void publishStatus() {
    if(!mqttClient.connected()) return;

    JsonDocument doc;
    doc["position"] = motion.getCurrentPosition();
    doc["homed"] = motion.isHomed();
    doc["moving"] = motion.isMoving();

    String output;
    serializeJson(doc, output);
    mqttClient.publish("beamershutter/status", output.c_str());
}

void loop() {
    webHandler.loop();
    handleMqtt();
    motion.loop();

    // Check for commands
    Command cmd;
    if(xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
        switch(cmd.type) {
            case CMD_HOME:
                Serial.println("Executing Home");
                motion.homeAll();
                break;
            case CMD_MOVE:
                Serial.println("Executing Move to " + String(cmd.value));
                motion.moveAllTo(cmd.value);
                break;
            case CMD_STOP:
                 motion.stop();
                 break;
            default: break;
        }
        publishStatus();
    }

    if(millis() - lastStatusTime > 2000) {
        lastStatusTime = millis();
        publishStatus();
    }
}

#include "Settings.h"

Settings settings;

void Settings::begin() {
    mutex = xSemaphoreCreateMutex();
    prefs.begin("blinds_cfg", false);
    load();
}

void Settings::load() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    wifi_ssid = prefs.getString("ssid", "");
    wifi_pass = prefs.getString("pass", "");

    mqtt_host = prefs.getString("mq_host", "");
    mqtt_port = prefs.getInt("mq_port", 1883);
    mqtt_user = prefs.getString("mq_user", "");
    mqtt_pass = prefs.getString("mq_pass", "");
    mqtt_client_id = prefs.getString("mq_id", "esp32_blinds");

    // Load Stepper Settings
    // Default values if not set
    // X: SG=4, Curr=600, Micro=64
    // Y: SG=1, Curr=600, Micro=64
    // Z: SG=4, Curr=600, Micro=64
    // A: SG=4, Curr=600, Micro=64

    int defaultSg[4] = {4, 1, 4, 4};

    for(int i=0; i<4; i++) {
        String key = "st_" + String(i);
        stepperSettings[i].stallGuardThreshold = prefs.getInt((key + "_sg").c_str(), defaultSg[i]);
        stepperSettings[i].current = prefs.getInt((key + "_cur").c_str(), 600);
        stepperSettings[i].microsteps = prefs.getInt((key + "_ms").c_str(), 64);
    }

    maxTravel = prefs.getFloat("max_len", 180.0);

    // Load Waypoints
    // We'll store waypoints as a JSON string for simplicity because Preferences doesn't support arrays nicely
    String wpJson = prefs.getString("waypoints", "[]");
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, wpJson);
    JsonArray arr = doc.as<JsonArray>();

    waypoints.clear();
    for(JsonObject obj : arr) {
        Waypoint wp;
        wp.name = obj["n"].as<String>();
        wp.position = obj["p"].as<float>();
        waypoints.push_back(wp);
    }
    xSemaphoreGive(mutex);
}

String Settings::getWifiSSID() { xSemaphoreTake(mutex, portMAX_DELAY); String s = wifi_ssid; xSemaphoreGive(mutex); return s; }
void Settings::setWifiSSID(String ssid) { xSemaphoreTake(mutex, portMAX_DELAY); wifi_ssid = ssid; prefs.putString("ssid", ssid); xSemaphoreGive(mutex); }

String Settings::getWifiPass() { xSemaphoreTake(mutex, portMAX_DELAY); String s = wifi_pass; xSemaphoreGive(mutex); return s; }
void Settings::setWifiPass(String pass) { xSemaphoreTake(mutex, portMAX_DELAY); wifi_pass = pass; prefs.putString("pass", pass); xSemaphoreGive(mutex); }

String Settings::getMqttHost() { xSemaphoreTake(mutex, portMAX_DELAY); String s = mqtt_host; xSemaphoreGive(mutex); return s; }
void Settings::setMqttHost(String host) { xSemaphoreTake(mutex, portMAX_DELAY); mqtt_host = host; prefs.putString("mq_host", host); xSemaphoreGive(mutex); }

int Settings::getMqttPort() { xSemaphoreTake(mutex, portMAX_DELAY); int i = mqtt_port; xSemaphoreGive(mutex); return i; }
void Settings::setMqttPort(int port) { xSemaphoreTake(mutex, portMAX_DELAY); mqtt_port = port; prefs.putInt("mq_port", port); xSemaphoreGive(mutex); }

String Settings::getMqttUser() { xSemaphoreTake(mutex, portMAX_DELAY); String s = mqtt_user; xSemaphoreGive(mutex); return s; }
void Settings::setMqttUser(String user) { xSemaphoreTake(mutex, portMAX_DELAY); mqtt_user = user; prefs.putString("mq_user", user); xSemaphoreGive(mutex); }

String Settings::getMqttPass() { xSemaphoreTake(mutex, portMAX_DELAY); String s = mqtt_pass; xSemaphoreGive(mutex); return s; }
void Settings::setMqttPass(String pass) { xSemaphoreTake(mutex, portMAX_DELAY); mqtt_pass = pass; prefs.putString("mq_pass", pass); xSemaphoreGive(mutex); }

String Settings::getMqttClientId() { xSemaphoreTake(mutex, portMAX_DELAY); String s = mqtt_client_id; xSemaphoreGive(mutex); return s; }
void Settings::setMqttClientId(String clientId) { xSemaphoreTake(mutex, portMAX_DELAY); mqtt_client_id = clientId; prefs.putString("mq_id", clientId); xSemaphoreGive(mutex); }

StepperSettings Settings::getStepperSettings(int axisIndex) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    if(axisIndex < 0 || axisIndex > 3) { xSemaphoreGive(mutex); return {0,0,0}; }
    StepperSettings s = stepperSettings[axisIndex];
    xSemaphoreGive(mutex);
    return s;
}

void Settings::setStepperSettings(int axisIndex, StepperSettings s) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    if(axisIndex < 0 || axisIndex > 3) { xSemaphoreGive(mutex); return; }
    stepperSettings[axisIndex] = s;
    String key = "st_" + String(axisIndex);
    prefs.putInt((key + "_sg").c_str(), s.stallGuardThreshold);
    prefs.putInt((key + "_cur").c_str(), s.current);
    prefs.putInt((key + "_ms").c_str(), s.microsteps);
    xSemaphoreGive(mutex);
}

float Settings::getMaxTravel() { xSemaphoreTake(mutex, portMAX_DELAY); float f = maxTravel; xSemaphoreGive(mutex); return f; }
void Settings::setMaxTravel(float mm) { xSemaphoreTake(mutex, portMAX_DELAY); maxTravel = mm; prefs.putFloat("max_len", mm); xSemaphoreGive(mutex); }

std::vector<Waypoint> Settings::getWaypoints() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    std::vector<Waypoint> v = waypoints;
    xSemaphoreGive(mutex);
    return v;
}

void Settings::addWaypoint(String name, float position) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    // Check if exists, update if so
    for(auto &wp : waypoints) {
        if(wp.name == name) {
            wp.position = position;
            save(); // Updates the JSON in prefs
            xSemaphoreGive(mutex);
            return;
        }
    }
    waypoints.push_back({name, position});

    // Save waypoints to prefs
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.to<JsonArray>();
    for(auto &wp : waypoints) {
        JsonObject obj = arr.createNestedObject();
        obj["n"] = wp.name;
        obj["p"] = wp.position;
    }
    String output;
    serializeJson(doc, output);
    prefs.putString("waypoints", output);
    xSemaphoreGive(mutex);
}

void Settings::removeWaypoint(String name) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    for(auto it = waypoints.begin(); it != waypoints.end(); ++it) {
        if(it->name == name) {
            waypoints.erase(it);
            break;
        }
    }
     // Save waypoints to prefs
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.to<JsonArray>();
    for(auto &wp : waypoints) {
        JsonObject obj = arr.createNestedObject();
        obj["n"] = wp.name;
        obj["p"] = wp.position;
    }
    String output;
    serializeJson(doc, output);
    prefs.putString("waypoints", output);
    xSemaphoreGive(mutex);
}

void Settings::clearWaypoints() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    waypoints.clear();
    prefs.putString("waypoints", "[]");
    xSemaphoreGive(mutex);
}

float Settings::getWaypointPosition(String name) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    for(auto &wp : waypoints) {
        if(wp.name == name) {
            float p = wp.position;
            xSemaphoreGive(mutex);
            return p;
        }
    }
    xSemaphoreGive(mutex);
    return -1.0;
}

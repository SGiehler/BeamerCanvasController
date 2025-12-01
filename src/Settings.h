#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>

struct StepperSettings {
    int stallGuardThreshold;
    int current; // mA
    int microsteps;
};

struct Waypoint {
    String name;
    float position; // mm
};

class Settings {
public:
    void begin();

    // WiFi
    String getWifiSSID();
    void setWifiSSID(String ssid);
    String getWifiPass();
    void setWifiPass(String pass);

    // MQTT
    String getMqttHost();
    void setMqttHost(String host);
    int getMqttPort();
    void setMqttPort(int port);
    String getMqttUser();
    void setMqttUser(String user);
    String getMqttPass();
    void setMqttPass(String pass);
    String getMqttClientId();
    void setMqttClientId(String clientId);

    // Stepper
    StepperSettings getStepperSettings(int axisIndex); // 0=X, 1=Y, 2=Z, 3=A
    void setStepperSettings(int axisIndex, StepperSettings settings);

    // Max Travel
    float getMaxTravel();
    void setMaxTravel(float mm);

    // Waypoints
    std::vector<Waypoint> getWaypoints();
    void addWaypoint(String name, float position);
    void removeWaypoint(String name);
    void clearWaypoints();
    float getWaypointPosition(String name); // Returns -1 if not found

    // Load/Save helpers
    void load(); // Loads from preferences
    void save(); // Saves to preferences (usually done in setters)

    SemaphoreHandle_t mutex;

private:
    Preferences prefs;

    String wifi_ssid;
    String wifi_pass;

    String mqtt_host;
    int mqtt_port;
    String mqtt_user;
    String mqtt_pass;
    String mqtt_client_id;

    float maxTravel;

    StepperSettings stepperSettings[4];
    std::vector<Waypoint> waypoints;
};

extern Settings settings;

#endif

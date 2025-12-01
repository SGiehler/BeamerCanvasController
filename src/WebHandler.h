#ifndef WEB_HANDLER_H
#define WEB_HANDLER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ElegantOTA.h>
#include "Settings.h"
#include "MotionController.h"

class WebHandler {
public:
    void begin();
    void loop(); // for DNS server if AP mode, or periodic status updates

private:
    void setupEndpoints();
    void connectWifi();

    AsyncWebServer server;
    bool apMode = false;
};

extern WebHandler webHandler;

#endif

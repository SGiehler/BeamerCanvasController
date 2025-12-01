#include "WebHandler.h"

// Define external command structure
enum CommandType { CMD_NONE, CMD_HOME, CMD_MOVE, CMD_STOP };
struct Command {
    CommandType type;
    float value; // for move
};
extern QueueHandle_t commandQueue;

WebHandler webHandler;

// HTML Content (Minified or simple)
const char* index_html = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Blinds Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin:0; padding:20px;}
    .card { background: #eee; padding: 20px; margin: 10px auto; max-width: 500px; border-radius: 8px;}
    button { padding: 10px 20px; font-size: 16px; margin: 5px; cursor: pointer;}
    input { padding: 8px; margin: 5px; width: 80%;}
    .status { font-weight: bold; color: green; }
    h2 { margin-top: 0; }
  </style>
</head>
<body>
  <h1>Blinds Control</h1>

  <div class="card">
    <h2>Status</h2>
    <p>Position: <span id="pos">0.0</span> mm</p>
    <p>Homed: <span id="homed">No</span></p>
    <p>State: <span id="state">Idle</span></p>
  </div>

  <div class="card">
    <h2>Control</h2>
    <button onclick="home()">Home All</button>
    <br><br>
    <input type="number" id="targetPos" placeholder="Position (mm)">
    <button onclick="move()">Move</button>
    <br>
    <h3>Waypoints</h3>
    <div id="waypoints"></div>
  </div>

  <div class="card">
    <h2>Settings</h2>
    <button onclick="location.href='/settings'">Configure</button>
  </div>

<script>
function fetchStatus() {
    fetch('/api/status').then(res => res.json()).then(data => {
        document.getElementById('pos').innerText = data.position;
        document.getElementById('homed').innerText = data.homed ? "Yes" : "No";
        document.getElementById('state').innerText = data.moving ? "Moving" : "Idle";

        let wpHtml = "";
        data.waypoints.forEach(wp => {
            wpHtml += `<button onclick="moveTo('${wp.name}')">${wp.name} (${wp.position}mm)</button>`;
        });
        document.getElementById('waypoints').innerHTML = wpHtml;
    });
}
function home() { fetch('/api/home', {method:'POST'}); }
function move() {
    let p = document.getElementById('targetPos').value;
    fetch('/api/move?pos='+p, {method:'POST'});
}
function moveTo(name) {
    fetch('/api/move?name='+name, {method:'POST'});
}
setInterval(fetchStatus, 1000);
fetchStatus();
</script>
</body>
</html>
)rawliteral";

const char* settings_html = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Blinds Settings</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin:0; padding:20px;}
    .card { background: #eee; padding: 20px; margin: 10px auto; max-width: 500px; border-radius: 8px; text-align: left;}
    label { display: block; margin-top: 10px; font-weight: bold;}
    input { padding: 8px; margin: 5px 0; width: 100%; box-sizing: border-box;}
    button { padding: 10px 20px; font-size: 16px; margin: 15px 0; cursor: pointer; width: 100%;}
  </style>
</head>
<body>
  <h1>Settings</h1>
  <button onclick="location.href='/'">Back</button>

  <form action="/api/save_wifi" method="POST" class="card">
    <h2>WiFi</h2>
    <label>SSID</label><input type="text" name="ssid" id="ssid">
    <label>Password</label><input type="password" name="pass" id="pass">
    <button type="submit">Save WiFi</button>
  </form>

  <form action="/api/save_mqtt" method="POST" class="card">
    <h2>MQTT</h2>
    <label>Host</label><input type="text" name="host" id="mq_host">
    <label>Port</label><input type="number" name="port" id="mq_port">
    <label>User</label><input type="text" name="user" id="mq_user">
    <label>Pass</label><input type="password" name="pass" id="mq_pass">
    <label>Client ID</label><input type="text" name="id" id="mq_id">
    <button type="submit">Save MQTT</button>
  </form>

  <form action="/api/save_max_travel" method="POST" class="card">
      <h2>Limits</h2>
      <label>Max Travel (mm)</label><input type="number" name="max_len" id="max_len" step="1">
      <button type="submit">Save Limits</button>
  </form>

  <div class="card">
    <h2>Steppers</h2>
    <div id="stepper_forms"></div>
  </div>

  <div class="card">
      <h2>Add Waypoint</h2>
      <form action="/api/add_waypoint" method="POST">
          <label>Name</label><input type="text" name="name">
          <label>Position (mm)</label><input type="number" name="pos" step="0.1">
          <button type="submit">Add</button>
      </form>
  </div>

  <div class="card">
      <h2>OTA Update</h2>
      <button type="button" onclick="location.href='/update'">Go to Update Page</button>
  </div>

<script>
const axisNames = ["X", "Y", "Z", "A"];

function renderStepperForms(steppersData) {
    let html = "";
    steppersData.forEach((st, i) => {
        html += `
        <form action="/api/save_stepper" method="POST" style="border-top: 1px solid #ccc; padding-top: 10px;">
            <input type="hidden" name="axis" value="${i}">
            <h3>Axis ${axisNames[i]}</h3>
            <label>SG Threshold</label><input type="number" name="sg" value="${st.sg}">
            <label>Current (mA)</label><input type="number" name="cur" value="${st.cur}">
            <label>Microsteps</label><input type="number" name="ms" value="${st.ms}">
            <button type="submit">Save ${axisNames[i]}</button>
        </form>
        `;
    });
    document.getElementById('stepper_forms').innerHTML = html;
}

// Fetch current settings to populate fields
fetch('/api/settings').then(res => res.json()).then(data => {
    document.getElementById('ssid').value = data.wifi.ssid;
    document.getElementById('mq_host').value = data.mqtt.host;
    document.getElementById('mq_port').value = data.mqtt.port;
    document.getElementById('mq_user').value = data.mqtt.user;
    document.getElementById('mq_id').value = data.mqtt.id;

    document.getElementById('max_len').value = data.max_len;

    renderStepperForms(data.steppers);
});
</script>
</body>
</html>
)rawliteral";

WebHandler::WebHandler() : server(80) {}

void WebHandler::begin() {
    connectWifi();
    setupEndpoints();
    ElegantOTA.begin(&server);
    server.begin();
}

void WebHandler::connectWifi() {
    String ssid = settings.getWifiSSID();
    String pass = settings.getWifiPass();

    if(ssid.length() > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        Serial.print("Connecting to WiFi");
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        if(WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected. IP: " + WiFi.localIP().toString());
            apMode = false;
            return;
        }
    }

    // Fallback to AP
    Serial.println("\nStarting AP Mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Blinds-Setup", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    apMode = true;
}

void WebHandler::setupEndpoints() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", index_html);
    });

    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", settings_html);
    });

    // API Status
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        DynamicJsonDocument doc(1024);
        doc["position"] = motion.getCurrentPosition();
        doc["homed"] = motion.isHomed();
        doc["moving"] = motion.isMoving();

        JsonArray wpArr = doc.createNestedArray("waypoints");
        std::vector<Waypoint> wps = settings.getWaypoints();
        for(auto &wp : wps) {
            JsonObject obj = wpArr.createNestedObject();
            obj["name"] = wp.name;
            obj["position"] = wp.position;
        }

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // API Control
    server.on("/api/home", HTTP_POST, [](AsyncWebServerRequest *request){
        Command cmd = {CMD_HOME, 0};
        xQueueSend(commandQueue, &cmd, 0);
        request->send(200, "text/plain", "Homing started");
    });

    server.on("/api/move", HTTP_POST, [](AsyncWebServerRequest *request){
        Command cmd = {CMD_NONE, 0};
        if(request->hasParam("pos")) {
             float pos = request->getParam("pos")->value().toFloat();
             cmd = {CMD_MOVE, pos};
        } else if (request->hasParam("name")) {
             String name = request->getParam("name")->value();
             float pos = settings.getWaypointPosition(name);
             if(pos >= 0) cmd = {CMD_MOVE, pos};
        }

        if(cmd.type != CMD_NONE) {
            xQueueSend(commandQueue, &cmd, 0);
        }

        request->send(200, "text/plain", "Move requested");
    });

    // API Settings GET
    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request){
        DynamicJsonDocument doc(1024);

        JsonObject wifi = doc.createNestedObject("wifi");
        wifi["ssid"] = settings.getWifiSSID();

        JsonObject mqtt = doc.createNestedObject("mqtt");
        mqtt["host"] = settings.getMqttHost();
        mqtt["port"] = settings.getMqttPort();
        mqtt["user"] = settings.getMqttUser();
        mqtt["id"] = settings.getMqttClientId();

        doc["max_len"] = settings.getMaxTravel();

        JsonArray stArr = doc.createNestedArray("steppers");
        for(int i=0; i<4; i++) {
            JsonObject st = stArr.createNestedObject();
            StepperSettings s = settings.getStepperSettings(i);
            st["sg"] = s.stallGuardThreshold;
            st["cur"] = s.current;
            st["ms"] = s.microsteps;
        }

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // API Save WiFi
    server.on("/api/save_wifi", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("ssid", true) && request->hasParam("pass", true)) {
            settings.setWifiSSID(request->getParam("ssid", true)->value());
            settings.setWifiPass(request->getParam("pass", true)->value());
            request->send(200, "text/plain", "Saved. Rebooting...");
            delay(1000);
            ESP.restart();
        } else {
            request->send(400, "text/plain", "Missing params");
        }
    });

    // API Save MQTT
    server.on("/api/save_mqtt", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("host", true)) settings.setMqttHost(request->getParam("host", true)->value());
        if(request->hasParam("port", true)) settings.setMqttPort(request->getParam("port", true)->value().toInt());
        if(request->hasParam("user", true)) settings.setMqttUser(request->getParam("user", true)->value());
        if(request->hasParam("pass", true)) settings.setMqttPass(request->getParam("pass", true)->value());
        if(request->hasParam("id", true)) settings.setMqttClientId(request->getParam("id", true)->value());

        request->redirect("/settings");
    });

    // API Save Stepper
    server.on("/api/save_stepper", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("axis", true)) {
            int axis = request->getParam("axis", true)->value().toInt();
            StepperSettings s = settings.getStepperSettings(axis);
            if(request->hasParam("sg", true)) s.stallGuardThreshold = request->getParam("sg", true)->value().toInt();
            if(request->hasParam("cur", true)) s.current = request->getParam("cur", true)->value().toInt();
            if(request->hasParam("ms", true)) s.microsteps = request->getParam("ms", true)->value().toInt();
            settings.setStepperSettings(axis, s);
            motion.requestUpdateSettings();
        }
        request->redirect("/settings");
    });

    // API Save Max Travel
    server.on("/api/save_max_travel", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("max_len", true)) {
            settings.setMaxTravel(request->getParam("max_len", true)->value().toFloat());
        }
        request->redirect("/settings");
    });

    // API Add Waypoint
    server.on("/api/add_waypoint", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("name", true) && request->hasParam("pos", true)) {
            settings.addWaypoint(request->getParam("name", true)->value(), request->getParam("pos", true)->value().toFloat());
        }
        request->redirect("/settings");
    });
}

void WebHandler::loop() {
    ElegantOTA.loop();
}

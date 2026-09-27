/*
  ======================================================================
   SMART FISHERMAN SAFETY & EMERGENCY ALERT SYSTEM (REAL IMBL EDITION)
   With 2km Proximity & Collision Alert Reception
  ======================================================================
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======================================================================
// PIN DEFINITIONS
// ======================================================================
#define BUZZER_PIN      25
#define SOS_BUTTON_PIN  26

#define GPS_RX_PIN      16   // ESP32 RX2 <- GPS TX
#define GPS_TX_PIN      17   // ESP32 TX2 -> GPS RX

#define GSM_RX_PIN      27   // ESP32 RX1 <- SIM800 TX
#define GSM_TX_PIN      14   // ESP32 TX1 -> SIM800 RX

// ======================================================================
// USER SETTINGS
// ======================================================================
const char* WIFI_SSID     = "Apple mn";          // 2.4 GHz network / phone hotspot
const char* WIFI_PASSWORD = "nadeem@1234";
String phoneNumber        = "+919865168850";      // Emergency SMS recipient number

// MQTT Dashboard Config
const char* MQTT_BROKER    = "broker.hivemq.com";
const int   MQTT_PORT      = 1883;
const char* MQTT_TOPIC     = "sentinel-line/triode_titans/hwboat";
const char* MQTT_CMD_TOPIC = "sentinel-line/triode_titans/hwcmd";

// ======================================================================
// THRESHOLDS & TIMINGS
// ======================================================================
const float TILT_THRESHOLD_DEG  = 35.0;            // Threshold angle for capsize warning
const float TILT_HYSTERESIS_DEG = 5.0;             // Must fall 5 deg below threshold to clear alert
const unsigned long TILT_DEBOUNCE_MS = 2000;       // Tilt must persist for 2 continuous seconds

const float PRESSURE_THRESHOLD = 950.0;
const float BORDER_SAFE_M      = 2000.0;
const float BORDER_DANGER_M    = 500.0;

const unsigned long MQTT_PUBLISH_INTERVAL_MS = 3000;
const unsigned long WIFI_RETRY_INTERVAL_MS   = 5000;
const unsigned long MQTT_RETRY_INTERVAL_MS   = 3000;
const unsigned long LCD_REFRESH_MS           = 250;
const unsigned long SERIAL_PRINT_MS          = 1000;
const unsigned long GPS_FIX_TIMEOUT_MS       = 5000;
const unsigned long SCREEN_INTERVAL          = 2500;

// ======================================================================
// GLOBAL OBJECTS
// ======================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
MPU6050 mpu(0x68);
Adafruit_BMP280 bmp;

HardwareSerial gpsSerial(2);
HardwareSerial gsmSerial(1);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ======================================================================
// REAL-WORLD IMBL BOUNDARY DATA (1974 Palk Strait Treaty, Positions 1-6)
// ======================================================================
struct GPSPoint {
  double lat;
  double lon;
};

const int IMBL_POINT_COUNT = 6;
const GPSPoint IMBL_POINTS[IMBL_POINT_COUNT] = {
  {10.08333, 80.05000},
  {9.95000,  79.58333},
  {9.66917,  79.37667},
  {9.36333,  79.51167},
  {9.21667,  79.53333},
  {9.10000,  79.53333}
};

// ======================================================================
// GLOBAL STATE VARIABLES
// ======================================================================
int16_t ax, ay, az, gx, gy, gz;
float rollOffset = 0, pitchOffset = 0;
float currentRoll = 0, currentPitch = 0;

float temperature = 0;
float pressure = 1013.0;

double latitude = 0;
double longitude = 0;
float speedKnots = 0;
float courseDeg = 0;
int satellites = 0;
bool gpsLocked = false;

double distanceToBorder = 999999;

bool tiltDanger    = false;
bool weatherAlert  = false;
bool borderWarning = false;
bool borderDanger  = false;
bool sosActive     = false;

// Collision alerts received from dashboard
bool collisionCritical = false;
bool collisionCaution  = false;
String collisionTarget = "";
int collisionDistMeters = 0;

bool smsSentTilt   = false;
bool smsSentBorder = false;
bool smsSentSOS    = false;

bool lastButtonState = HIGH;

unsigned long lastScreenChange = 0;
int currentScreen = 0;
const int TOTAL_SCREENS = 5;

unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastTelemetryPublish = 0;
unsigned long lastLcdRefresh = 0;
unsigned long lastSerialPrint = 0;

enum Priority {
  PRI_NORMAL    = 0,
  PRI_WEATHER   = 1,
  PRI_BORDER    = 2,
  PRI_TILT      = 3,
  PRI_COLLISION = 4,
  PRI_SOS       = 5
};

// ======================================================================
// FORWARD DECLARATIONS
// ======================================================================
void showBootScreen();
void setupMPU();
void calibrateMPU();
void readMPUData();
void setupBMP();
void readBMPData();
void readGPSData();
void setupGSM();
String readGSMResponse();
void sendSMS(String message);
String buildLocationText();
void checkSOSButton();
void checkTiltCondition();
void checkBorderCondition();
void checkWeatherCondition();
void handleBuzzer();
void handleSMSLogic();
int getCurrentPriority();
void lcdLine(uint8_t row, const String& text);
void updateLCDDisplay();
void rotateNormalScreens();
void printSerialData();
void maintainWiFi();
void maintainMQTT();
void publishTelemetry();
void mqttCallback(char* topic, byte* payload, unsigned int length);
double distanceToSegmentMeters(double lat, double lon, double lat1, double lon1, double lat2, double lon2);
double getDistanceToIMBLMeters(double lat, double lon);

// ======================================================================
// SETUP
// ======================================================================
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(21, 22);

  lcd.init();
  lcd.backlight();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512); 

  showBootScreen();

  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  setupMPU();
  setupBMP();
  setupGSM();

  lcd.clear();
  lcdLine(0, "System Ready");
  delay(1500);
  lcd.clear();
}

// ======================================================================
// MAIN LOOP
// ======================================================================
void loop() {
  readGPSData();
  readMPUData();
  readBMPData();

  checkSOSButton();
  checkTiltCondition();
  checkBorderCondition();
  checkWeatherCondition();

  handleBuzzer();
  handleSMSLogic();

  if (millis() - lastLcdRefresh >= LCD_REFRESH_MS) {
    lastLcdRefresh = millis();
    updateLCDDisplay();
  }
  
  if (millis() - lastSerialPrint >= SERIAL_PRINT_MS) {
    lastSerialPrint = millis();
    printSerialData();
  }

  maintainWiFi();
  maintainMQTT();
  publishTelemetry();
}

// ======================================================================
// BOOT SCREEN
// ======================================================================
void showBootScreen() {
  lcd.clear();
  lcdLine(0, "Smart Fisherman");
  lcdLine(1, "Starting...");
  delay(1500);

  lcdLine(0, "Checking GPS...");
  delay(400);
  lcdLine(0, "Checking BMP...");
  delay(400);
  lcdLine(0, "Checking MPU...");
  delay(400);
  lcdLine(0, "Checking GSM...");
  delay(400);
}

// ======================================================================
// MPU6050
// ======================================================================
void setupMPU() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    calibrateMPU();
  }
}

void calibrateMPU() {
  const int samples = 200;
  float rollSum = 0, pitchSum = 0;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float p = atan2((float)ay, (float)ax) * 180.0 / PI;
    float r = atan2((float)az, sqrt((float)ax * ax + (float)ay * ay)) * 180.0 / PI;
    rollSum  += r;
    pitchSum += p;
    delay(5);
  }

  rollOffset  = rollSum / samples;
  pitchOffset = pitchSum / samples;

  Serial.print("MPU Calibrated. Roll Offset: ");
  Serial.print(rollOffset);
  Serial.print(" | Pitch Offset: ");
  Serial.println(pitchOffset);
}

void readMPUData() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitch = atan2((float)ay, (float)ax) * 180.0 / PI;
  float roll  = atan2((float)az, sqrt((float)ax * ax + (float)ay * ay)) * 180.0 / PI;

  currentRoll  = roll - rollOffset;
  currentPitch = pitch - pitchOffset;

  if (currentRoll > 180.0)   currentRoll -= 360.0;
  if (currentRoll < -180.0)  currentRoll += 360.0;
  if (currentPitch > 180.0)  currentPitch -= 360.0;
  if (currentPitch < -180.0) currentPitch += 360.0;
}

// ======================================================================
// BMP280
// ======================================================================
void setupBMP() {
  bool found = bmp.begin(0x76);
  if (!found) found = bmp.begin(0x77);

  if (!found) {
    Serial.println("BMP280 sensor not detected!");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
}

void readBMPData() {
  temperature = bmp.readTemperature();
  float p = bmp.readPressure() / 100.0F;
  if (p > 300.0 && p < 1100.0) {
    pressure = p;
  }
}

// ======================================================================
// IMBL DISTANCE GEOMETRY
// ======================================================================
const double M_PER_DEG_LAT = 111000.0;
const double M_PER_DEG_LON = 109500.0;

double distanceToSegmentMeters(double lat, double lon, double lat1, double lon1, double lat2, double lon2) {
  double x  = lon  * M_PER_DEG_LON;
  double y  = lat  * M_PER_DEG_LAT;
  double x1 = lon1 * M_PER_DEG_LON;
  double y1 = lat1 * M_PER_DEG_LAT;
  double x2 = lon2 * M_PER_DEG_LON;
  double y2 = lat2 * M_PER_DEG_LAT;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double lenSq = dx * dx + dy * dy;

  double t = (lenSq == 0) ? 0 : ((x - x1) * dx + (y - y1) * dy) / lenSq;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;

  double projX = x1 + t * dx;
  double projY = y1 + t * dy;

  return sqrt((x - projX) * (x - projX) + (y - projY) * (y - projY));
}

double getDistanceToIMBLMeters(double lat, double lon) {
  double minDistance = 1e9;
  for (int i = 0; i < IMBL_POINT_COUNT - 1; i++) {
    double dist = distanceToSegmentMeters(
      lat, lon,
      IMBL_POINTS[i].lat, IMBL_POINTS[i].lon,
      IMBL_POINTS[i + 1].lat, IMBL_POINTS[i + 1].lon
    );
    if (dist < minDistance) minDistance = dist;
  }
  return minDistance;
}

// ======================================================================
// GPS
// ======================================================================
void readGPSData() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  gpsLocked = gps.location.isValid() && gps.location.age() < GPS_FIX_TIMEOUT_MS;

  if (gpsLocked) {
    latitude   = gps.location.lat();
    longitude  = gps.location.lng();
    speedKnots = gps.speed.knots();
    courseDeg  = gps.course.deg();
    distanceToBorder = getDistanceToIMBLMeters(latitude, longitude);
  }

  if (gps.satellites.isValid()) {
    satellites = gps.satellites.value();
  }
}

// ======================================================================
// GSM (SIM800L)
// ======================================================================
void setupGSM() {
  delay(1000);
  gsmSerial.println("AT");
  delay(300);
  gsmSerial.println("AT+CMGF=1");
  delay(300);

  unsigned long start = millis();
  bool registered = false;
  while (millis() - start < 10000) {
    gsmSerial.println("AT+CREG?");
    delay(500);
    String response = readGSMResponse();
    if (response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1) {
      registered = true;
      break;
    }
  }

  Serial.println(registered ? "GSM Registered to network" : "GSM Network Registration Failed");
}

String readGSMResponse() {
  String resp = "";
  unsigned long timeout = millis() + 300;
  while (millis() < timeout) {
    while (gsmSerial.available()) {
      resp += (char)gsmSerial.read();
    }
  }
  return resp;
}

void sendSMS(String message) {
  gsmSerial.println("AT+CMGF=1");
  delay(200);
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(phoneNumber);
  gsmSerial.println("\"");
  delay(200);
  gsmSerial.print(message);
  delay(200);
  gsmSerial.write(26);
  delay(500);

  Serial.println("SMS Dispatched:\n" + message);
}

String buildLocationText() {
  if (!gpsLocked) return "Location unavailable (No GPS fix)";
  return "Lat: " + String(latitude, 6) + "\nLon: " + String(longitude, 6) +
         "\nMap: https://maps.google.com/?q=" + String(latitude, 6) + "," + String(longitude, 6);
}

// ======================================================================
// SAFETY CHECKS & ALERTS
// ======================================================================
void checkSOSButton() {
  static unsigned long lastChange = 0;
  bool reading = digitalRead(SOS_BUTTON_PIN);

  if (reading != lastButtonState && millis() - lastChange > 50) {
    lastChange = millis();
    lastButtonState = reading;

    if (reading == LOW) {
      sosActive = true;
      Serial.println("\n🚨 [ALERT] PHYSICAL SOS BUTTON PRESSED!");
      publishTelemetry();
    } else {
      sosActive = false;
      smsSentSOS = false;
      Serial.println("ℹ️ [INFO] SOS Button Released.");
      publishTelemetry();
    }
  }
}

void checkTiltCondition() {
  float tiltMagnitude = max(abs(currentRoll), abs(currentPitch));
  static unsigned long tiltStartTime = 0;

  if (tiltMagnitude > TILT_THRESHOLD_DEG) {
    if (tiltStartTime == 0) {
      tiltStartTime = millis();
    } else if (millis() - tiltStartTime >= TILT_DEBOUNCE_MS) {
      tiltDanger = true;
    }
  } 
  else if (tiltMagnitude < (TILT_THRESHOLD_DEG - TILT_HYSTERESIS_DEG)) {
    tiltStartTime = 0;
    tiltDanger = false;
    smsSentTilt = false;
  }
  else {
    tiltStartTime = 0;
  }
}

void checkBorderCondition() {
  if (!gpsLocked) {
    borderWarning = false;
    borderDanger  = false;
    return;
  }

  if (distanceToBorder <= BORDER_DANGER_M) {
    borderDanger  = true;
    borderWarning = false;
  } else if (distanceToBorder <= BORDER_SAFE_M) {
    borderDanger  = false;
    borderWarning = true;
  } else {
    borderDanger  = false;
    borderWarning = false;
    smsSentBorder = false;
  }
}

void checkWeatherCondition() {
  weatherAlert = (pressure < PRESSURE_THRESHOLD);
}

void handleBuzzer() {
  if (sosActive) {
    tone(BUZZER_PIN, 1200);
  } else if (collisionCritical) {
    tone(BUZZER_PIN, 1500); 
  } else if (tiltDanger) {
    tone(BUZZER_PIN, 1000);
  } else if (borderDanger) {
    tone(BUZZER_PIN, 800);
  } else {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void handleSMSLogic() {
  if (sosActive && !smsSentSOS) {
    sendSMS("EMERGENCY SOS ALERT!\nBoat requires immediate assistance.\n" + buildLocationText());
    smsSentSOS = true;
  }

  if (tiltDanger && !smsSentTilt) {
    sendSMS("CAPSIZE ALERT!\nDangerous boat tilt detected.\n" + buildLocationText());
    smsSentTilt = true;
  }

  if (borderDanger && !smsSentBorder) {
    sendSMS("IMBL BORDER ALERT!\nBoat entered critical danger zone. Turn back immediately!\n" + buildLocationText());
    smsSentBorder = true;
  }
}

// ======================================================================
// LCD REFRESH
// ======================================================================
int getCurrentPriority() {
  if (sosActive)                             return PRI_SOS;
  if (collisionCritical || collisionCaution) return PRI_COLLISION;
  if (tiltDanger)                            return PRI_TILT;
  if (borderDanger || borderWarning)         return PRI_BORDER;
  if (weatherAlert)                          return PRI_WEATHER;
  return PRI_NORMAL;
}

void lcdLine(uint8_t row, const String& text) {
  String s = text;
  if (s.length() > 16) s = s.substring(0, 16);
  while (s.length() < 16) s += ' ';
  lcd.setCursor(0, row);
  lcd.print(s);
}

void updateLCDDisplay() {
  int priority = getCurrentPriority();

  if (priority == PRI_SOS) {
    lcdLine(0, "SOS ACTIVE");
    lcdLine(1, "HELP NEEDED");
    return;
  }

  if (priority == PRI_COLLISION) {
    if (collisionCritical) {
      lcdLine(0, "!COLLISION RISK!");
      lcdLine(1, collisionTarget + " " + String(collisionDistMeters) + "m AWAY");
    } else {
      lcdLine(0, "CAUTION: < 2KM");
      lcdLine(1, collisionTarget + " " + String(collisionDistMeters) + "m");
    }
    return;
  }

  if (priority == PRI_TILT) {
    lcdLine(0, "DANGER TILT");
    lcdLine(1, "CHECK BOAT");
    return;
  }

  if (priority == PRI_BORDER) {
    if (borderDanger) {
      lcdLine(0, "BORDER ALERT");
      lcdLine(1, "TURN BACK");
    } else {
      lcdLine(0, "NEAR BORDER");
      lcdLine(1, String(distanceToBorder, 0) + "m to IMBL");
    }
    return;
  }

  if (priority == PRI_WEATHER) {
    lcdLine(0, "WEATHER ALERT");
    lcdLine(1, "LOW PRESSURE");
    return;
  }

  if (!gpsLocked) {
    lcdLine(0, "GPS WAITING");
    lcdLine(1, "Sats: " + String(satellites));
    return;
  }

  rotateNormalScreens();
}

void rotateNormalScreens() {
  unsigned long now = millis();
  if (now - lastScreenChange >= SCREEN_INTERVAL) {
    lastScreenChange = now;
    currentScreen = (currentScreen + 1) % TOTAL_SCREENS;
  }

  switch (currentScreen) {
    case 0:
      lcdLine(0, "R:" + String(currentRoll, 1) + " P:" + String(currentPitch, 1));
      lcdLine(1, "Boat Stable");
      break;
    case 1:
      lcdLine(0, "GPS LOCKED");
      lcdLine(1, "Sats: " + String(satellites) + " Spd:" + String(speedKnots, 1) + "kn");
      break;
    case 2:
      lcdLine(0, "Lat:" + String(latitude, 4));
      lcdLine(1, "Lon:" + String(longitude, 4));
      break;
    case 3:
      lcdLine(0, "Border Zone");
      lcdLine(1, "Dist:" + String(distanceToBorder, 0) + "m");
      break;
    case 4:
      lcdLine(0, "Temp:" + String(temperature, 1) + "C");
      lcdLine(1, "Pres:" + String(pressure, 1) + "hPa");
      break;
  }
}

// ======================================================================
// LOGGING
// ======================================================================
void printSerialData() {
  Serial.print("Temp: "); Serial.print(temperature);
  Serial.print("C | Pres: "); Serial.print(pressure);
  Serial.print("hPa | Roll: "); Serial.print(currentRoll);
  Serial.print(" Pitch: "); Serial.print(currentPitch);
  Serial.print(" | GPS: "); Serial.print(gpsLocked ? "LOCK" : "NO FIX");
  Serial.print(" Lat: "); Serial.print(latitude, 6);
  Serial.print(" Lon: "); Serial.print(longitude, 6);
  Serial.print(" | IMBL Dist: "); Serial.print(distanceToBorder, 0);
  Serial.print("m | Sats: "); Serial.print(satellites);

  if (collisionCritical) {
    Serial.print(" | 🚨 COLLISION RISK: ");
    Serial.print(collisionTarget);
    Serial.print(" ");
    Serial.print(collisionDistMeters);
    Serial.print("m");
  } else if (collisionCaution) {
    Serial.print(" | ⚠️ <2KM PROXIMITY: ");
    Serial.print(collisionTarget);
    Serial.print(" ");
    Serial.print(collisionDistMeters);
    Serial.print("m");
  }

  Serial.println();
}

// ======================================================================
// NETWORKING & MQTT INBOUND/OUTBOUND
// ======================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("MQTT JSON Parse Failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char* alert = doc["alert"] | "NONE";
  collisionDistMeters = doc["dist"] | 0;
  collisionTarget = (const char*)(doc["target"] | "VESSEL");

  if (String(alert) == "COLLISION_CRITICAL") {
    collisionCritical = true;
    collisionCaution  = false;
    Serial.printf("\n🚨 [ALERT RECEIVED] Critical Collision with %s! Distance: %d meters!\n", collisionTarget.c_str(), collisionDistMeters);
  } 
  else if (String(alert) == "COLLISION_CAUTION") {
    collisionCritical = false;
    collisionCaution  = true;
    Serial.printf("\n⚠️ [WARN RECEIVED] 2KM Safety Warning: %s nearby at %d meters.\n", collisionTarget.c_str(), collisionDistMeters);
  } 
  else {
    collisionCritical = false;
    collisionCaution  = false;
  }
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastWifiAttempt < WIFI_RETRY_INTERVAL_MS) return;
  lastWifiAttempt = now;

  Serial.println("Reconnecting WiFi...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void maintainMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  unsigned long now = millis();
  if (now - lastMqttAttempt < MQTT_RETRY_INTERVAL_MS) return;
  lastMqttAttempt = now;

  String clientId = "sentinel-esp32-" + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFF), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("MQTT Broker Connected.");
    mqttClient.subscribe(MQTT_CMD_TOPIC);
    Serial.print("Subscribed to: ");
    Serial.println(MQTT_CMD_TOPIC);
  } else {
    Serial.print("MQTT Connection Failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void publishTelemetry() {
  if (!mqttClient.connected()) return;

  unsigned long now = millis();

  static bool lastPublishedSOS = false;
  bool sosStateChanged = (sosActive != lastPublishedSOS);

  if (!sosStateChanged && (now - lastTelemetryPublish < MQTT_PUBLISH_INTERVAL_MS)) {
    return;
  }

  lastTelemetryPublish = now;
  lastPublishedSOS = sosActive;

  JsonDocument doc;

  if (gpsLocked) {
    doc["lat"]   = latitude;
    doc["lon"]   = longitude;
    doc["speed"] = speedKnots;
    doc["dist"]  = distanceToBorder;
  } else {
    doc["lat"]   = 0.0;
    doc["lon"]   = 0.0;
    doc["speed"] = 0.0;
    doc["dist"]  = 999999;
  }

  doc["pressure"] = pressure;
  doc["temp"]     = temperature;
  doc["roll"]     = currentRoll;
  doc["pitch"]    = currentPitch;
  doc["sats"]     = satellites;

  doc["sos"]      = sosActive;
  doc["sos_val"]  = sosActive ? 1 : 0;
  doc["status"]   = sosActive ? "EMERGENCY" : "NORMAL";
  doc["alarm"]    = sosActive ? "ACTIVE" : "INACTIVE";

  char payload[512];
  size_t len = serializeJson(doc, payload, sizeof(payload));

  bool ok = mqttClient.publish(MQTT_TOPIC, (const uint8_t*)payload, len, false);

  if (!ok) {
    lastTelemetryPublish = 0; 
  }

  Serial.print(ok ? "Telemetry Published: " : "Telemetry Publish Failed: ");
  Serial.println(payload);
}
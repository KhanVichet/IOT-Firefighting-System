#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// ====== Device ID ======
#define DEVICE_ID "station_1"

// ====== Pin Configuration ======
#define MQ2_PIN       36
#define FLAME_PIN     34
#define BUZZER_PIN    25
#define GPS_RX_PIN    16
#define GPS_TX_PIN    17

// ====== Settings ======
#define GAS_THRESHOLD      1900
#define GPS_BAUD_RATE      9600
#define ALERT_INTERVAL     3000

//====== Firebase Configuration ======
#define WIFI_SSID "MERCURY_EC71"
#define WIFI_PASSWORD "12345678"
// #define WIFI_SSID "Bali Apartment"
// #define WIFI_PASSWORD "bali2019"
// #define WIFI_SSID "NUM PRINCIPAL"
// #define WIFI_PASSWORD "num@2023"
// #define WIFI_SSID "HIDDEN6"
// #define WIFI_PASSWORD "66667777"
#define DATABASE_URL "https://firefighting-241dc-default-rtdb.firebaseio.com/"
#define API_KEY "AIzaSyAxOdDcQ3kgUskH_nq0A40KQkrvUrYTzew"


#define DEFAULT_LAT 12.5659  // Example: Phnom Penh latitude
#define DEFAULT_LNG 104.9908 // Example: Phnom Penh longitude

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool emergencyActive = false;
bool alertLogged = false;
unsigned long lastAlertTime = 0;
unsigned long lastSerialOutput = 0;

void initWiFi();
void initFirebase();
void reportFireToFirebase(bool fireDetected, bool gasDetected);
void logToRealtimeDatabase(bool fireDetected, bool gasDetected);
void handleEmergency();
void printSystemStatus(bool fire, bool gas);
void updateGPS();

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsSerial.print("$PMTK220,200*2C\r\n");

  pinMode(FLAME_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  initWiFi();

  // ⏰ Sync time from NTP
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  delay(2000);  // Wait for NTP sync

  initFirebase();

  Serial.println("\n🔥 Fire/Gas Detector with Firebase Alerts (Realtime DB)");
  Serial.print("This is: "); Serial.println(DEVICE_ID);
}

void loop() {
  updateGPS();

  bool fireDetected = (digitalRead(FLAME_PIN) == LOW);
  bool gasDetected = (analogRead(MQ2_PIN) > GAS_THRESHOLD);

    if (fireDetected || gasDetected) {
    if (!emergencyActive) {
      emergencyActive = true;
      lastAlertTime = millis() - ALERT_INTERVAL;
      Serial.println("\n⚠️ EMERGENCY DETECTED!");

      if (!alertLogged) {
        logToRealtimeDatabase(fireDetected, gasDetected);
        alertLogged = true;
      }
    }

    handleEmergency();

  } else {
    if (emergencyActive) {
      Serial.println("\n✅ Danger cleared");
    }

    emergencyActive = false;
    alertLogged = false;  // Reset only after danger is cleared
    digitalWrite(BUZZER_PIN, LOW);
  }


  if (millis() - lastSerialOutput > 2000) {
    printSystemStatus(fireDetected, gasDetected);
    reportFireToFirebase(fireDetected, gasDetected);
    lastSerialOutput = millis();
  }
}

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
}

void initFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = "user1@gmail.com";
  auth.user.password = "12345678";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("✅ Firebase initialized");
}

void reportFireToFirebase(bool fireDetected, bool gasDetected) {
  if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) {
    Serial.println("WiFi or Firebase not ready - can't report to RTDB");
    return;
  }

  String path = "fire_status/" + String(DEVICE_ID);
  FirebaseJson json;

  // Determine status
  String status;
  if (fireDetected) {
    status = "active";
  } else if (gasDetected) {
    status = "smoke";
  } else {
    status = "clear";
  }

  json.set("status", status);
  json.set("gasLevel", analogRead(MQ2_PIN));

  if (gps.location.isValid()) {
    json.set("latitude", gps.location.lat());
    json.set("longitude", gps.location.lng());
  } else {
    json.set("latitude", DEFAULT_LAT);
    json.set("longitude", DEFAULT_LNG);
  }

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("✅ Realtime fire_status updated");
  } else {
    Serial.println("❌ fire_status update failed: " + fbdo.errorReason());
  }
}


void logToRealtimeDatabase(bool fireDetected, bool gasDetected) {
  if (!Firebase.ready() || WiFi.status() != WL_CONNECTED) return;

  String path = "fire_alerts/" + String(DEVICE_ID);

  FirebaseJson alert;

  // Generate unique message ID
  String messageId = String(DEVICE_ID) + "_" + String(millis());
  alert.set("messageId", messageId);
  alert.set("stationId", DEVICE_ID);

  // Determine status
  String status;
  if (fireDetected) {
    status = "active";
  } else if (gasDetected) {
    status = "smoke";
  } else {
    status = "clear";
  }
  alert.set("status", status);

  alert.set("fireDetected", fireDetected);
  alert.set("gasDetected", gasDetected);
  alert.set("gasLevel", analogRead(MQ2_PIN));

  if (gps.location.isValid()) {
    alert.set("latitude", gps.location.lat());
    alert.set("longitude", gps.location.lng());
  } else {
    alert.set("latitude", 0);
    alert.set("longitude", 0);
  }

  // Add timestamp
  time_t now;
  time(&now);
  char timeStr[32];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
  alert.set("triggeredAt", String(timeStr));

  Serial.println("Logging alert to fire_alerts...");
  if (Firebase.RTDB.pushJSON(&fbdo, path.c_str(), &alert)) {
    Serial.println("✅ Alert logged with messageId: " + messageId);
  } else {
    Serial.println("❌ Failed to log alert: " + fbdo.errorReason());
  }
}


void handleEmergency() {
  digitalWrite(BUZZER_PIN, HIGH);

  if (millis() - lastAlertTime > ALERT_INTERVAL) {
    if (gps.location.isValid() && gps.location.age() < 2000) {
      Serial.println("\n🚨 EMERGENCY LOCATION 🚨");
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
    } else {
      Serial.println("⌛️ Acquiring GPS fix...");
    }
    lastAlertTime = millis();
  }
}

void updateGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

void printSystemStatus(bool fire, bool gas) {
  Serial.print("\nSYSTEM STATUS | ");
  Serial.print("Fire: ");
  Serial.print(fire ? "DETECTED" : "Clear");
  Serial.print(" | Gas: ");
  Serial.print(gas ? "DETECTED (" : "Clear (");
  Serial.print(analogRead(MQ2_PIN));
  Serial.print("/") ;
  Serial.print(GAS_THRESHOLD);
  Serial.print(") | WiFi: ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.print(" | Firebase: ");
  Serial.print(Firebase.ready() ? "Ready" : "Not ready");
  Serial.print(" | GPS: ");
  Serial.print(gps.satellites.isValid() ? String(gps.satellites.value()) + " sats" : "No fix");
}

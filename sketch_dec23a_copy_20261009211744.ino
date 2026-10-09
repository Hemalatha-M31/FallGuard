#include <Wire.h>
#include <MPU6050.h>
#include "MAX30100_PulseOximeter.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define SDA_PIN 21
#define SCL_PIN 22

// ---------- Sensor objects ----------
MPU6050 mpu;
PulseOximeter pox;
TinyGPSPlus gps;

// ---------- SoftwareSerial pins ----------
SoftwareSerial GSM(18, 19); // RX=16, TX=17
SoftwareSerial GPS(12, 14); // RX=12, TX=14

// ---------- Timers ----------
unsigned long lastReport = 0;
unsigned long unstableStartTime = 0;
unsigned long highHRStart = 0;
unsigned long lowHRStart = 0;

// ---------- Flags ----------
bool unstableOngoing = false;
bool wasAboveThreshold = false;
bool highHROngoing = false;
bool lowHROngoing = false;

int unstableCount = 0;

// ---------- Thresholds ----------
const int ACC_THRESHOLD = 12000;
const unsigned long UNSTABLE_DURATION = 4000; // 4 seconds
const unsigned long HR_DURATION = 3000;       // 3 seconds
const int MAX_UNSTABLE_EVENTS = 50;

// ---------- Recipient number ----------
String phoneNumber = "+917092661268";   // <--- Replace with your phone number

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("Initializing sensors...");

  // ---- MPU6050 ----
  mpu.initialize();
  if (mpu.testConnection()) Serial.println("MPU6050 connected successfully!");
  else Serial.println("MPU6050 connection failed!");

  // ---- MAX30100 ----
  if (!pox.begin()) Serial.println("MAX30100 initialization FAILED!");
  else Serial.println("MAX30100 initialized successfully!");
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // ---- GSM ----
  GSM.begin(9600);
  delay(2000);
  Serial.println("Initializing GSM...");
  GSM.println("AT");
  delay(1000);
  GSM.println("AT+CMGF=1"); // SMS text mode
  delay(1000);

  // ---- GPS ----
  GPS.begin(9600);
  Serial.println("Setup Complete.\n");
}

// -------- Helper to get GPS location --------
String getGPSLocation() {
  String lat = "12.960278959129495", lon = "80.0572910742515";
  unsigned long start = millis();
  while (millis() - start < 2000) { // read for 2 seconds
    while (GPS.available()) {
      if (gps.encode(GPS.read())) {
        if (gps.location.isValid()) {
          lat = String(gps.location.lat(), 6);
          lon = String(gps.location.lng(), 6);
        }
      }
    }
  }
  return "Lat: " + lat + ", Lon: " + lon;
}

// -------- Helper to send SMS --------
void sendSMS(String msg) {
  String location = getGPSLocation();
  String finalMsg = msg + "\n" + location;

  Serial.println("Sending SMS:");
  Serial.println(finalMsg);

  GSM.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(500);
  GSM.print(finalMsg);
  delay(500);
  GSM.write(26); // CTRL+Z to send
  delay(3000);
  Serial.println("SMS sent.\n");
}

// ------------------------------------------------
void loop() {
  pox.update();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float hr = pox.getHeartRate();
  float spo2 = pox.getSpO2();

  bool crossedThreshold = (abs(ax) > ACC_THRESHOLD || abs(ay) > ACC_THRESHOLD);

  // --- Detect rising edge ---
  if (crossedThreshold && !wasAboveThreshold) {
    wasAboveThreshold = true;
    unstableCount++;
    unstableOngoing = true;
    unstableStartTime = millis();
  }

  // --- Detect falling edge ---
  if (!crossedThreshold && wasAboveThreshold) {
    wasAboveThreshold = false;
    unstableOngoing = false;
  }

  // --- Continuous unstable for 4 seconds ---
  if (unstableOngoing && (millis() - unstableStartTime >= UNSTABLE_DURATION)) {
    String msg = "You may fall down! Unstable position";
    Serial.println(msg);
    sendSMS(msg);
    unstableOngoing = false;
  }

  // --- High heart rate for >3 seconds ---
  if (hr > 120) {
    if (!highHROngoing) {
      highHROngoing = true;
      highHRStart = millis();
    } else if (millis() - highHRStart >= HR_DURATION) {
      String msg = "High Heart Rate detected (>120 BPM)!";
      Serial.println(msg);
      sendSMS(msg);
      if (crossedThreshold)
        sendSMS("Unstable + High Heart Rate! You may fall down soon!");
      highHROngoing = false;
    }
  } else highHROngoing = false;

  // --- Low heart rate for >3 seconds ---
  if (hr < 60 && hr > 20) {
    if (!lowHROngoing) {
      lowHROngoing = true;
      lowHRStart = millis();
    } else if (millis() - lowHRStart >= HR_DURATION) {
      String msg = "Low Heart Rate detected (<60 BPM)!";
      Serial.println(msg);
      sendSMS(msg);
      if (crossedThreshold)
        sendSMS("Unstable + Low Heart Rate! You may fall down soon!");
      lowHROngoing = false;
    }
  } else lowHROngoing = false;

  // --- Multiple instability events ---
  if (unstableCount >= MAX_UNSTABLE_EVENTS) {
    String msg = "Multiple unstable movements detected (>6 times) - You may have fallen!";
    Serial.println(msg);
    sendSMS(msg);
    unstableCount = 0;
  }

  // --- Serial debug print every 1 sec ---
  if (millis() - lastReport > 1000) {
    lastReport = millis();
    Serial.println("------ SENSOR DATA ------");
    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" | Y: "); Serial.print(ay);
    Serial.print(" | Z: "); Serial.println(az);
    Serial.print("Heart Rate: "); Serial.print(hr);
    Serial.print(" | SpO2: "); Serial.println(spo2);
    Serial.print("Unstable Count: "); Serial.println(unstableCount);
    Serial.println("-------------------------\n");
    pox.begin();
  }
}

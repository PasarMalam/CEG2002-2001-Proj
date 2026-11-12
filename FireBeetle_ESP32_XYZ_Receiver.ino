/* --------------------------------------------------
FireBeetle ESP32 UART Sensor Receiver & Firebase Uploader

Author:
Version: 1.2.0

Responsibilities:
- Receive MSP430 measurements using UART2 at 9600 baud
- Display sensor measurements on USB Serial Monitor at 115200 baud
- Display X, Y and Z vibration RMS when provided by MSP430
- Continue accepting the original five-field DATA message
- Display calibration, baseline and error messages
- Upload latest and historical telemetry to Google Firebase RTDB

UART message formats:
Old: DATA,current,temp,totalRms,status,impact
New: DATA,current,temp,xRms,yRms,zRms,totalRms,status,impact
Values: current in 0.1 mA, temperature in 0.1 C, RMS in 0.1 mg
-------------------------------------------------- */
#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>
#include <stdio.h>
#include <string.h>

// ---------------- Wi-Fi & Firebase Configuration ----------------
// Note: ESP32 only supports 2.4 GHz Wi-Fi networks
#define WIFI_SSID     "HONOR X9c"
#define WIFI_PASSWORD "bzqyyyjbc3pijcm"
// Hostname only (no "https://"): Firebase.begin() skips URL parsing in Test Mode, so "https://" gets sent in the Host header and causes HTTP 400 "Bad request."
#define DATABASE_URL  "ceg2001-2-project-group-p2-5-default-rtdb.asia-southeast1.firebasedatabase.app"

const unsigned long FIREBASE_UPLOAD_INTERVAL_MS = 2000; // Upload at most once every 2 seconds

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

HardwareSerial MSP430Serial(2);

const int RX_PIN = 16;           // FireBeetle D11, connected to MSP430 P3.3 TX
const int TX_PIN = 17;           // FireBeetle D10, connected to MSP430 P3.4 RX
char message[120];               // Store one complete UART message
unsigned int messageLength = 0;

struct TelemetrySnapshot {
    float currentMa;
    float temperatureC;
    float xRmsMg;
    float yRmsMg;
    float zRmsMg;
    float totalRmsMg;
    unsigned int status;
    unsigned int impact;
    bool hasXYZ;
    bool pendingUpload;
};

TelemetrySnapshot latestTelemetry = {0};
unsigned long lastFirebaseSendMs = 0;

/* --------------------------------------------------
Function: GetStatusText

Responsibilities:
- Convert MSP430 numeric vibration status into readable text
-------------------------------------------------- */
const char* GetStatusText(unsigned int status)
{
    switch (status) {
        case 0: return "WITHIN BASELINE";
        case 1: return "ABOVE BASELINE (checking)";
        case 2: return "ABOVE NORMAL BASELINE";
        case 3: return "BELOW BASELINE (checking)";
        case 4: return "BELOW NORMAL BASELINE";
        default: return "UNKNOWN";
    }
}

/* --------------------------------------------------
Function: PrintStatus

Author:
Version: 1.1.0

Responsibilities:
- Convert MSP430 numeric vibration status into readable text and print to Serial

Parameters:
status - Vibration warning status from MSP430
-------------------------------------------------- */
void PrintStatus(unsigned int status)
{
    Serial.print("RMS status: ");
    Serial.println(GetStatusText(status));
}

/* --------------------------------------------------
Function: UploadToFirebase

Responsibilities:
- Push the latest telemetry snapshot to /current and /history in Firebase RTDB
-------------------------------------------------- */
void UploadToFirebase(const TelemetrySnapshot &data)
{
    if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) {
        return;
    }

    // 1. Update "/current" node (drives the top cards on the dashboard)
    FirebaseJson currentJson;
    currentJson.set("current", data.currentMa);
    currentJson.set("temperature", data.temperatureC);
    currentJson.set("totalRms", data.totalRmsMg);
    currentJson.set("status", (int)data.status);
    currentJson.set("statusText", GetStatusText(data.status));
    currentJson.set("impact", (int)data.impact);
    currentJson.set("impactText", data.impact ? "POSSIBLE IMPACT" : "NONE");
    currentJson.set("hasXYZ", data.hasXYZ);
    if (data.hasXYZ) {
        currentJson.set("xRms", data.xRmsMg);
        currentJson.set("yRms", data.yRmsMg);
        currentJson.set("zRms", data.zRmsMg);
    }
    currentJson.set("t/.sv", "timestamp");

    if (Firebase.RTDB.setJSON(&fbdo, "/current", &currentJson)) {
        Serial.println("Firebase /current updated");
    } else {
        Serial.printf("Firebase /current failed (HTTP %d): %s\n", fbdo.httpCode(), fbdo.errorReason().c_str());
    }

    // 2. Append to "/history" node (drives the 1m / 1h / 1d trend charts)
    FirebaseJson historyJson;
    historyJson.set("t/.sv", "timestamp");
    historyJson.set("current", data.currentMa);
    historyJson.set("temperature", data.temperatureC);
    historyJson.set("totalRms", data.totalRmsMg);
    historyJson.set("status", (int)data.status);
    historyJson.set("impact", (int)data.impact);
    if (data.hasXYZ) {
        historyJson.set("xRms", data.xRmsMg);
        historyJson.set("yRms", data.yRmsMg);
        historyJson.set("zRms", data.zRmsMg);
    }

    if (Firebase.RTDB.pushJSON(&fbdo, "/history", &historyJson)) {
        Serial.println("Firebase /history appended");
    } else {
        Serial.printf("Firebase /history failed (HTTP %d): %s\n", fbdo.httpCode(), fbdo.errorReason().c_str());
    }
}

/* --------------------------------------------------
Function: ProcessMessage

Author:
Version: 1.2.0

Responsibilities:
- Identify a received UART message
- Decode both old and new DATA formats
- Print sensor values and monitoring status
- Stage decoded values for throttled Firebase upload

Parameters:
line - Null-terminated UART message received from MSP430
-------------------------------------------------- */
void ProcessMessage(const char *line)
{
    long current = 0, temperature = 0;
    unsigned long x = 0, y = 0, z = 0, rms = 0;
    unsigned int status = 0, impact = 0;
    unsigned int commas = 0;

    // Count commas to distinguish the old and new message formats
    for (const char *p = line; *p != '\0'; p++) {
        if (*p == ',') commas++;
    }

    if (strncmp(line, "DATA,", 5) == 0) {
        bool hasXYZ = false;
        bool valid = false;

        if (commas == 8) {
            // New format: 8 numeric fields, including X, Y and Z RMS
            valid = (sscanf(line, "DATA,%ld,%ld,%lu,%lu,%lu,%lu,%u,%u",
                            &current, &temperature, &x, &y, &z, &rms,
                            &status, &impact) == 8);
            hasXYZ = valid;
        }
        else if (commas == 5) {
            // Old format: 5 numeric fields, total RMS only
            valid = (sscanf(line, "DATA,%ld,%ld,%lu,%u,%u",
                            &current, &temperature, &rms,
                            &status, &impact) == 5);
        }

        if (!valid) {
            Serial.print("Invalid DATA message: ");
            Serial.println(line);
            return;
        }

        float currentMa = current / 10.0f;
        float temperatureC = temperature / 10.0f;
        float xRmsMg = x / 10.0f;
        float yRmsMg = y / 10.0f;
        float zRmsMg = z / 10.0f;
        float totalRmsMg = rms / 10.0f;

        Serial.println("-------------------------------");
        Serial.print("Current: ");
        Serial.print(currentMa, 1);
        Serial.println(" mA");
        Serial.print("Temperature: ");
        Serial.print(temperatureC, 1);
        Serial.println(" C");

        if (hasXYZ) {
            Serial.print("Vibration X RMS: ");
            Serial.print(xRmsMg, 1);
            Serial.println(" mg");
            Serial.print("Vibration Y RMS: ");
            Serial.print(yRmsMg, 1);
            Serial.println(" mg");
            Serial.print("Vibration Z RMS: ");
            Serial.print(zRmsMg, 1);
            Serial.println(" mg");
        } else {
            Serial.println("X/Y/Z RMS: not sent by MSP430 yet");
        }

        Serial.print("Vibration Total RMS: ");
        Serial.print(totalRmsMg, 1);
        Serial.println(" mg");
        PrintStatus(status);
        Serial.print("Impact: ");
        Serial.println(impact ? "POSSIBLE IMPACT" : "NONE");
        Serial.println("-------------------------------");
        MSP430Serial.println("ACK,DATA");

        // Stage snapshot for non-blocking Firebase upload
        latestTelemetry.currentMa = currentMa;
        latestTelemetry.temperatureC = temperatureC;
        latestTelemetry.xRmsMg = xRmsMg;
        latestTelemetry.yRmsMg = yRmsMg;
        latestTelemetry.zRmsMg = zRmsMg;
        latestTelemetry.totalRmsMg = totalRmsMg;
        latestTelemetry.status = status;
        latestTelemetry.impact = impact;
        latestTelemetry.hasXYZ = hasXYZ;
        latestTelemetry.pendingUpload = true;
    }
    else if (strncmp(line, "CAL,", 4) == 0) {
        int window = 0;
        long calRms = 0;

        if (sscanf(line, "CAL,%d,%ld", &window, &calRms) == 2) {
            Serial.print("Calibration: ");
            Serial.print(window);
            Serial.print("/20");
            Serial.print(" | Vibration RMS: ");
            Serial.print(calRms / 10.0, 1);
            Serial.println(" mg");
        }
        MSP430Serial.println("ACK,CAL");
    }
    else if (strncmp(line, "BASE,", 5) == 0) {
        Serial.print("Baseline: ");
        Serial.println(line);
        MSP430Serial.println("ACK,BASE");
    }
    else if (strncmp(line, "HELLO,", 6) == 0) {
        Serial.println("MSP430 connected");
        MSP430Serial.println("ACK,HELLO");
    }
    else if (strncmp(line, "ERROR,", 6) == 0) {
        Serial.print("MSP430 error: ");
        Serial.println(line + 6);
    }
    else {
        Serial.print("Unknown message: ");
        Serial.println(line);
    }
}

void setup()
{
    Serial.begin(115200);                                   // USB Serial Monitor
    MSP430Serial.setRxBufferSize(1024);                     // Prevent UART drops during HTTPS uploads
    MSP430Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);   // UART2 to MSP430

    Serial.println("ESP32 sensor receiver starting...");

    // Connect to Wi-Fi (with timeout so UART receiver still works if Wi-Fi is unavailable)
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < 10000)) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Wi-Fi not connected yet; will retry in background.");
    }

    // Configure Firebase RTDB in Test Mode (matches public .read/.write rules)
    config.database_url = DATABASE_URL;
    config.signer.test_mode = true;
    Firebase.reconnectNetwork(true);
    fbdo.setBSSLBufferSize(4096, 1024);
    Firebase.begin(&config, &auth);

    Serial.println("Waiting for MSP430 measurements...");
}

void loop()
{
    while (MSP430Serial.available() > 0) {
        char c = (char)MSP430Serial.read(); // Read one UART byte

        if (c == '\r') continue;           // Ignore carriage return

        if (c == '\n') {
            message[messageLength] = '\0'; // End the received message
            if (messageLength > 0) ProcessMessage(message);
            messageLength = 0;             // Prepare for next message
        }
        else if (messageLength < sizeof(message) - 1) {
            message[messageLength++] = c;  // Store the next character
        }
        else {
            messageLength = 0;             // Discard overly long messages
        }
    }

    // Upload the latest telemetry at most once per interval so UART remains responsive
    if (latestTelemetry.pendingUpload &&
        (millis() - lastFirebaseSendMs >= FIREBASE_UPLOAD_INTERVAL_MS)) {
        latestTelemetry.pendingUpload = false;
        lastFirebaseSendMs = millis();
        UploadToFirebase(latestTelemetry);
    }
}

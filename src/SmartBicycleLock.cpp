#include "SmartBicycleLock.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include<string>

using namespace std;

SmartBicycleLock::SmartBicycleLock() {}

void SmartBicycleLock::connectToWiFi() {
    Serial.println("Attempting to connect to Wi-Fi...");

    WiFi.begin(internetSSID, internetPassword);
    int retries = 0;

    // Try to connect to internet WiFi
    while (WiFi.status() != WL_CONNECTED && retries < 10) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Internet Wi-Fi!");
    } else {
        Serial.println("\nFailed to connect to Internet Wi-Fi. Switching to Local Wi-Fi.");

        WiFi.begin(localSSID, localPassword);
        retries = 0;

        // Try to connect to local WiFi
        while (WiFi.status() != WL_CONNECTED && retries < 10) {
            delay(500);
            Serial.print(".");
            retries++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to Local Wi-Fi!");
        } else {
            Serial.println("\nFailed to connect to both Wi-Fi networks.");
        }
    }

    if (WiFi.status() == WL_CONNECTED && ThingSpeak.begin(client)) {
        Serial.println("Connected to ThingSpeak.");
    }
}

void SmartBicycleLock::setup() {
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Initialize GPS

    connectToWiFi();

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    lockServo.attach(15);
    lockServo.write(lockAngle);

    Serial.println("Setup complete");
}

void SmartBicycleLock::startSession() {
    sessionActive = true;
    sessionStart = millis();
    maxSpeed = 0;
    distance = 0;
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();

    Serial.println("Session started.");
    Serial.print("Start Coordinates: Lat: ");
    Serial.print(lastLat, 6);
    Serial.print(", Lon: ");
    Serial.println(lastLon, 6);
}

void SmartBicycleLock::endSession() {
    sessionActive = false;
    sessionEnd = millis();
    Serial.println("Session ended.");
    sendToThingSpeak();
}

void SmartBicycleLock::sendToThingSpeak() {
    if (WiFi.status() == WL_CONNECTED) {
        int x = sessionStart / 1000;
        int y = millis() / 1000;
        float a = (gps.location.lat(), 6);
        float b = (gps.location.lng(), 6);

    //     float a = gps.location.lat();
	//     float b = gps.location.lng();

	// // Round to 6 decimal places
	//     a = roundf(a * 1000000) / 1000000;
	//     b = roundf(b * 1000000) / 1000000;

        ThingSpeak.setField(1, x);  // Field 1: Session Start Time (in seconds)
        ThingSpeak.setField(2, y);       // Field 2: Current Time (in seconds)
        ThingSpeak.setField(3, maxSpeed);               // Field 3: Max Speed
        ThingSpeak.setField(4, distance);               // Field 4: Distance
        ThingSpeak.setField(5, a);  // Field 5: Latitude
        ThingSpeak.setField(6, b);  // Field 6: Longitude

        int response = ThingSpeak.writeFields(channelID, writeAPIKey);
        if (response == 200) {
            Serial.println("Data sent to ThingSpeak successfully.");
        } else {
            Serial.print("Failed to send data to ThingSpeak. Response code: ");
            Serial.println(response);
        }
    } else {
        Serial.println("Wi-Fi not connected. Cannot send data to ThingSpeak.");
   }
}

void SmartBicycleLock::checkGPS() {
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }

    if (gps.location.isUpdated() && sessionActive) {
        float currentLat = gps.location.lat();
        float currentLon = gps.location.lng();

        if (gps.speed.isUpdated()) {
            float speed = gps.speed.kmph();
            maxSpeed = max(maxSpeed, speed);
            distance += gps.distanceBetween(lastLat, lastLon, currentLat, currentLon);
            lastLat = currentLat;
            lastLon = currentLon;

            Serial.print("Current Speed: ");
            Serial.print(speed);
            Serial.println(" km/h");
            Serial.print("Current Coordinates: Lat: ");
            Serial.print(currentLat, 6);
            Serial.print(", Lon: ");
            Serial.println(currentLon, 6);
            Serial.print("Current Distance: ");
            Serial.println(distance);
        }
    }
}

void SmartBicycleLock::displayData() {
    Serial.print("Session Start Time: ");
    Serial.print(sessionStart / 1000);
    Serial.print(" | Session End Time: ");
    Serial.print(sessionEnd / 1000);
    Serial.print(" | Max Speed: ");
    Serial.print(maxSpeed);
    Serial.print(" km/h | Distance: ");
    Serial.print(distance);
    Serial.print(" m | Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: ");
    Serial.println(gps.location.lng(), 6);

    sendToThingSpeak();
}

void SmartBicycleLock::checkWiFiSignal() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!sessionActive) {
            lockServo.write(unlockAngle);
            startSession();
        }
    } else if (sessionActive) {
        lockServo.write(lockAngle);
        endSession();
    }
}

void SmartBicycleLock::checkAccelerometer() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float tilt = abs(a.acceleration.x) + abs(a.acceleration.z);
    if (tilt < 1.0 && sessionActive) {
        Serial.println("SOS: Cycle fell!");
        fallDetected = true;
    } else {
        fallDetected = false;
    }
}

void SmartBicycleLock::loop() {
    checkGPS();
    checkWiFiSignal();
    checkAccelerometer();

    // Every 10 seconds, update the data and send it to ThingSpeak
    if (millis() - lastUpdateTime >= 10000) {
        displayData();
        lastUpdateTime = millis();
    }
}

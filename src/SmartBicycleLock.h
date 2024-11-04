#ifndef SMART_BICYCLE_LOCK_H
#define SMART_BICYCLE_LOCK_H

#include <WiFi.h>
#include <ThingSpeak.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <esp32Servo.h>
#include <Arduino_JSON.h>  // Include necessary libraries

class SmartBicycleLock {
public:
    SmartBicycleLock();
    void setup();
    void loop();

private:
    void connectToWiFi();
    void startSession();
    void endSession();
    void sendToThingSpeak();
    void checkGPS();
    void checkWiFiSignal();
    void checkAccelerometer();
    void displayData();

    WiFiClient client;
    TinyGPSPlus gps;
    Adafruit_MPU6050 mpu;
    Servo lockServo;

    const char* internetSSID;
    const char* internetPassword;
    const char* localSSID;
    const char* localPassword;
    const char* writeAPIKey;
    const unsigned long channelID = 2725494;

    const int lockAngle = 90;
    const int unlockAngle = 0;
    float lastLat = 0, lastLon = 0, maxSpeed = 0, distance = 0;
    bool sessionActive = false, fallDetected = false;
    unsigned long sessionStart = 0, sessionEnd = 0, lastUpdateTime = 0;
};

#endif

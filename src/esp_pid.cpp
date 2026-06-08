#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

const char* ssid = "KAUA_LQ";
const char* password = "12345678";

WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[255];

#define BLINK 8

Servo servoX;
Servo servoY;
const int servoPinX = 2;
const int servoPinY = 3;
float posX = 0;
float posY = 60;

// --- Parâmetros PID ---
float KpX = 0.02;
float KiX = 0.000;
float KdX = 0.00;
float errorX = 0;
float previousErrorX = 0;
float integralX = 0;

float KpY = 0.02;
float KiY = 0.000;
float KdY = 0.00;
float errorY = 0;
float previousErrorY = 0;
float integralY = 0;

unsigned long previousTime = 0;

void setup() {
    Serial.begin(115200);
    pinMode(BLINK, OUTPUT);
    bool ledState = false;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        ledState = !ledState;
        digitalWrite(BLINK, ledState);
    }

    digitalWrite(BLINK, LOW);

    udp.begin(udpPort);

    servoX.attach(servoPinX);
    servoY.attach(servoPinY);
    servoX.write(posX);
    servoY.write(posY);

    delay(1000);

    previousTime = millis();
}

void loop() {
    int packetSize = udp.parsePacket();

    if (packetSize) {
        int len = udp.read(incomingPacket, 255);

        if (len > 0) {
            incomingPacket[len] = 0;
        }

        sscanf(incomingPacket, "%f,%f", &errorX, &errorY);
        if (abs(errorX) < 15) errorX = 0;
        if (abs(errorY) < 15) errorY = 0;

        // Cáculo da variação de tempo (dt)
        unsigned long currentTime = millis();
        float dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;
        if (dt <= 0) return;

        // --- Controle PID ---
        integralX += errorX * dt;
        float derivativeX = (errorX - previousErrorX) / dt;
        float outputX =
            KpX * errorX +
            KiX * integralX +
            KdX * derivativeX;
        previousErrorX = errorX;

        integralY += errorY * dt;
        float derivativeY = (errorY - previousErrorY) / dt;
        float outputY =
            KpY * errorY +
            KiY * integralY +
            KdY * derivativeY;
        previousErrorY = errorY;

        // --- MOVE SERVOS ---
        posX -= outputX;
        posY += outputY;
        posX = constrain(posX, 0, 180);
        posY = constrain(posY, 0, 180);
        servoX.write(posX);
        servoY.write(posY);
    }
}
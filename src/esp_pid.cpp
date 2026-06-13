#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "control_index.h"
#include "env.h" // Lembre-se de criar seu próprio arquivo env.h se for usar o bot do Telegram, ou de substituir as variáveis botToken e chatId pelos seus valores reais.

const char* ssid = "IFCE-PECEM-ADM";
const char* password = "IFCE&pecem";

AsyncWebServer server(80);
WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[255];

#define BLINK 8

Servo servoX;
Servo servoY;
const int servoPinX = 3;
const int servoPinY = 2;
float posX = 60;
float posY = 60;

// --- Parâmetros PID ---
volatile float KpX = 0.02;
volatile float KiX = 0.000;
volatile float KdX = 0.00;
float errorX = 0;
float previousErrorX = 0;
float integralX = 0;

volatile float KpY = 0.02;
volatile float KiY = 0.000;
volatile float KdY = 0.00;
float errorY = 0;
float previousErrorY = 0;
float integralY = 0;

unsigned long previousTime = 0;

void enviarTelegram(const char *ip) {
    HTTPClient http;

    String url =
      "https://api.telegram.org/bot" +
      botToken +
      "/sendMessage?chat_id=" +
      chatId +
      "&text=" +
      ip;

    http.begin(url);
    int httpCode = http.GET();

    http.end();
}

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

    IPAddress ip = WiFi.localIP();
    char ipStr[16];
    snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    enviarTelegram(ipStr);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", htmlPage());
    });

    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if(request->hasParam("kpx"))
            KpX = request->getParam("kpx")->value().toFloat();

        if(request->hasParam("kix"))
            KiX = request->getParam("kix")->value().toFloat();

        if(request->hasParam("kdx"))
            KdX = request->getParam("kdx")->value().toFloat();

        if(request->hasParam("kpy"))
            KpY = request->getParam("kpy")->value().toFloat();

        if(request->hasParam("kiy"))
            KiY = request->getParam("kiy")->value().toFloat();

        if(request->hasParam("kdy"))
            KdY = request->getParam("kdy")->value().toFloat();

        request->redirect("/");
    });

    digitalWrite(BLINK, LOW);

    server.begin();
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
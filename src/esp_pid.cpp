#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include "env.h" // NOTE: crie seu próprio arquivo env.h se for usar o bot do Telegram (botToken e chatId), ou apague a chamada a enviarTelegram() em setup() se não quiser usar.

const char* ssid = "CLEUDO";
const char* password = "91898487";

WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[255];

#define BLINK 8

Servo servoX;
Servo servoY;
const int servoPinX = 2;
const int servoPinY = 3;
float posX = 60;
float posY = 60;
const int SERVO_X_MIN = 0;
const int SERVO_X_MAX = 180;
const int SERVO_Y_MIN = 0;
const int SERVO_Y_MAX = 180;

// --- Parâmetros PID ---
// Agora configuráveis em tempo real via pacote UDP "CFG,..." (ver parseConfigPacket)
volatile float KpX = 0.001;
volatile float KiX = 0.000;
volatile float KdX = 0.00;
float errorX = 0;
float previousErrorX = 0;
float integralX = 0;

volatile float KpY = 0.001;
volatile float KiY = 0.000;
volatile float KdY = 0.00;
float errorY = 0;
float previousErrorY = 0;
float integralY = 0;

unsigned long previousTime = 0;

// --- Watchdog de "alvo perdido" ---
// vale a pena saber quando paramos de receber pacotes de rastreamento, para zerar o integral do PID
// (anti-windup) e não causar um "chute" nos servos quando um rosto novo for detectado depois de um tempo sem nenhum na tela.
unsigned long lastPacketTime = 0;
const unsigned long TRACKING_TIMEOUT_MS = 1000;
bool alvoAtivo = false;

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

// ----------------------------------------------------------------------
// Parser do pacote de configuração (mesmo protocolo do RescueTracker).
// Formato esperado: "CFG,kpx=0.02,kix=0.0,kdx=0.0,kpy=0.02,kiy=0.0,kdy=0.0"
// Não é obrigatório enviar todas as chaves — apenas as presentes no
// pacote são atualizadas, as demais mantêm o valor atual.
// ----------------------------------------------------------------------
void parseConfigPacket(char *packet) {
    // Pula o prefixo "CFG," antes de começar a tokenizar os pares.
    char *cursor = packet + 4;
    char *par = strtok(cursor, ",");

    while (par != NULL) {
        char *igual = strchr(par, '=');

        if (igual != NULL) {
            *igual = '\0';            // separa "chave" de "valor" no mesmo buffer
            const char *chave = par;
            float valor = atof(igual + 1);

            if (strcmp(chave, "kpx") == 0) KpX = valor;
            else if (strcmp(chave, "kix") == 0) KiX = valor;
            else if (strcmp(chave, "kdx") == 0) KdX = valor;
            else if (strcmp(chave, "kpy") == 0) KpY = valor;
            else if (strcmp(chave, "kiy") == 0) KiY = valor;
            else if (strcmp(chave, "kdy") == 0) KdY = valor;
        }

        par = strtok(NULL, ",");
    }

    Serial.println("Configuração PID atualizada via UDP.");
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
        int len = udp.read(incomingPacket, 254);
        if (len > 0) {
            incomingPacket[len] = 0;
        } else {
            return;
        }

        // --- Pacote de configuração de PID (não atualiza lastPacketTime,
        //     pois isso não é um pacote de tracking) ---
        if (strncmp(incomingPacket, "CFG,", 4) == 0) {
            parseConfigPacket(incomingPacket);
            return;
        }

        // --- Pacote de tracking: "errorX,errorY" (um rosto por vez) ---
        lastPacketTime = millis();
        alvoAtivo = true;

        sscanf(incomingPacket, "%f,%f", &errorX, &errorY);
        if (abs(errorX) < 15) errorX = 0;
        if (abs(errorY) < 15) errorY = 0;

        // Cálculo da variação de tempo (dt)
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
        posX = constrain(posX, SERVO_X_MIN, SERVO_X_MAX);
        posY = constrain(posY, SERVO_Y_MIN, SERVO_Y_MAX);
        servoX.write(posX);
        servoY.write(posY);
    }

    // --- Anti-windup: se ficamos sem receber pacote de tracking por um
    //     tempo (rosto saiu de cena), zera os acumuladores do PID para não
    //     causar um "chute" nos servos quando o próximo rosto for detectado. ---
    if (alvoAtivo && (millis() - lastPacketTime > TRACKING_TIMEOUT_MS)) {
        alvoAtivo = false;
        integralX = 0;
        integralY = 0;
        previousErrorX = 0;
        previousErrorY = 0;
    }
}
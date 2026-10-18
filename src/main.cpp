#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include <ArduinoJson.h>

// ---------- Пины SX1262 на Heltec WiFi LoRa 32 V3 ----------
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_NRST  12
#define LORA_BUSY  13
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// ---------- Параметры по умолчанию ----------
float  freq     = 868.0;   // МГц
float  bw       = 125.0;   // кГц
uint8_t sf      = 7;
uint8_t cr      = 5;       // 4/5
uint8_t syncWord = 0x12;
uint16_t preamble = 8;
bool    crcOn   = true;
bool    iqInverted = false;

// ---------- Состояние ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
volatile bool packetReceived = false;
bool recording = false;
File dumpFile;

const int MAX_PACKETS = 50;   // сколько последних пакетов храним в памяти для веб-таблицы
struct Packet {
    uint32_t timeMs;
    float rssi;
    float snr;
    uint8_t len;
    uint8_t data[255];
};
Packet packets[MAX_PACKETS];
int packetIndex = 0;

// ---------- Прерывание DIO1 ----------
void IRAM_ATTR onDio1() {
    packetReceived = true;
}

// ---------- Настройка радио ----------
void applyRadioConfig() {
    radio.standby();
    radio.setFrequency(freq);
    radio.setBandwidth(bw);
    radio.setSpreadingFactor(sf);
    radio.setCodingRate(cr);
    radio.setSyncWord(syncWord);
    radio.setPreambleLength(preamble);
    radio.setCRC(crcOn);
    //radio.setIQ(iqInverted);
    radio.invertIQ(iqInverted);
    radio.startReceive();
    Serial.printf("Radio: %.1f MHz, BW=%.1f, SF=%d, CR=4/%d, Sync=0x%02X\n",
                  freq, bw, sf, cr, syncWord);
}

// ---------- Инициализация ----------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- LoRa Sniffer Heltec V3 ---");

    // LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    }

    // Радио
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio init failed: %d\n", state);
        while (true) delay(1000);
    }
    radio.setDio1Action(onDio1);
    applyRadioConfig();

    // SoftAP + DHCP
    IPAddress local_ip(192, 168, 0, 1);
    IPAddress gateway(192, 168, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP("LoRaSniffer", "sniffer123");
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    // Веб-сервер
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/dump.loradump")) {
            request->send(LittleFS, "/dump.loradump", "application/octet-stream", true);
        } else {
            request->send(404, "text/plain", "No dump");
        }
    });

    server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("freq", true)) freq = request->getParam("freq", true)->value().toFloat();
        if (request->hasParam("bw", true))   bw   = request->getParam("bw", true)->value().toFloat();
        if (request->hasParam("sf", true))   sf   = request->getParam("sf", true)->value().toInt();
        if (request->hasParam("cr", true))   cr   = request->getParam("cr", true)->value().toInt();
        if (request->hasParam("sync", true)) syncWord = strtol(request->getParam("sync", true)->value().c_str(), nullptr, 16);
        if (request->hasParam("preamble", true)) preamble = request->getParam("preamble", true)->value().toInt();
        if (request->hasParam("crc", true))  crcOn = request->getParam("crc", true)->value() == "1";
        if (request->hasParam("iq", true))   iqInverted = request->getParam("iq", true)->value() == "1";
        applyRadioConfig();
        request->send(200, "text/plain", "OK");
    });

    server.on("/record/start", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!recording) {
            dumpFile = LittleFS.open("/dump.loradump", FILE_WRITE);
            recording = true;
        }
        request->send(200, "text/plain", "Recording started");
    });

    server.on("/record/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (recording) {
            dumpFile.close();
            recording = false;
        }
        request->send(200, "text/plain", "Recording stopped");
    });

    server.on("/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/dump.loradump")) LittleFS.remove("/dump.loradump");
        packetIndex = 0;
        request->send(200, "text/plain", "Cleared");
    });

    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        // не требуется обработка входящих сообщений
    });
    server.addHandler(&ws);
    server.begin();
}

// ---------- Основной цикл ----------
void loop() {
    if (packetReceived) {
        packetReceived = false;

        uint8_t buf[255];
        size_t len = radio.getPacketLength();
        int state = radio.readData(buf, len);
        if (state == RADIOLIB_ERR_NONE) {
            float rssi = radio.getRSSI();
            float snr  = radio.getSNR();

            // Сохраняем в кольцевой буфер
            Packet &p = packets[packetIndex % MAX_PACKETS];
            p.timeMs = millis();
            p.rssi = rssi;
            p.snr = snr;
            p.len = len;
            memcpy(p.data, buf, len);
            packetIndex++;

            // Отправляем JSON через WebSocket
            JsonDocument doc;
            doc["t"] = p.timeMs;
            doc["rssi"] = p.rssi;
            doc["snr"] = p.snr;
            doc["len"] = p.len;
            String hex = "";
            for (size_t i = 0; i < len; i++) {
                char h[3]; sprintf(h, "%02X", buf[i]); hex += h;
            }
            doc["hex"] = hex;
            String ascii = "";
            for (size_t i = 0; i < len; i++) {
                ascii += (buf[i] >= 32 && buf[i] < 127) ? (char)buf[i] : '.';
            }
            doc["ascii"] = ascii;
            String out; serializeJson(doc, out);
            ws.textAll(out);

            // Запись в файл
            if (recording) {
                uint32_t t = p.timeMs;
                dumpFile.write((uint8_t*)&t, 4);
                dumpFile.write((uint8_t*)&p.rssi, 4);
                dumpFile.write((uint8_t*)&p.snr, 4);
                dumpFile.write((uint8_t*)&p.len, 1);
                dumpFile.write(buf, len);
            }
        }
        radio.startReceive(); // снова в RX
    }

    ws.cleanupClients();
}
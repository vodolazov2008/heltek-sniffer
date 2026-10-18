#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <Preferences.h>      // NEW: для сохранения настроек
#include <U8g2lib.h>         // NEW: для OLED

// ---------- Пины SX1262 ----------
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_NRST  12
#define LORA_BUSY  13

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// ---------- OLED (Heltec V3: SCL=18, SDA=17, RST=21, addr 0x3C) ----------
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/18, /* data=*/17, /* reset=*/21);

// ---------- Параметры по умолчанию ----------
float   freq       = 868.0;
float   bw         = 125.0;
uint8_t sf         = 7;
uint8_t cr         = 5;
uint8_t syncWord   = 0x12;
uint16_t preamble  = 8;
bool    crcOn      = true;
bool    iqInverted = false;

// ---------- NVS ----------
Preferences prefs;
const char* NVS_NAMESPACE = "lorasniff";

// ---------- Состояние ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
volatile bool packetReceived = false;
bool recording = false;
File dumpFile;
uint32_t packetCount = 0;      // NEW: общий счётчик принятых пакетов

const int MAX_PACKETS = 50;
struct Packet {
    uint32_t timeMs;
    float rssi;
    float snr;
    uint8_t len;
    uint8_t data[255];
};
Packet packets[MAX_PACKETS];
int packetIndex = 0;

void IRAM_ATTR onDio1() {
    packetReceived = true;
}

// ---------- Сохранение / загрузка настроек ----------
void saveConfig() {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putFloat("freq", freq);
    prefs.putFloat("bw", bw);
    prefs.putUChar("sf", sf);
    prefs.putUChar("cr", cr);
    prefs.putUChar("sync", syncWord);
    prefs.putUShort("preamble", preamble);
    prefs.putBool("crc", crcOn);
    prefs.putBool("iq", iqInverted);
    prefs.end();
}

void loadConfig() {
    prefs.begin(NVS_NAMESPACE, true);
    freq       = prefs.getFloat("freq", freq);
    bw         = prefs.getFloat("bw", bw);
    sf         = prefs.getUChar("sf", sf);
    cr         = prefs.getUChar("cr", cr);
    syncWord   = prefs.getUChar("sync", syncWord);
    preamble   = prefs.getUShort("preamble", preamble);
    crcOn      = prefs.getBool("crc", crcOn);
    iqInverted = prefs.getBool("iq", iqInverted);
    prefs.end();
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
    radio.invertIQ(iqInverted);   // было setIQ, исправлено ранее
    radio.startReceive();
    Serial.printf("Radio: %.1f MHz, BW=%.1f, SF=%d, CR=4/%d, Sync=0x%02X\n",
                  freq, bw, sf, cr, syncWord);
}

// ---------- OLED ----------
void updateDisplay() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.setCursor(0, 10);
    u8g2.print("LoRa Sniffer V3");

    u8g2.setCursor(0, 24);
    u8g2.printf("Pkts: %lu", (unsigned long)packetCount);

    u8g2.setCursor(0, 38);
    u8g2.printf("IP: %s", WiFi.softAPIP().toString().c_str());

    u8g2.setCursor(0, 52);
    u8g2.printf("SF%d BW%.0f", sf, bw);

    u8g2.setCursor(0, 64);
    if (recording) {
        u8g2.print("REC");
    } else {
        u8g2.print("Idle");
    }

    u8g2.sendBuffer();
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- LoRa Sniffer Heltec V3 ---");

    // OLED
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 10);
    u8g2.print("Booting...");
    u8g2.sendBuffer();

    // LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    }

    // Загружаем сохранённые настройки
    loadConfig();

    // Радио
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio init failed: %d\n", state);
        while (true) delay(1000);
    }
    radio.setDio1Action(onDio1);
    applyRadioConfig();

    // SoftAP
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
        if (request->hasParam("freq", true))     freq = request->getParam("freq", true)->value().toFloat();
        if (request->hasParam("bw", true))       bw   = request->getParam("bw", true)->value().toFloat();
        if (request->hasParam("sf", true))       sf   = request->getParam("sf", true)->value().toInt();
        if (request->hasParam("cr", true))       cr   = request->getParam("cr", true)->value().toInt();
        if (request->hasParam("sync", true))     syncWord = strtol(request->getParam("sync", true)->value().c_str(), nullptr, 16);
        if (request->hasParam("preamble", true)) preamble = request->getParam("preamble", true)->value().toInt();
        if (request->hasParam("crc", true))      crcOn = request->getParam("crc", true)->value() == "1";
        if (request->hasParam("iq", true))       iqInverted = request->getParam("iq", true)->value() == "1";

        saveConfig();          // NEW: сохраняем в NVS
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
        // ничего не делаем с входящими сообщениями
    });
    server.addHandler(&ws);
    server.begin();

    updateDisplay();
}

// ---------- Loop ----------
void loop() {
    if (packetReceived) {
        packetReceived = false;

        uint8_t buf[255];
        size_t len = radio.getPacketLength();
        int state = radio.readData(buf, len);
        if (state == RADIOLIB_ERR_NONE) {
            float rssi = radio.getRSSI();
            float snr  = radio.getSNR();

            packetCount++;     // NEW: увеличиваем счётчик

            // Кольцевой буфер
            Packet &p = packets[packetIndex % MAX_PACKETS];
            p.timeMs = millis();
            p.rssi = rssi;
            p.snr = snr;
            p.len = len;
            memcpy(p.data, buf, len);
            packetIndex++;

            // JSON для WebSocket
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
        radio.startReceive(); // возвращаемся в RX
    }

    // Периодическое обновление OLED (раз в ~500 мс)
    static uint32_t lastDisplay = 0;
    if (millis() - lastDisplay > 500) {
        lastDisplay = millis();
        updateDisplay();
    }

    ws.cleanupClients();
}
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <U8g2lib.h>

// ---------- Пины SX1262 ----------
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_NRST  12
#define LORA_BUSY  13

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// ---------- OLED ----------
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT_PIN 36

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);

// ---------- Кнопка USER / PRG ----------
// На Heltec WiFi LoRa 32 V3 кнопка USER подключена к GPIO0.
// Внимание: GPIO0 — strapping-пин. Не удерживайте кнопку при включении/сбросе,
// иначе плата может войти в режим загрузчика вместо обычной работы.
#define BUTTON_PIN 0

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
uint32_t packetCount = 0;

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

// ---------- Режим сканирования ----------
#define PRESET_DURATION_MS 60000
#define MAX_PRESETS 32

struct Preset {
    const char* name;
    float   freq;
    float   bw;
    uint8_t sf;
    uint8_t sync;
};

const Preset presets[] = {
    // LoRaWAN РФ (Sync 0x34, BW 125)
    {"LW 864.0 SF7",  864.0, 125.0,  7, 0x34},
    {"LW 864.0 SF12", 864.0, 125.0, 12, 0x34},
    {"LW 864.9 SF7",  864.9, 125.0,  7, 0x34},
    {"LW 864.9 SF12", 864.9, 125.0, 12, 0x34},
    {"LW 868.8 SF7",  868.8, 125.0,  7, 0x34},
    {"LW 868.8 SF12", 868.8, 125.0, 12, 0x34},
    
    // Стандартный LoRaWAN EU868
    {"LW 868.1 SF7",  868.1, 125.0,  7, 0x34},
    {"LW 868.1 SF9",  868.1, 125.0,  9, 0x34},
    {"LW 868.1 SF12", 868.1, 125.0, 12, 0x34},
    
    // Приватные сети (Sync 0x12, BW 125)
    {"PV 868.7 SF7",  868.7, 125.0,  7, 0x12},
    {"PV 868.7 SF9",  868.7, 125.0,  9, 0x12},
    {"PV 868.8 SF7",  868.8, 125.0,  7, 0x12},
    {"PV 868.8 SF9",  868.8, 125.0,  9, 0x12},
};

const int presetCount = sizeof(presets) / sizeof(presets[0]);

bool scanMode = false;
int  currentPresetIdx = -1;
uint32_t presetStartTime = 0;
uint16_t presetHits[MAX_PRESETS] = {0};

float   savedFreq;
float   savedBw;
uint8_t savedSf, savedCr, savedSync;
uint16_t savedPreamble;
bool    savedCrc, savedIq;

// ---------- Антидребезг кнопки ----------
bool     buttonLastState       = HIGH;
bool     buttonStableState     = HIGH;
uint32_t buttonLastChangeMs    = 0;
uint32_t buttonPressStartMs    = 0;
bool     buttonWasPressed      = false;
const uint32_t DEBOUNCE_MS     = 50;
const uint32_t MIN_PRESS_MS    = 50;

// ---------- Прерывание DIO1 ----------
void IRAM_ATTR onDio1() {
    packetReceived = true;
}

// ---------- NVS ----------
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

// ---------- Радио ----------
void applyRadioConfig() {
    radio.standby();
    radio.setFrequency(freq);
    radio.setBandwidth(bw);
    radio.setSpreadingFactor(sf);
    radio.setCodingRate(cr);
    radio.setSyncWord(syncWord);
    radio.setPreambleLength(preamble);
    radio.setCRC(crcOn);
    radio.invertIQ(iqInverted);
    radio.startReceive();
    Serial.printf("Radio: %.1f MHz, BW=%.1f, SF=%d, CR=4/%d, Sync=0x%02X\n",
                  freq, bw, sf, cr, syncWord);
}

// ---------- Сканирование ----------
void applyPreset(int idx) {
    if (idx < 0 || idx >= presetCount) return;
    const Preset &p = presets[idx];
    packetReceived = false;
    radio.standby();
    radio.setFrequency(p.freq);
    radio.setBandwidth(p.bw);
    radio.setSpreadingFactor(p.sf);
    radio.setCodingRate(5);
    radio.setSyncWord(p.sync);
    radio.setPreambleLength(8);
    radio.setCRC(true);
    radio.invertIQ(false);
    radio.startReceive();
    Serial.printf("Scan preset %d/%d: %s\n", idx + 1, presetCount, p.name);
}

void startScan() {
    if (scanMode) return;
    savedFreq = freq; savedBw = bw; savedSf = sf; savedCr = cr;
    savedSync = syncWord; savedPreamble = preamble;
    savedCrc = crcOn; savedIq = iqInverted;

    memset(presetHits, 0, sizeof(presetHits));
    scanMode = true;
    currentPresetIdx = 0;
    presetStartTime = millis();
    applyPreset(0);
    Serial.println("Scan mode STARTED");
}

void stopScan() {
    if (!scanMode) return;
    scanMode = false;
    int lastIdx = currentPresetIdx;
    currentPresetIdx = -1;

    freq = savedFreq; bw = savedBw; sf = savedSf; cr = savedCr;
    syncWord = savedSync; preamble = savedPreamble;
    crcOn = savedCrc; iqInverted = savedIq;
    applyRadioConfig();
    Serial.printf("Scan mode STOPPED (last preset idx=%d)\n", lastIdx);
}

void checkScanTimer() {
    if (!scanMode) return;
    if (millis() - presetStartTime >= PRESET_DURATION_MS) {
        currentPresetIdx = (currentPresetIdx + 1) % presetCount;
        presetStartTime = millis();
        applyPreset(currentPresetIdx);
    }
}

// ---------- Обработка кнопки ----------
void checkButton() {
    bool raw = digitalRead(BUTTON_PIN);
    uint32_t now = millis();

    // Фиксируем изменение «сырого» состояния для антидребезга
    if (raw != buttonLastState) {
        buttonLastChangeMs = now;
        buttonLastState = raw;
    }

    // Если «сырое» состояние стабильно дольше DEBOUNCE_MS — принимаем его
    if (now - buttonLastChangeMs >= DEBOUNCE_MS) {
        if (raw != buttonStableState) {
            buttonStableState = raw;

            if (buttonStableState == LOW) {
                // Нажатие началось
                buttonPressStartMs = now;
                buttonWasPressed = true;
            } else {
                // Кнопка отпущена
                if (buttonWasPressed) {
                    uint32_t held = now - buttonPressStartMs;
                    buttonWasPressed = false;
                    if (held >= MIN_PRESS_MS) {
                        // Короткое нажатие — переключаем сканирование
                        if (scanMode) {
                            stopScan();
                        } else {
                            startScan();
                        }
                        Serial.printf("Button press (%lu ms) -> scan %s\n",
                                      (unsigned long)held, scanMode ? "ON" : "OFF");
                    }
                }
            }
        }
    }
}

// ---------- OLED ----------
void updateDisplay() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    if (scanMode && currentPresetIdx >= 0) {
        u8g2.setCursor(0, 10);
        u8g2.printf("SCAN %d/%d", currentPresetIdx + 1, presetCount);

        u8g2.setCursor(0, 24);
        u8g2.print(presets[currentPresetIdx].name);

        uint32_t elapsed = millis() - presetStartTime;
        int barW = (int)(elapsed * 124 / PRESET_DURATION_MS);
        if (barW > 124) barW = 124;
        u8g2.drawFrame(0, 28, 128, 6);
        if (barW > 0) u8g2.drawBox(2, 30, barW, 2);

        u8g2.setCursor(0, 46);
        u8g2.printf("Hits: %u  Total: %lu",
                    presetHits[currentPresetIdx], (unsigned long)packetCount);

        int found = -1;
        for (int i = 0; i < presetCount; i++) {
            if (presetHits[i] > 0) { found = i; break; }
        }
        u8g2.setCursor(0, 60);
        if (found >= 0) {
            u8g2.printf("Found: %s", presets[found].name);
        } else {
            u8g2.printf("No hits yet");
        }
    } else {
        u8g2.setCursor(0, 10);
        u8g2.print("LoRa Sniffer V3");

        u8g2.setCursor(0, 24);
        u8g2.printf("Pkts: %lu", (unsigned long)packetCount);

        u8g2.setCursor(0, 38);
        u8g2.printf("IP: %s", WiFi.softAPIP().toString().c_str());

        u8g2.setCursor(0, 52);
        u8g2.printf("SF%d BW%.0f", sf, bw);

        u8g2.setCursor(0, 64);
        u8g2.print(recording ? "REC" : "Idle");
    }

    u8g2.sendBuffer();
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- LoRa Sniffer Heltec V3 ---");

    // Кнопка USER (GPIO0) — active low, внутренняя подтяжка
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    buttonLastState = buttonStableState = digitalRead(BUTTON_PIN);

    // Питание Vext + OLED
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(100);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 10);
    u8g2.print("Booting...");
    u8g2.sendBuffer();

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    }

    loadConfig();

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

    // ---- GET /config ----
    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["freq"]      = freq;
        doc["bw"]        = bw;
        doc["sf"]        = sf;
        doc["cr"]        = cr;
        doc["sync"]      = String(syncWord, HEX);
        doc["preamble"]  = preamble;
        doc["crc"]       = crcOn ? "1" : "0";
        doc["iq"]        = iqInverted ? "1" : "0";
        doc["packets"]   = packetCount;
        doc["recording"] = recording;
        doc["scanning"]  = scanMode;
        String out; serializeJson(doc, out);
        request->send(200, "application/json", out);
    });

    // ---- POST /config ----
    server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (scanMode) stopScan();
        if (request->hasParam("freq", true))     freq = request->getParam("freq", true)->value().toFloat();
        if (request->hasParam("bw", true))       bw   = request->getParam("bw", true)->value().toFloat();
        if (request->hasParam("sf", true))       sf   = request->getParam("sf", true)->value().toInt();
        if (request->hasParam("cr", true))       cr   = request->getParam("cr", true)->value().toInt();
        if (request->hasParam("sync", true))     syncWord = strtol(request->getParam("sync", true)->value().c_str(), nullptr, 16);
        if (request->hasParam("preamble", true)) preamble = request->getParam("preamble", true)->value().toInt();
        if (request->hasParam("crc", true))      crcOn = request->getParam("crc", true)->value() == "1";
        if (request->hasParam("iq", true))       iqInverted = request->getParam("iq", true)->value() == "1";
        saveConfig();
        applyRadioConfig();
        request->send(200, "text/plain", "OK");
    });

    // ---- Сканирование ----
    server.on("/scan/start", HTTP_POST, [](AsyncWebServerRequest *request) {
        startScan();
        request->send(200, "text/plain", "Scan started");
    });

    server.on("/scan/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
        stopScan();
        request->send(200, "text/plain", "Scan stopped");
    });

    server.on("/scan/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["active"]  = scanMode;
        doc["current"] = currentPresetIdx;
        doc["total"]   = presetCount;
        doc["elapsed"] = scanMode ? (millis() - presetStartTime) : 0;
        doc["duration"] = PRESET_DURATION_MS;

        JsonArray arr = doc["presets"].to<JsonArray>();
        for (int i = 0; i < presetCount; i++) {
            JsonObject p = arr.add<JsonObject>();
            p["name"] = presets[i].name;
            p["freq"] = presets[i].freq;
            p["bw"]   = presets[i].bw;
            p["sf"]   = presets[i].sf;
            p["sync"] = String(presets[i].sync, HEX);
            p["hits"] = presetHits[i];
        }

        String out; serializeJson(doc, out);
        request->send(200, "application/json", out);
    });

    // ---- Запись ----
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

    server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/dump.loradump")) {
            request->send(LittleFS, "/dump.loradump", "application/octet-stream", true);
        } else {
            request->send(404, "text/plain", "No dump");
        }
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {});
    server.addHandler(&ws);
    server.begin();

    updateDisplay();
}

// ---------- Loop ----------
void loop() {
    checkButton();

    if (packetReceived) {
        packetReceived = false;

        uint8_t buf[255];
        size_t len = radio.getPacketLength();
        int state = radio.readData(buf, len);
        if (state == RADIOLIB_ERR_NONE) {
            float rssi = radio.getRSSI();
            float snr  = radio.getSNR();

            packetCount++;
            if (scanMode && currentPresetIdx >= 0) {
                if (presetHits[currentPresetIdx] < 0xFFFF) {
                    presetHits[currentPresetIdx]++;
                }
            }

            Packet &p = packets[packetIndex % MAX_PACKETS];
            p.timeMs = millis();
            p.rssi = rssi;
            p.snr = snr;
            p.len = len;
            memcpy(p.data, buf, len);
            packetIndex++;

            JsonDocument doc;
            doc["t"]    = p.timeMs;
            doc["rssi"] = p.rssi;
            doc["snr"]  = p.snr;
            doc["len"]  = p.len;
            if (scanMode) doc["scan"] = currentPresetIdx;

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

            if (recording) {
                uint32_t t = p.timeMs;
                dumpFile.write((uint8_t*)&t, 4);
                dumpFile.write((uint8_t*)&p.rssi, 4);
                dumpFile.write((uint8_t*)&p.snr, 4);
                dumpFile.write((uint8_t*)&p.len, 1);
                dumpFile.write(buf, len);
            }
        }
        radio.startReceive();
    }

    checkScanTimer();

    static uint32_t lastDisplay = 0;
    if (millis() - lastDisplay > 500) {
        lastDisplay = millis();
        updateDisplay();
    }

    ws.cleanupClients();
}
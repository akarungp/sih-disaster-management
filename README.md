/* Household Node for ESP32
   Features:
   - GSM (TinyGSM) HTTP poll for alerts + SMS fallback
   - LoRa broadcast of JSON status
   - Pushbutton "I'M HERE" with interrupt & debounce
   - Alarm + LED + optional ultrasonic emitter control
   - Non-blocking tasks using FreeRTOS
   Configure pins and APN/URLs before using.
*/

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <LoRa.h>
#include <WiFi.h> // used only for millis() / timers, not wifi network
#include <ArduinoJson.h>
#include <Preferences.h>

// -------- CONFIG --------
#define GSM_RX_PIN 16    // to GSM TX
#define GSM_TX_PIN 17    // to GSM RX
#define GSM_POWER_PIN 4  // optional power control for GSM module
const char APN[] = "your_apn_here";
const char GSM_USER[] = "";
const char GSM_PASS[] = "";
const char ALERT_SERVER[] = "http://example.com/get_alert?district=MYDIST"; // replace

// LoRa pins (common for SX127x modules on many ESP32 boards)
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26
const long LORA_FREQ = 915E6; // change to 433E6 or 866E6 depending on region

#define BUZZER_PIN  13
#define LED_PIN     2
#define BUTTON_PIN  34 // use input pin that supports interrupts
#define ULTRASONIC_PIN 12 // optional

// timing
const unsigned long POLL_INTERVAL_MS = 30UL * 1000UL; // poll server every 30s
const unsigned long LOUD_ALARM_MS = 10UL * 1000UL; // alarm duration
const int MAX_HTTP_RETRIES = 3;

// -------- GLOBALS --------
HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);
Preferences prefs;

volatile bool btnPressedFlag = false;
volatile unsigned long btnLast = 0;

String deviceId;
uint32_t seq = 0;

// ---------- helpers ----------
uint32_t crc32Of(const uint8_t *data, size_t length) {
  // standard CRC32 from ESP32 SDK
  return crc32_le(0, data, length);
}

String makeStatusJSON(const char* status, bool alarmActive) {
  StaticJsonDocument<256> doc;
  doc["device_id"] = deviceId;
  doc["seq"] = seq++;
  doc["status"] = status;
  doc["alarm"] = alarmActive;
  doc["ts"] = (unsigned long)(time(NULL)); // if RTC set; otherwise epoch 0
  // optional fields: battery, RSSI, lat/lon if available
  String out;
  serializeJson(doc, out);
  return out;
}

void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - btnLast < 250) return; // simple debounce (250ms)
  btnLast = now;
  btnPressedFlag = true;
}

// ---------- LoRa send ----------
void sendLoRaPacket(const String &json) {
  // create small integrity header: size + CRC32
  uint32_t crc = crc32Of((const uint8_t*)json.c_str(), json.length());
  LoRa.beginPacket();
  LoRa.print("{\"p\":");
  LoRa.print(json);
  LoRa.print(",\"crc\":");
  LoRa.print(crc);
  LoRa.print("}");
  LoRa.endPacket();
}

// ---------- Alarm control ----------
void activateAlarm(unsigned long durationMs) {
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(durationMs));
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(ULTRASONIC_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

// ---------- GSM alert fetch (HTTP) ----------
bool fetchAlertFromServer(String &alertJson) {
  // returns true if new alert JSON received, false otherwise
  if (!modem.networkConnect(APN, GSM_USER, GSM_PASS)) {
    // no GPRS
    return false;
  }
  for (int attempt=0; attempt<MAX_HTTP_RETRIES; ++attempt) {
    if (client.connect("example.com", 80)) {
      // simple HTTP GET
      client.print(String("GET ") + "/get_alert?district=MYDIST HTTP/1.1\r\n" +
                   "Host: example.com\r\nConnection: close\r\n\r\n");
      unsigned long start = millis();
      // wait for response headers
      while (client.connected() && millis() - start < 5000) {
        if (client.available()) break;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      String response;
      while (client.available()) {
        String line = client.readStringUntil('\n');
        response += line + '\n';
      }
      client.stop();
      // parse minimal: assume server returns JSON in body
      int jsonStart = response.indexOf("{");
      if (jsonStart >= 0) {
        alertJson = response.substring(jsonStart);
        return true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000 * (attempt+1)));
  }
  return false;
}

// ---------- SMS fallback (simple) ----------
bool checkSMSForAlert(String &alertJson) {
  // checks unread SMS messages for a prefixed alert
  // This is a simplified skeleton; actual TinyGSM AT commands might be required per module
  String sms;
  if (modem.sendAT(GF("+CMGL=\"REC UNREAD\""))) {
    // read SMS list, then parse messages with prefix ALERT:
    // Implement module-specific parsing here if you use SMS fallback.
    // For brevity: return false here
    return false;
  }
  return false;
}

// ---------- Tasks ----------
void taskGSM(void* pvParameters) {
  String alertJson;
  unsigned long lastPoll = 0;
  for (;;) {
    unsigned long now = millis();
    if (now - lastPoll >= POLL_INTERVAL_MS) {
      lastPoll = now;
      // ensure modem is awake/connected
      if (!modem.begin()) {
        // try to restart modem power if available
        digitalWrite(GSM_POWER_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(GSM_POWER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(2000));
        modem.restart();
      }
      if (fetchAlertFromServer(alertJson)) {
        // parse JSON to see severity/type — for demo, activate alarm for any alert
        Serial.printf("[GSM] Alert JSON: %s\n", alertJson.c_str());
        // notify via LoRa
        String payload = makeStatusJSON("ALERT_RECEIVED", true);
        sendLoRaPacket(payload);
        // Local alarm for loud duration (non-blocking approach below triggers)
        // Instead of blocking here, flip a flag and let main loop handle
        // For demo, short alarm:
        activateAlarm(LOUD_ALARM_MS);
      } else {
        // no alert; optionally send heartbeat
        String heartbeat = makeStatusJSON("OK", false);
        sendLoRaPacket(heartbeat);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskButton(void* pvParameters) {
  for (;;) {
    if (btnPressedFlag) {
      btnPressedFlag = false;
      Serial.println("[BTN] Button pressed: sending presence message over LoRa");
      String payload = makeStatusJSON("PRESENT", false);
      sendLoRaPacket(payload);
      // quick LED blink
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(LED_PIN, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Household Node booting...");

  // pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // Preferences / device id
  prefs.begin("node", false);
  deviceId = prefs.getString("device_id", "");
  if (deviceId.length() == 0) {
    // generate a simple ID using MAC
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[32];
    sprintf(buf, "NODE-%02X%02X%02X", mac[3], mac[4], mac[5]);
    deviceId = String(buf);
    prefs.putString("device_id", deviceId);
  }
  prefs.end();

  Serial.printf("Device ID: %s\n", deviceId.c_str());

  // LoRa init
  SPI.begin(); // use VSPI default pins (MISO=19 MOSI=23 SCK=18) adjust if necessary
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Starting LoRa failed!");
    // continue but LoRa won't work
  } else {
    Serial.println("LoRa OK");
  }

  // GSM init
  SerialGSM.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  pinMode(GSM_POWER_PIN, OUTPUT);
  digitalWrite(GSM_POWER_PIN, HIGH);
  delay(1000);
  if (!modem.restart()) {
    Serial.println("Warning: GSM modem restart failed (but continuing).");
  } else {
    Serial.println("GSM modem ready");
  }

  // Create tasks
  xTaskCreatePinnedToCore(taskGSM, "GSMTask", 6*1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskButton, "ButtonTask", 4*1024, NULL, 1, NULL, 1);
}

// ---------- main loop ----------
void loop() {
  // keep loop small — all work in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}


// === SIH Prototype: Disaster Management Gadget ===


#define HELP_CENTER_NUMBER "+911234567890"   
#define DEVICE_ID "VILLAGEX-HOUSE12"         


#define GSM_RX_PIN 16 
#define GSM_TX_PIN 17 
#define GSM_BAUD   115200


#define ALARM_PIN       18         
#define ULTRASONIC_PIN  19    
#define BUTTON_PIN      21        
#define LED_PIN         2            


bool alertActive = false;
unsigned long alertActivatedAt = 0;
const unsigned long ALARM_DURATION = 5 * 60 * 1000UL; // 5 min

HardwareSerial SerialAT(1);


void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Disaster Gadget Initializing...");

  
  pinMode(ALARM_PIN, OUTPUT); digitalWrite(ALARM_PIN, LOW);
  pinMode(ULTRASONIC_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  
  const int channel = 0;
  ledcSetup(channel, 40000, 8);
  ledcAttachPin(ULTRASONIC_PIN, channel);
  ledcWrite(channel, 0); // off initially

  
  SerialAT.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(300);
  initModem();

  Serial.println("[READY] Gadget is Online.");
}


void loop() {
  checkSMS();          // Listen for disaster alerts
  checkButton();       // Survivor "I'm Alive" response
  manageAlertState();  // Handle siren + ultrasonic timing
}




void initModem() {
  Serial.println("[GSM] Initializing...");
  SerialAT.println("AT");
  delay(200);
  SerialAT.println("AT+CMGF=1"); // SMS in text mode
  delay(200);
  Serial.println("[GSM] Ready for SMS.");
}


void checkSMS() {
  while (SerialAT.available()) {
    String line = SerialAT.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) Serial.println("[GSM-IN] " + line);

    if (line.indexOf("ALERT") >= 0) {
      triggerAlert();
    }
  }
}


void triggerAlert() {
  Serial.println("[ALERT] Disaster Alert Received!");
  digitalWrite(ALARM_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  ledcWrite(0, 128);  // Enable ultrasonic emitter
  alertActive = true;
  alertActivatedAt = millis();
}


void manageAlertState() {
  if (alertActive && millis() - alertActivatedAt > ALARM_DURATION) {
    Serial.println("[ALERT] Auto timeout reached. Turning OFF alarm.");
    digitalWrite(ALARM_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    ledcWrite(0, 0); // stop ultrasonic
    alertActive = false;
  }
}


void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("[BUTTON] Survivor present. Sending ALIVE SMS...");
    sendAliveSMS();
    delay(1000); 
  }
}


void sendAliveSMS() {
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(HELP_CENTER_NUMBER);
  SerialAT.println("\"");
  delay(200);
  SerialAT.print("ALIVE: ");
  SerialAT.print(DEVICE_ID);
  SerialAT.println(" - Survivor confirmed.");
  SerialAT.write(26); // Ctrl+Z to send
  Serial.println("[GSM] Alive SMS Sent.");
}

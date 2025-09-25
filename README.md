# sih-disaster-management
/* 
  ESP32 - Household Disaster Alert Gadget (SIM800 text-mode SMS)
  - Receives an SMS alert from Help Centre (text-mode)
  - On alert: sounds alarm (ALARM_PIN) and enables ultrasonic emitter (ULTRASONIC_PIN)
  - Push-button: press to send "I AM ALIVE" SMS to HELP_CENTER_NUMBER
  - Uses Hardware Serial1 to communicate with GSM module (SIM800/SIM800L)
  - Author: Adapted for SIH prototype
*/

#define HELP_CENTER_NUMBER "+911234567890"   
#define DEVICE_ID "VILLAGEX-HOUSE12"         


const int GSM_RX_PIN = 16; 
const int GSM_TX_PIN = 17; 
const long GSM_BAUD = 115200;


const int ALARM_PIN = 18;         
const int ULTRASONIC_PIN = 19;    
const int BUTTON_PIN = 21;        
const int LED_PIN = 2;            


unsigned long alertActivatedAt = 0;
bool alertActive = false;
unsigned long lastSmsCheck = 0;
const unsigned long SMS_POLL_INTERVAL = 2000; 
const unsigned long ALARM_DURATION = 5 * 60 * 1000UL; 


HardwareSerial SerialAT(1);

String incomingLine = "";
String lastSMSFrom = "";
String lastSMSBody = "";

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Disaster Gadget Booting ===");

 
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

  pinMode(ULTRASONIC_PIN, OUTPUT);

  const int channel = 0;
  const int freq = 40000;
  const int resolution = 8;
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(ULTRASONIC_PIN, channel);
  ledcWrite(channel, 0);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  
  SerialAT.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(300);

  initModem();
 

#define RX_PIN 3
#define GREEN_LED 4
#define RED_LED 5

uint8_t lastReceivedVal = 255; 
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT_MS = 1000; // זמן ההמתנה לפני שמכריזים על נתק (שנייה 1)

bool isConnected = false; 

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  // מתחילים ממצב של "אין תקשורת"
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  
  delay(1500); // השהייה קטנה להתחברות ה-USB
  Serial.println("--- Uno R4 Receiver with LEDs & Prints ---");
  Serial.println("Waiting for signal...");
}

void loop() {
  // 1. כלב שמירה (Watchdog) - האם עבר יותר מדי זמן (נתק)?
  if (millis() - lastReceiveTime > TIMEOUT_MS) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH); // נורה אדומה - אין קליטה
    
    if (isConnected) {
      Serial.println("\n>>> WARNING: Connection Lost! <<<");
      isConnected = false;
    }
  }

  // 2. חיפוש תחילת פולס סנכרון
  unsigned long syncHigh = pulseIn(RX_PIN, HIGH, 50000); 

  if (syncHigh > 600 && syncHigh < 1400) {
    unsigned long tSyncLowStart = micros();
    while (digitalRead(RX_PIN) == LOW && (micros() - tSyncLowStart < 2000));
    unsigned long syncLow = micros() - tSyncLowStart;

    if (syncLow > 600 && syncLow < 1400) {
      unsigned long t0 = micros(); 
      uint16_t receivedVal = 0; 
      bool error = false;

      // קריאת 9 הביטים (1 התחלה + 8 מידע)
      for (int i = 0; i < 9; i++) {
        while (micros() - t0 < (i * 1000 + 250));
        int v1 = digitalRead(RX_PIN);
        
        while (micros() - t0 < (i * 1000 + 750));
        int v2 = digitalRead(RX_PIN);

        if (v1 == HIGH && v2 == LOW) receivedVal |= (1 << (8 - i)); 
        else if (v1 == LOW && v2 == HIGH) { /* Do nothing */ } 
        else { error = true; break; }
      }

      if (!error && ((receivedVal >> 8) & 1)) {
        uint8_t finalData = receivedVal & 0xFF;
        if (finalData != lastReceivedVal || (millis() - lastReceiveTime > 200)) {
          // Output for Python script
          Serial.write(finalData);
          
          lastReceivedVal = finalData;
          lastReceiveTime = millis();
        }
      }
    }
  }
}

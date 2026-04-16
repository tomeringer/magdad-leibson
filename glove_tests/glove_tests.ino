#include <Arduino.h>

constexpr int NUM_FLEX = 5; 
// REORDERED PINS: f0(A3), f1(A0), f2(A2), f3(A7), f4(A1)
int flexPins[NUM_FLEX] = {A3, A0, A2, A7, A1}; 

constexpr float R_FIXED = 47000.0f;
constexpr float V_IN = 3.3f;
constexpr int NUM_SAMPLES = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Listen for the state commands from Python
    if (cmd >= '0' && cmd <= '3') {
      
      for (int s = 0; s < NUM_SAMPLES; s++) {
        for (int i = 0; i < NUM_FLEX; i++) {
          
          long sum = 0;
          for (int k = 0; k < 5; k++) {
            sum += analogRead(flexPins[i]);
          }
          
          float v = (sum / 5.0f) * (V_IN / 4095.0f);
          float r = 0.0f;
          
          // Hardware-level safety to prevent infinity (inf)
          if (v > 0.01f) {
            if (v >= V_IN - 0.01f) {
              r = 1000000.0f; // Cap at 1M Ohm
            } else {
              r = R_FIXED * (v / (V_IN - v));
            }
          }
          
          Serial.print(r, 1);
          if (i < NUM_FLEX - 1) {
            Serial.print(",");
          }
        }
        Serial.println();
        delay(2); 
      }
      Serial.println("DONE"); 
    }
  }
}
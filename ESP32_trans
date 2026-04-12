#include "driver/rmt.h"

#define TX_PIN 5  // GPIO 5 תואם לפין D2 הפיזי ב-Nano ESP32
#define HALF_BIT 500 

uint8_t testCounter = 0; // מונה רץ לבדיקת הטווח והרציפות

void setup() {
  Serial.begin(115200);
  
  // הגדרת חומרת ה-RMT
  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = RMT_CHANNEL_0;
  config.gpio_num = (gpio_num_t)TX_PIN;
  config.mem_block_num = 1;
  config.clk_div = 80; 
  config.tx_config.loop_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);

  Serial.println("--- ESP32 Range Test Transmitter ---");
  Serial.println("Broadcasting counter automatically...");
}

void transmitManchester(uint8_t data) {
  rmt_item32_t items[45]; 
  int idx = 0;

  // 1. Preamble (חימום המקלט - 8 פולסים)
  for(int i = 0; i < 8; i++) {
    items[idx++] = {{{ 400, 1, 400, 0 }}}; 
  }

  // 2. Sync Pulse (פולס סנכרון)
  items[idx++] = {{{ 1000, 1, 1000, 0 }}};

  // 3. Start Bit (קבוע על '1' - מונע התנגשויות)
  items[idx++] = {{{ HALF_BIT, 1, HALF_BIT, 0 }}};

  // 4. קידוד המידע (8 ביטים)
  for (int i = 7; i >= 0; i--) {
    if ((data >> i) & 1) {
      items[idx++] = {{{ HALF_BIT, 1, HALF_BIT, 0 }}}; 
    } else {
      items[idx++] = {{{ HALF_BIT, 0, HALF_BIT, 1 }}}; 
    }
  }
  
  // 5. פולס סיום
  items[idx++] = {{{ 0, 0, 0, 0 }}};

  // 6. שידור משולש לאמינות
  for(int r = 0; r < 3; r++) {
    rmt_write_items(RMT_CHANNEL_0, items, idx, true);
    delay(15); 
  }
  
  Serial.print("Transmitted: ");
  Serial.println(data);
}

void loop() {
  // שולח את המספר הנוכחי
  transmitManchester(testCounter);
  
  // מקדם את המונה (יחזור ל-0 אחרי 255)
  testCounter++;
  
  // משדר כל 300 מילי-שניות
  delay(300); 
}

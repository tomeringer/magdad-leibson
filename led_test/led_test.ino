// --- Configuration ---
// Define the pins exactly as they are printed on your board
const int redPin = D5;
const int greenPin = D6;
const int bluePin = D7;

void setup() {
  // Tell the ESP32 these pins will send power OUT to the MOSFETs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  // Start with all LEDs OFF for safety
  setColor(0, 0, 0);
}

void loop() {
  // 1. Pure RED Test
  setColor(255, 0, 0);   
  delay(1000);           

  // 2. Pure GREEN Test
  setColor(0, 255, 0);
  delay(1000);

  // 3. Pure BLUE Test
  setColor(0, 0, 255);
  delay(1000);

  // 4. Color Mixing: YELLOW (Red + Green)
  setColor(255, 255, 0);
  delay(1000);
  
  // 5. Color Mixing: PURPLE (Red + Blue)
  setColor(255, 0, 255);
  delay(1000);

  // 6. Turn OFF before restarting the loop
  setColor(0, 0, 0);
  delay(1000);
}

// --- Helper Function ---
void setColor(int redValue, int greenValue, int blueValue) {
  // Send a rapid PWM signal to the MOSFETs to dim the colors
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}
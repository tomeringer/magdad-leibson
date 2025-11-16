// החלף לערך הנגד הידוע שלך (למשל, 10000 עבור 10k אוהם)
float R_KNOWN = 10000.0;

// הפין האנלוגי
int analogPin = A0;

// משתנה לספירת איטרציות
int iterationCounter = 0;

const byte START_BYTE = 0xAA;
const byte END_BYTE   = 0x55;
byte output;

void setup() {
  Serial.begin(115200); 
}

void loop() {
  // 1. קרא את הערך הדיגיטלי מהארדואינו
  int rawValue = analogRead(analogPin);

  // 2. בצע את החישוב הפיזיקלי
  //    (שים לב להבדל בנוסחה מהפעם הקודמת)
  float R_UNKNOWN = R_KNOWN * ( (float)rawValue / (1023.0 - (float)rawValue) );
  if (R_UNKNOWN > 45000){
    //Serial.print(R_UNKNOWN);
    //Serial.print("number ");
    Serial.println("1");
    byte output = 1;
  }
  else{
    //Serial.print(R_UNKNOWN);
    //Serial.print("number ");
    Serial.println("0");
    byte output = 0;
  }

  //Serial.write(START_BYTE);
  //Serial.write(output);
  //Serial.write(END_BYTE);




  iterationCounter++;
  delay(1000); 
}
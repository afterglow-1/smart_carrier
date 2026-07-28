const int zeroPin = 23;  // the number of the pushbutton pin

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
volatile unsigned int interruptCount = 0;  // .
unsigned int lastCount=0;
//外部中断回调函数1
void interruptFunction() { interruptCount++; }
void setup() {
  Serial.begin(115200);
  Serial.println("zero test");
  // OLED初始化
#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("1Gray test");
  u8g2.sendBuffer();
#endif
  pinMode(zeroPin,
          INPUT);  // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(zeroPin, interruptFunction, RISING);
  delay(20);
}




void loop() {
 
  if (interruptCount!=lastCount)
  {
    lastCount=interruptCount;
  Serial.println("---------------------------------------");
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);
  Serial.println(" times so far.");
  buttonState = digitalRead(zeroPin);
  Serial.print("Pin State: ");
  Serial.print(buttonState);
  }
  


}
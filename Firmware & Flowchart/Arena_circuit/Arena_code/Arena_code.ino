// Final code arena 

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> 
#include <Arduino.h>
#include "HX711.h"

const int relay1 = 14; 
const int relay2 = 22;

const int irSensorPin  = 16;
const int irSensorPin2 = 19;   // Relay2 trigger sensor
const int irSensorPin3 = 21;

#define PIN_TFT_CS   13
#define PIN_TFT_DC   17
#define PIN_TFT_RST  4

Adafruit_ST7735 tft = Adafruit_ST7735(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 33;
const int LOADCELL_SCK_PIN  = 32;

// ------------------------------
//  BITMAP IMAGE
// ------------------------------
const unsigned char PROGMEM smile_bmp[] = {
  0x00,0x00,0x00,0x00,0x00, 
  0x00,0x07,0x80,0x00,0x00, 
  0x00,0x1F,0xC0,0x00,0x00, 
  0x00,0x7F,0xC0,0x00,0x00, 
  0x01,0xF8,0x1E,0x00,0x00, 
  0x03,0xC0,0x1E,0x00,0x00, 
  0x07,0x00,0x0E,0x00,0x00, 
  0x0C,0x00,0x06,0x00,0x00, 
  0x18,0x00,0x0C,0x00,0x00, 
  0x30,0x11,0x06,0x00,0x00, 
  0x60,0x11,0x03,0x00,0x00, 
  0x40,0x00,0x01,0x00,0x00, 
  0x40,0x0E,0x01,0x00,0x00, 
  0x60,0x00,0x03,0x00,0x00, 
  0x30,0x00,0x06,0x00,0x00, 
  0x18,0x00,0x0C,0x00,0x00, 
  0x0C,0x00,0x06,0x00,0x00, 
  0x03,0x00,0x06,0x00,0x00, 
  0x00,0xF0,0x0F,0x00,0x00, 
  0x00,0x00,0x00,0x00,0x00, 
  0x00,0x00,0x00,0x00,0x00 
};

HX711 scale;

// ------------------------------
//  NON-BLOCKING RELAY2 TIMER
// ------------------------------
unsigned long relay2TimerStart = 0;
bool relay2Active = false;
unsigned long relay2Delay = 5000;   // 5 seconds auto-off time


// ------------------------------
//  SETUP
// ------------------------------
void setup() {
  pinMode(relay1, OUTPUT); 
  pinMode(relay2, OUTPUT); 
  digitalWrite(relay1, HIGH); 
  digitalWrite(relay2, HIGH); 

  pinMode(irSensorPin,  INPUT); 
  pinMode(irSensorPin2, INPUT);
  pinMode(irSensorPin3, INPUT);

  // TFT
  tft.initR(INITR_BLACKTAB); 
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // HX711
  Serial.begin(57600);
  Serial.println("HX711 Demo");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println(scale.read());
  Serial.println(scale.read_average(20));
  Serial.println(scale.get_value(5));
  Serial.println(scale.get_units(5), 1);

  scale.set_scale(419.667);
  scale.tare();

  Serial.println(scale.read());
  Serial.println(scale.read_average(20));
  Serial.println(scale.get_value(5));
  Serial.println(scale.get_units(5), 1);
}

// ------------------------------
// DISPLAY FUNCTION
// ------------------------------
void displayPhoto(int a) {
  tft.fillScreen(ST77XX_GREEN);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.println("Circuit_RAJA");
  tft.setTextSize(1);
  tft.setCursor(30, 40);
  tft.println(a);
  tft.drawBitmap(60, 60, smile_bmp, 40, 40, ST77XX_BLUE);
  delay(1000);
}


// ------------------------------
// MAIN LOOP
// ------------------------------
void loop() {

  int a=0;
  int sensorState  = digitalRead(irSensorPin);
  int sensorState2 = digitalRead(irSensorPin2);
  int sensorState3 = digitalRead(irSensorPin3);

  // --------------------------------------------------
  // RELAY1 CONTROL (UNCHANGED)
  // --------------------------------------------------
  if (sensorState == LOW) {
    digitalWrite(relay1, LOW);
  }

  // --------------------------------------------------
  // RELAY2 CONTROL WITH NON-BLOCKING TIMER (NEW)
  // --------------------------------------------------
  if (sensorState2 == LOW) {
    digitalWrite(relay2, LOW);           // Activate relay2
    relay2TimerStart = millis();         // Start countdown
    relay2Active = true;                 // Enable auto-off logic
  }

  // Auto-turn-off relay2 after 5 seconds
  if (relay2Active && (millis() - relay2TimerStart >= relay2Delay)) {
    digitalWrite(relay2, HIGH);          // Turn relay2 OFF automatically
    relay2Active = false;                // Stop timer
  }


  // --------------------------------------------------
  // HX711 WEIGHING + TFT DISPLAY (UNCHANGED)
  // --------------------------------------------------
  if (sensorState3 == LOW) { 
    for (int f = 0; f < 10; f++) {
      Serial.print("one reading:\t");
      Serial.print(scale.get_units(), 1);
      Serial.print("\t| average:\t");

      a = scale.get_units(10);
      Serial.println(scale.get_units(10), 5);

      delay(500);
      displayPhoto(a);


    }

    delay(5000);
    tft.fillScreen(ST77XX_BLACK);
  }
}
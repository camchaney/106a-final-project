//#include "neo_driver.h"
#include <Adafruit_NeoPixel.h>

// Constants
#define PIN        13 // On Trinket or Gemma, suggest changing this to 1
//#define NUMPIXELS 64 // Popular NeoPixel ring size
// #define NUMPIXELS 1    // one neopixel
#define NUMPIXELS 16    // one neopixel
const int pin_use = 0;        // single pin to use for single mode
const int pin_pot = A3;       // pin for potentiometer dimming
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
const float alpha = 0.7;

// Variables
int sensorValue = 4096;       // 12 bit max
int dimValue = 255;
int cmdState = 0;
int lightState = 0;           // To-do: make into object (dim val,color,etc.)

// Object Initialization
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//DrillDriver TheDriver(/*direction*/ 10, /*speed*/ 9);

void setup() {
  // For reasons beyond me, it is important that this get called in setup.
  // Attempting to attach the servos in the global constructor causes weird
  // behavior.
  //TheDriver.init();

  // Pin modes
  //pinMode(pin_pot, INPUT);

  // Begin functions
  Serial.begin(9600);
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  analogReadResolution(12);
  Serial.println("Pixel set up");
}

void loop() {
  // Potentiometer values
  sensorValue = analogRead(pin_pot);
  int dimMeas = map(sensorValue, 0, 4096, 0, 255);    // (measurement) Lowest value = 1340
  dimValue = alpha * dimMeas + (1 - alpha)*dimValue;      // value smoothing
  //Serial.println(dimValue);

  if (Serial.available())
  {
    char input = Serial.read();

    switch(input)
    {
      // For now only on/off
      // To-Do: get working for dimming and color change
      case '1':
        // Go to 255 for full blast
        cmdState = 1;
        break;
      case '0':
        cmdState = 0;
        break;
    }
  }

  if (cmdState) {
    pixels.setPixelColor(pin_use, pixels.Color(0, dimValue, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    lightState = 1;
    //Serial.println("on");
  } else {
    pixels.setPixelColor(pin_use, pixels.Color(0, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    lightState = 0;
    //Serial.println("off");
  }
}

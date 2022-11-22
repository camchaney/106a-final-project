//#include "neo_driver.h"
#include <Adafruit_NeoPixel.h>

// Constants
#define PIN        13 // On Trinket or Gemma, suggest changing this to 1
//#define NUMPIXELS 64 // Popular NeoPixel ring size
#define NUMPIXELS 1    // one neopixel
const int pin_pot = A3;       // pin for potentiometer dimming
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

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
  int sensorValue = analogRead(pin_pot);
  int mapValue = map(sensorValue, 1340, 4096, 0, 255);    // Lowest value = 1340
  Serial.println(sensorValue);

  if (Serial.available())
  {
    char input = Serial.read();

    switch(input)
    {
      // For now only on/off
      // To-Do: get working for dimming and color change
      case '1':
        // Go to 255 for full blast
        pixels.setPixelColor(0, pixels.Color(0, 150, 0));
        pixels.show();   // Send the updated pixel colors to the hardware.
        break;
      case '0':
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();   // Send the updated pixel colors to the hardware.
        break;
    }
  }
}

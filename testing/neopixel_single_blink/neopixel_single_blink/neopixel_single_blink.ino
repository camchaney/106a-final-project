// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>

// Constants
#define PIN        13 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 64 // Popular NeoPixel ring size
const int pin_pot = A3;       // pin for potentiometer dimming



// Object Initialization

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

void setup() {
  Serial.begin(9600);
  //pinMode(pin_pot, INPUT);
  analogReadResolution(12);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  Serial.println("Pixel set up");
}

void loop() {
  //pixels.clear(); // Set all pixel colors to 'off' (for some reason this doesn't work with 1 neopixel)
  pixels.setPixelColor(32, pixels.Color(0, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  Serial.println("off");
  delay(DELAYVAL); // Pause before next pass through loop

  // The first NeoPixel in a strand is 0, second is 1, all the way up
  // to the count of pixels minus one.
  //for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
  // Read potentiometer value
  int sensorValue = analogRead(pin_pot);
  int mapValue = map(sensorValue, 0, 1, 0, 255);
  Serial.println(digitalRead(mapValue));
  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  // Here we're using a moderately bright green color:
  pixels.setPixelColor(32, pixels.Color(0, 150, 0));

  pixels.show();   // Send the updated pixel colors to the hardware.
  Serial.println("on");

  delay(DELAYVAL); // Pause before next pass through loop
  //}
}

// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        13 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 64 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

void setup() {
  Serial.begin(9600);

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

  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  // Here we're using a moderately bright green color:
  pixels.setPixelColor(32, pixels.Color(0, 150, 0));

  pixels.show();   // Send the updated pixel colors to the hardware.
  Serial.println("on");

  delay(DELAYVAL); // Pause before next pass through loop
  //}
}

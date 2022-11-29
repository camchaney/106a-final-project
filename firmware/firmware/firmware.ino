//#include "neo_driver.h"
#include <Adafruit_NeoPixel.h>

// Constants
#define PIN        13 // On Trinket or Gemma, suggest changing this to 1
//#define NUMPIXELS 64 // Popular NeoPixel ring size
// #define NUMPIXELS 1    // one neopixel
#define NUMPIXELS 16    // one neopixel
int pin_use = 0;        // single pin to use for single mode
const int pin_pot = A3;       // pin for potentiometer dimming
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
const float alpha = 0.7;
// Serial
const byte numChars = 32;     // number of bytes in received chars

// Variables
int sensorValue = 4096;       // 12 bit max
int dimValue = 255;
//int cmdState = 0;
int cmdState = 1;             // keeping at 1 for new mode
int lightState = 0;           // To-do: make into object (dim val,color,etc.)
// Serial
char receivedChars[numChars];
boolean newData = false;
// Light control
int R = 0;
int G = 0;
int B = 0;

// Object Initialization
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//DrillDriver TheDriver(/*direction*/ 10, /*speed*/ 9);

// Main functions -----------------------------------------------------------------------
void setup() {
  // For reasons beyond me, it is important that this get called in setup.
  // Attempting to attach the servos in the global constructor causes weird
  // behavior.
  //TheDriver.init();

  // Pin modes
  //pinMode(pin_pot, INPUT);

  // Begin functions
  Serial.begin(115200);
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

  // Serial read
  // if (Serial.available())
  // {
  //   char input = Serial.read();
  //   switch(input)
  //   {
  //     // For now only on/off
  //     // To-Do: get working for dimming and color change
  //     case '1':
  //       // Go to 255 for full blast
  //       cmdState = 1;
  //       break;
  //     case '0':
  //       cmdState = 0;
  //       break;
  //   }
  // }
  recvWithStartEndMarkers();


  if (newData == true) {
    parseData();
    newData = false;
  }

  if (cmdState) {
    pixels.setPixelColor(pin_use, pixels.Color(R, G, B));
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

// Subfunctions ------------------------------------------------------------------------
void recvWithStartEndMarkers() {
  // Input receiver
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
 
  while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          }
          else {
              receivedChars[ndx] = '\0'; // terminate the string
              recvInProgress = false;
              ndx = 0;
              newData = true;
          }
      }

      else if (rc == startMarker) {
          recvInProgress = true;
      }
  }
}

void parseData {
  // Input parser
  // Input in form: <R,G,B,i>
  //  - R = red value from (0,255)
  //  - G = green value from (0,255)
  //  - B = blue value from (0,255)
  //  - i = index of light to actuate

  char * strtokIndx; // this is used by strtok() as an index
  //string sep = ",";
  
  // Red value
  strtokIndx = strtok(receivedChars,",");   // Get first set
  R = atoi(strtokIndx);

  // Green value
  strtokIndx = strtok(NULL, ",");           // Get second set
  G = atoi(strtokIndx);

  // Blue value
  strtokIndx = strtok(NULL, ",");           // Get third set
  B = atoi(strtokIndx);

  // LED index
  strtokIndx = strtok(NULL, ",");           // Get third set
  pin_use = atoi(strtokIndx);
}

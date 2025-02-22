/// @file    BasicPromotionalLights.ino
/// @brief   Basic green lighting
/// @author  Cole Hetherington

#include <FastLED.h>

#define LENGTH 50 // length of each of the strips. careful, this can easily overflow the ram
#define BRIGHTNESS 100 // 0 - 100 of brightness

// Define the arrays of LEDs
CRGB pin2[LENGTH];
// CRGB pin3[LENGTH];
// CRGB pin4[LENGTH];
// CRGB pin5[LENGTH];

void setup() { 
  // Add all the LEDs
	FastLED.addLeds<WS2811,2,RGB>(pin2,LENGTH);
  // FastLED.addLeds<WS2811,3,RGB>(pin3,LENGTH);
  // FastLED.addLeds<WS2811,4,RGB>(pin4,LENGTH);
  // FastLED.addLeds<WS2811,5,RGB>(pin5,LENGTH);
  FastLED.setBrightness(BRIGHTNESS);
  
}

void setLights(int r, int g, int b) {
  for (int i = 0; i < LENGTH; i++){s
    pin2[i] = CRGB(r, g, b);
    // pin3[i] = CRGB(r, g, b);
    // pin4[i] = CRGB(r, g, b);
    // pin5[i] = CRGB(r, g, b);
  }
}

void loop() {
  setLights(0, 0, 255);
  FastLED.show();
}

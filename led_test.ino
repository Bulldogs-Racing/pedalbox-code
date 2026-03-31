#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN  A10
#define NUM_PIXELS    20
#define BRIGHTNESS 100
#define MIN_VOLTAGE 0
#define MAX_VOLTAGE 5

Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int mode = 0;

void setAll(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();

  Serial.println("NeoPixel Color Toggle");
  Serial.println("Press ENTER in Serial Monitor to cycle colors");
  Serial.println("Current: OFF");
}

void loop() {
  if(Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int val = input.toInt();

    if(input.length() > 0 && val >= 0 && val <= 255) {
      strip.setBrightness(val);
      strip.show();
      Serial.print("Brightness set to: ");
      Serial.println(val);
    }
    else {
      mode++;
      if(mode > 6) mode = 0;

      switch(mode) {
        case 0: setAll(0,0,0); Serial.println("OFF"); break;
        case 1: setAll(255,0,0); Serial.println("RED"); break;
        case 2: setAll(0,255,0); Serial.println("GREEN"); break;
        case 3: setAll(0,0,255); Serial.println("BLUE"); break;
        case 4: setAll(252,240,3); Serial.println("YELLOW"); break;
        case 5: setAll(152,3,252); Serial.println("PURPLE"); break;
        case 6: setAll(252,3,219); Serial.println("PINK"); break;
      }
    }
  }
}
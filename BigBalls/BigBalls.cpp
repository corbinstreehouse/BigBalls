//
//  BigBalls.cpp
//  Index
//
//  Created by Corbin Dunn on 7/20/18.
//

#include "BigBalls.h"
#include "FastLED.h"
#include "LEDPatterns.h"
#include "SPI.h"

LEDPatterns patterns(NUM_LEDS);
uint32_t lastTime = 0;
int lastType = LEDPatternTypeMin;

void setup() {
#if DEBUG
    Serial.begin(19200);
    delay(5);
    Serial.println("--- begin serial --- (WARNING: delay on start!!)");
#endif
    SPI.begin();
    
    // Set the initial pattern
    patterns.setPatternType(LEDPatternTypeFire);
    patterns.setPatternDuration(50);
    patterns.setPatternColor(CRGB::Red);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(patterns.getLEDs(), NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness( BRIGHTNESS);
    //    FastLED.setMaxPowerInVoltsAndMilliamps(5.0, 18000);
    
    lastTime = millis();
}

void loop() {
    // 5 seconds, and switch patterns
    if (millis() - lastTime > 5000) {
        lastTime = millis();
        lastType++;
        if (lastType == LEDPatternTypeBitmap || lastType == LEDPatternTypeFadeOut || lastType == LEDPatternTypeDoNothing) {
            lastType++; // skip patterns that we can't deal with
        }
        // loop back to the start
        if (lastType >= LEDPatternTypeCount) {
            lastType == LEDPatternTypeMin;
        }
        patterns.setPatternType((LEDPatternType)lastType);
    }
    
    patterns.show();
}

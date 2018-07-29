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



///////////////// PARAMETERS YOU CAN TWEAK
#define WAIT_FADE_DURATION (5*1000) // Duration the fade takes when in the wiating state, in ms (5 seconds)
#define WAIT_TIME_BEFORE_COLOR_CHANGE (WAIT_FADE_DURATION*2) // Go to the next rainbow color after this amount of time in MS. IE: every 3 seconds the hue changes. This should be much bigger than the last value..


LEDPatterns g_patterns(NUM_LEDS);
uint32_t g_lastTimeInMS = 0; // in milliseconds

typedef CD_ENUM(int16_t, CDBallState)  {
    CDBallStateWaiting,
    CDBallStateInitialMovePattern,
    CDBallStateDirectionalPoint,
    CDBallStateBadDirection
};

CDBallState g_ballState = CDBallStateWaiting;

void gotoWaitingState();

bool checkBallMoved() {
    // TODO: This should return YES if it was moved and starts some other state transitions
    return false;
}


void setup() {
#if DEBUG
    Serial.begin(19200);
    delay(5);
    Serial.println("--- begin serial --- (WARNING: delay on start!!)");
#endif
    SPI.begin();
    
    // Set the initial pattern
    g_patterns.setPatternType(LEDPatternTypeFire);
    g_patterns.setPatternDuration(50);
    g_patterns.setPatternColor(CRGB::Red);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(g_patterns.getLEDs(), NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness( BRIGHTNESS);
    //    FastLED.setMaxPowerInVoltsAndMilliamps(5.0, 18000);
    
    gotoWaitingState();
}

uint8_t g_lastHueColor = 0;

void doWaiting() {
    // Should we glow to the next color?
    if (g_lastTimeInMS - millis() >= WAIT_TIME_BEFORE_COLOR_CHANGE) {
        // Yup
        g_lastHueColor++; // unsigned 8 bit.... auto wrap past 255 back to 0.
        CHSV hsv = CHSV(g_lastHueColor, 255, 255);
        CRGB color;
        hsv2rgb_rainbow(hsv, color);
        g_patterns.setPatternColor(color);
    }
    g_patterns.show();
}

void doInitialMovePattern() {
    
}

void doDirectionalPoint() {
    
}

void doBadDirection() {
    
}

void gotoWaitingState() {
    // Do a low-level glow of various colors; start the timer and set the pattern to show
    g_lastTimeInMS = millis();
    g_patterns.setPatternType(LEDPatternTypeFadeInFadeOut);
    g_patterns.setPatternDuration(WAIT_FADE_DURATION); // in ms

    // Generate a rainbow color that changes every X seconds
    CHSV hsv = CHSV(g_lastHueColor, 255, 255);
    CRGB color;
    hsv2rgb_rainbow(hsv, color);
    g_patterns.setPatternColor(color);
    
    g_patterns.show();
}

void loop() {
    switch (g_ballState) {
        case CDBallStateWaiting: {
            doWaiting();
            break;
        }
        case CDBallStateInitialMovePattern: {
            doInitialMovePattern();
            break;
        }
        case CDBallStateDirectionalPoint: {
            doDirectionalPoint;
            break;
        }
        case CDBallStateBadDirection: {
            doBadDirection();
            break;
        }
    }
    
    g_patterns.show();
}

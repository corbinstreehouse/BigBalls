//
//  BigBalls.cpp
//  Index
//
//  Created by Corbin Dunn on 7/20/18.
//

#include "BigBalls.h"
#include "FastLED.h"

///////////////// PARAMETERS YOU CAN TWEAK
#define WAIT_FADE_DURATION (5*1000) // Duration the fade takes when in the wiating state, in ms (5 seconds)
#define WAIT_TIME_BEFORE_COLOR_CHANGE (WAIT_FADE_DURATION*2) // Go to the next rainbow color after this amount of time in MS. IE: every 3 seconds the hue changes. This should be much bigger than the last value..

// How long to do the initial color flash when initially moved
#define INITIAL_MOVE_DURATION (3*1000) // in ms. X seconds

#define WAIT_TIME_BEFORE_GOING_TO_SLEEP (10*1000) // in ms. 10 seconds with no movement, and then go to sleep mode again (soft glow)


#if DEBUG

    #define FAKE_A_MOVE_TEST_DURATION (10*1000) // in ms. after X seconds pretend we moved

#endif


///////////

#include "LEDPatternType.h" // Defines CD_ENUM

// Non static globals
LEDPatterns g_patterns(NUM_LEDS);
CRGB *g_LEDs;
CRGB *g_LEDsPastEnd;


static uint32_t g_lastTimeInMS = 0; // in milliseconds
static uint32_t g_lastMovedTime = 0;

typedef CD_ENUM(int16_t, CDBallState)  {
    CDBallStateWaiting,
    CDBallStateInitialMovePattern,
    CDBallStateDirectionalPoint,
    CDBallStateBadDirection
};

CDBallState g_ballState = CDBallStateWaiting;

static void gotoWaitingState();


#if DEBUG
static uint32_t g_movedTestTime = 0;
#endif
static float g_lastDirectionInDegrees = 0; // Till i get vectors hookedup

bool checkBallMoved() {
    bool result = false;
    // TODO: This should return YES if it was moved and starts some other state transitions
#if DEBUG
    if (g_movedTestTime == 0) {
        g_movedTestTime = millis(); // initialize for debugging
    }
    if ((millis() - g_movedTestTime) > FAKE_A_MOVE_TEST_DURATION) {
        Serial.printf("DEBUG!! Faking a move after %f seconds.\r\n", FAKE_A_MOVE_TEST_DURATION/1000.0);
        result = true;
    }
#endif
    // keep track of how long we last moved
    if (result) {
        g_lastMovedTime = millis();
    }
    return result;
}

bool hasNotMovedInWhile() {
    if (checkBallMoved()) {
        return false;
    }
    // If we haven't moved in a while, then wait for a bit before we stop doing the directional point
    if ((millis() - g_lastMovedTime) >= WAIT_TIME_BEFORE_GOING_TO_SLEEP) {
        return true;
    }
    return false;
}


void setup() {
#if DEBUG
    Serial.begin(19200);
    delay(5);
    Serial.println("--- begin serial --- (WARNING: delay on start!!)");
#endif
    // Set the initial pattern
    g_patterns.setPatternType(LEDPatternTypeFire);
    g_patterns.setPatternDuration(50);
    g_patterns.setPatternColor(CRGB::Red);

    // Save off the LED pointer
    g_LEDs = g_patterns.getLEDs();
    g_LEDsPastEnd = &g_LEDs[NUM_LEDS];
    
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(g_LEDs, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness( BRIGHTNESS);
    //    FastLED.setMaxPowerInVoltsAndMilliamps(5.0, 18000);
    
    initializeBall();
    
#if DEBUG
    g_movedTestTime = millis();
#endif
    gotoWaitingState();
}

uint8_t g_lastHueColor = 0;

static void doWaiting() {
    // Should we glow to the next color?
    if (g_lastTimeInMS - millis() >= WAIT_TIME_BEFORE_COLOR_CHANGE) {
        g_lastHueColor++; // unsigned 8 bit.... auto wrap past 255 back to 0.
        CHSV hsv = CHSV(g_lastHueColor, 255, 255);
        CRGB color;
        hsv2rgb_rainbow(hsv, color);
        g_patterns.setPatternColor(color);
    }
    g_patterns.show();
}

// TODO: vector stuff instead of degrees
static float getDirectionalVector() {
    // TODO: vector return
#if DEBUG
    return g_lastDirectionInDegrees; //
#endif
    return 0;
}

static void gotoDirectionalPointState() {
#if DEBUG
    Serial.println("--- gotoDirectionalPointState");
    g_movedTestTime = millis(); // reset for debugging
#endif
    g_ballState = CDBallStateDirectionalPoint;
    
    // Clear the LEDs and set our pattern to be "nothing" so we can manually update individual ones with FastLED
    g_patterns.setPatternType(LEDPatternTypeDoNothing);
    
#if DEBUG
    g_lastDirectionInDegrees = g_lastDirectionInDegrees + 9; // change by 10 degrees each pass
#else
    g_lastDirectionInDegrees = getDirectionalVector();
#endif
    doDirectionalPoint(g_lastDirectionInDegrees);
}

static void updateDirectionalPointIfNeeded() {
    float possibleNewDirection = getDirectionalVector();
    if (g_lastDirectionInDegrees != possibleNewDirection) {
        g_lastDirectionInDegrees = possibleNewDirection;
        doDirectionalPoint(g_lastDirectionInDegrees);
    }
}

void doBadDirection() {
    g_patterns.flashThreeTimes(CRGB::Red); // Flash red (synchronous call)
    g_lastMovedTime = millis(); // Reset the time we were last moved to be now
    g_ballState = CDBallStateDirectionalPoint; // Go back to directional pointing for the time
}

static void gotoWaitingState() {
#if DEBUG
    Serial.println("------- gotoWaitingState");
    g_movedTestTime = millis();
#endif
    g_ballState = CDBallStateWaiting;

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

#define SKIP_COUNT 9
// Don't do these patterns; they suck for this setup
static const LEDPatternType g_patternsToSkip[SKIP_COUNT] = { LEDPatternTypeSolidColor, LEDPatternTypeFadeOut, LEDPatternTypeFadeIn, LEDPatternTypeColorWipe, LEDPatternTypeDoNothing, LEDPatternTypeImageReferencedBitmap, LEDPatternTypeImageEntireStrip_UNUSED, LEDPatternTypeBitmap, LEDPatternTypeFadeInFadeOut };
static int g_NextMovePatternToUse = LEDPatternTypeMin;

void gotoInitialMovedState() {
#if DEBUG
    Serial.println("------- gotoInitialMovedState");
    
    
#endif

    // Do a pattern (not random) to indicate something is going to happen. Do it for 3 seconds, then flash
    g_ballState = CDBallStateInitialMovePattern;
//    g_patterns.flashThreeTimes(CRGB::Green); /// this call is synchronous and will return when done
    // Then a random cool pattern for INITIAL_MOVE_DURATION time duration
    g_lastTimeInMS = millis();
    g_patterns.setPatternType((LEDPatternType)g_NextMovePatternToUse);
    g_patterns.setPatternDuration(INITIAL_MOVE_DURATION/3); // in ms...not sure this is what i want to do, as each pattern has a better duration
    g_patterns.setPatternColor(CRGB::DarkViolet); // TODO: random color
    // Setup the next
    g_NextMovePatternToUse++;
    if (g_NextMovePatternToUse >= LEDPatternTypeCount) {
        g_NextMovePatternToUse = LEDPatternTypeMin;
    } else {
        // skip bad ones; order is important
        for (int i = 0; i < SKIP_COUNT; i++) {
            if (g_NextMovePatternToUse == g_patternsToSkip[i]) {
                g_NextMovePatternToUse++;
            } else {
                break;
            }
        }
    }
}

void doInitialMovePattern() {
    // Do this pattern until the required time and then go to the direction state
    if ((millis() - g_lastTimeInMS) >= INITIAL_MOVE_DURATION) {
#if DEBUG
        Serial.printf("Initial move pattern done after %f seconds\r\n", INITIAL_MOVE_DURATION/1000.0);
#endif
        gotoDirectionalPointState();
    }
}

        

void loop() {
    switch (g_ballState) {
        case CDBallStateWaiting: {
            if (checkBallMoved()) {
                gotoInitialMovedState();
            } else {
                doWaiting();
            }
            break;
        }
        case CDBallStateInitialMovePattern: {
            doInitialMovePattern();
            break;
        }
        case CDBallStateDirectionalPoint: {
            // Are we done directional?
            if (hasNotMovedInWhile()) {
                gotoWaitingState();
            } else {
                // Highlight the new direction, if needed
                updateDirectionalPointIfNeeded();
            }
            break;
        }
        case CDBallStateBadDirection: {
            doBadDirection();
            break;
        }
    }
    
    g_patterns.show();
}

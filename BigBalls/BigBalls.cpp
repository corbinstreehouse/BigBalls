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

#define WAIT_TIME_BEFORE_GOING_TO_SLEEP (3*1000) // in ms. X seconds with no movement, and then go to sleep mode again (soft glow)

#define ACCELEROMETER_DIFF_TO_CONSIDER_MOVED 0.5 // Smaller values make it more sensitive; larger values make it less sensitive

#define FAKE_MOVE_EVENTS 0 // Set to 1 to test fake moving
#define FAKE_A_MOVE_TEST_DURATION (2*1000) // in ms. after X seconds pretend we moved (if FAKE_MOVE_EVENTS == 1)

///////////

#include "LEDPatternType.h" // Defines CD_ENUM


// Non static globals
LEDPatterns g_patterns(NUM_LEDS);
CRGB *g_LEDs;
CRGB *g_LEDsPastEnd;
Adafruit_BNO055 g_bno = Adafruit_BNO055();


static uint32_t g_lastTimeInMS = 0; // in milliseconds

typedef CD_ENUM(int16_t, CDBallState)  {
    CDBallStateWaiting,
    CDBallStateInitialMovePattern,
    CDBallStateDirectionalPoint,
    CDBallStateBadDirection
};

static CDBallState g_ballState = CDBallStateWaiting;

static void gotoWaitingState();


#if DEBUG
static uint32_t g_movedTestTime = 0;
#endif
static float g_lastDirectionInDegrees = 0; // Till i get vectors hookedup


static imu::Vector<3> g_lastAccelValue;

#if DEBUG

static void printVector(imu::Vector<3> accel) {
    Serial.print("X: ");
    Serial.print(accel.x());
    Serial.print(" Y: ");
    Serial.print(accel.y());
    Serial.print(" Z: ");
    Serial.println(accel.z());
}
#endif

static bool checkBallMoved() {
    bool result = false;
#if FAKE_MOVE_EVENTS
    if (g_movedTestTime == 0) {
        g_movedTestTime = millis(); // initialize for debugging
    }
    if (g_ballState == CDBallStateWaiting) {
        if ((millis() - g_movedTestTime) > FAKE_A_MOVE_TEST_DURATION) {
            Serial.printf("DEBUG!! Faking a move after %f seconds.\r\n", FAKE_A_MOVE_TEST_DURATION/1000.0);
            result = true;
        }
    }
#endif
    
#if BNO_ENABLED
    
// Uncomment to debug the values
//    imu::Vector<3> accel = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//    printVector(accel);
    
    // We'll check the accelerometer to see if the thing was hit/moved;
    imu::Vector<3> accel = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if ((accel.x() - g_lastAccelValue.x() >= ACCELEROMETER_DIFF_TO_CONSIDER_MOVED) ||
        (accel.y() - g_lastAccelValue.y() >= ACCELEROMETER_DIFF_TO_CONSIDER_MOVED) ||
        (accel.z() - g_lastAccelValue.z() >= ACCELEROMETER_DIFF_TO_CONSIDER_MOVED))
    {
#if DEBUG
//        Serial.println("moved!");
//        
//        Serial.println(" -- old:");
//        printVector(g_lastAccelValue);
//        Serial.println(" -- accel:");
//        printVector(accel);

#endif
        // update the value at this point
        g_lastAccelValue = accel;
        result = true;
    }
#endif
    
    return result;
}

static bool hasNotMovedInWhile() {
    if (checkBallMoved()) {
        g_lastTimeInMS = millis();
        return false;
    }
    // If we haven't moved in a while, then wait for a bit before we stop doing the directional point
    if ((millis() - g_lastTimeInMS) >= WAIT_TIME_BEFORE_GOING_TO_SLEEP) {
        return true;
    }
    return false;
}

static void intializeBNO() {
#if BNO_ENABLED
    if (!g_bno.begin()) {
#if DEBUG
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
#endif
    } else {
        g_bno.setExtCrystalUse(true);
        delay(500); // It is slow to initialize ; wait half a second
        g_lastAccelValue = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    }
#endif
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
    
    intializeBNO();
    
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
    g_lastTimeInMS = millis();
#if DEBUG
    Serial.println("--- gotoDirectionalPointState");
    g_movedTestTime = millis(); // reset for debugging
#endif
    g_ballState = CDBallStateDirectionalPoint;
    
    // Clear the LEDs and set our pattern to be "nothing" so we can manually update individual ones with FastLED
    g_patterns.setPatternType(LEDPatternTypeDoNothing);
    
#if DEBUG
    g_lastDirectionInDegrees = g_lastDirectionInDegrees + 40; // change by 10 degrees each pass
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
    g_lastTimeInMS = millis(); // Reset the time we were last moved to be now
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

    // Generate a color that changes every X seconds
    CHSV hsv = CHSV(g_lastHueColor, 255, 255);
    CRGB color;
    hsv2rgb_rainbow(hsv, color);
    g_patterns.setPatternColor(color);
    
    g_patterns.show();
}

#if DEBUG
#include "RamMonitor.h"
static RamMonitor ram;
#endif

#define SKIP_COUNT 9
// Don't do these patterns; they suck for this  p
static const LEDPatternType g_patternsToSkip[SKIP_COUNT] = { LEDPatternTypeSolidColor, LEDPatternTypeFadeOut, LEDPatternTypeFadeIn, LEDPatternTypeColorWipe, LEDPatternTypeDoNothing, /*LEDPatternTypeImageEntireStrip_UNUSED, LEDPatternTypeBitmap, */LEDPatternTypeFadeInFadeOut };
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
    
#if DEBUG
    Serial.printf("Going to pattern #: %d (count %d). Free ram: %d, free heap: %d, free stack %d\r\n", g_NextMovePatternToUse, LEDPatternTypeCount, ram.free(), ram.heap_free(), ram.stack_free());
#endif
    
    // Setup the next
    g_NextMovePatternToUse++;
    if (g_NextMovePatternToUse >= LEDPatternTypeCount) {
        g_NextMovePatternToUse = LEDPatternTypeMin;
    } else {
        // skip bad ones; order is important
        for (int i = 0; i < SKIP_COUNT; i++) {
            if (g_NextMovePatternToUse >= LEDPatternTypeCount) {
                g_NextMovePatternToUse = LEDPatternTypeMin;
                break;
            }
            if (g_NextMovePatternToUse == g_patternsToSkip[i]) {
                g_NextMovePatternToUse++;
            }
        }
    }

}

static void doInitialMovePattern() {
    // Do this pattern until the required time and then go to the direction state
    if ((millis() - g_lastTimeInMS) >= INITIAL_MOVE_DURATION) {
#if DEBUG
        Serial.printf("Initial move pattern done after %f seconds\r\n", INITIAL_MOVE_DURATION/1000.0);
#endif
        gotoDirectionalPointState();
    }
}

void print_bno(sensors_event_t event) {
    /* The processing sketch expects data as roll, pitch, heading */
    Serial.print(F("Orientation: "));
    Serial.print((float)event.orientation.x);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));
    
    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.println(euler.z());
    //Serial.print("\t\t");
    
    /* Also send calibration data for each sensor. */
    uint8_t sys, gyro, accel, mag = 0;
    g_bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC);
}
#if DEBUG

static void doBNOTest() {
    
    
    sensors_event_t event;
    g_bno.getEvent(&event);
    
    print_bno(event);
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

}

static uint32_t g_lastBNOTestTime = 0;

#endif

void loop() {
#if DEBUG
    if (millis() - g_lastBNOTestTime >= 100) {
//        doBNOTest();
        g_lastBNOTestTime = millis();
    }
#endif
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

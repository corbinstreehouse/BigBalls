//
//  BigBalls.cpp
//  Index
//
//  Created by Corbin Dunn on 7/20/18.
//

#include "BigBalls.h"
#include "FastLED.h"
#include "TinyGPS++.h"
#include "LEDPatternType.h" // Defines CD_ENUM

///////////////// PARAMETERS YOU CAN TWEAK
#define WAIT_FADE_DURATION (5*1000) // Duration the fade takes when in the wiating state, in ms (5 seconds)
#define WAIT_TIME_BEFORE_COLOR_CHANGE (WAIT_FADE_DURATION*2) // Go to the next rainbow color after this amount of time in MS. IE: every 3 seconds the hue changes. This should be much bigger than the last value..

// How long to do the initial color flash when initially moved
#define INITIAL_MOVE_DURATION (2*1000) // in ms. X seconds
#define WAIT_TIME_BEFORE_GOING_TO_SLEEP (3*1000) // in ms. X seconds with no movement, and then go to sleep mode again (soft glow)

#define ACCELEROMETER_DIFF_TO_CONSIDER_MOVED 0.5 // Smaller values make it more sensitive; larger values make it less sensitive

#define FAKE_MOVE_EVENTS !BNO_ENABLED // Set to 1 to test fake moving
#define FAKE_A_MOVE_TEST_DURATION (2*1000) // in ms. after X seconds pretend we moved (if FAKE_MOVE_EVENTS == 1)

const double g_towerLat = 40.783979;
const double g_towerLong = -119.214196;

#define TEST_GPS 1

///////////

// Non static globals
LEDPatterns g_patterns(NUM_LEDS);
CRGB *g_LEDs;
CRGB *g_LEDsPastEnd;
Adafruit_BNO055 g_bno = Adafruit_BNO055();
TinyGPSPlus g_gps;
//AltSoftSerial g_gpsSerial; // TODO: GPS serial input!

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

int checksum(const char *s) {
    int c = 0;
    
    while(*s)
        c ^= *s++;
    
    return c;
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
    
#if FAKE_MOVE_EVENTS
    g_movedTestTime = millis();
#endif
    
#if TEST_GPS
    // Don't even bother reading this.
    // GPS test coordinates: 37.1221962,-122.0067858
    char *testGPSCoordinate = "GPRMC,162614,A,3707.37916,N,12200.44676,W,10.0,90.0,131006,1.2,E,A";
    g_gps.encode('$');
    char *c = testGPSCoordinate;
    while (*c) {
      //  Serial.print(*c);
        g_gps.encode(*c);
        c++;
    }
    g_gps.encode('*');
    char tmp[6];
    sprintf(tmp, "%02X\r\n", checksum(testGPSCoordinate));
    c = tmp;
    while (*c) {
      //  Serial.print(*c);
        g_gps.encode(*c);
        c++;
    }
    
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

// TODO: vector stuff instead of degrees; this is just returning the X
static float getDirectionalVector() {
    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return euler.x();
}

static void updateDirectionalPoint();

static void gotoDirectionalPointState() {
    g_lastTimeInMS = millis();
#if DEBUG
    Serial.println("--- gotoDirectionalPointState");
#endif
    
#if FAKE_MOVE_EVENTS
    g_movedTestTime = millis(); // reset for debugging
#endif
    g_ballState = CDBallStateDirectionalPoint;
    
    // Clear the LEDs and set our pattern to be "nothing" so we can manually update individual ones with FastLED
    g_patterns.setPatternType(LEDPatternTypeDoNothing);
    
    updateDirectionalPoint();
}

static void updateDirectionalPoint() {
    // Get an update from the sensor
    sensors_event_t event;
    g_bno.getEvent(&event);

    // Figure out the target direction to the tower, based on our current GPS location and the tower's lat/long
    double targetDirectionInDegrees = TinyGPSPlus::courseTo(g_gps.location.lat(),
                                            g_gps.location.lng(),
                                            g_towerLat,
                                            g_towerLong);
    imu::Quaternion orientation = g_bno.getQuat();
    doDirectionalPointWithOrientation(targetDirectionInDegrees, orientation);
}

void doBadDirection() {
    g_patterns.flashThreeTimes(CRGB::Red); // Flash red (synchronous call)
    g_lastTimeInMS = millis(); // Reset the time we were last moved to be now
    g_ballState = CDBallStateDirectionalPoint; // Go back to directional pointing for the time
}

static void gotoWaitingState() {
#if DEBUG
    Serial.println("------- gotoWaitingState");
#endif
#if FAKE_MOVE_EVENTS
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

#if DEBUG

void print_bno() {
    sensors_event_t event;
    g_bno.getEvent(&event);
    
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    
    /* The processing sketch expects data as roll, pitch, heading */
    Serial.printf("Orientation.x: %f\r\n", (float)event.orientation.x);
    Serial.printf("Orientation.y: %f\r\n", (float)event.orientation.y);
    Serial.printf("Orientation.z: %f\r\n", (float)event.orientation.z);
    
    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.printf("Euler.x: %f\r\n", euler.x());
    Serial.printf("Euler.y: %f\r\n", euler.y());
    Serial.printf("Euler.z: %f\r\n", euler.z());
    
    imu::Quaternion quaterion = g_bno.getQuat();
    Serial.printf("quaterion: %f\r\n", (float)quaterion.x());
    Serial.printf("quaterion: %f\r\n", (float)quaterion.y());
    Serial.printf("quaterion: %f\r\n", (float)quaterion.z());

    Serial.println();
    
//    /* Also send calibration data for each sensor. */
//    uint8_t sys, gyro, accel, mag = 0;
//    g_bno.getCalibration(&sys, &gyro, &accel, &mag);
//    Serial.print(F("Calibration: "));
//    Serial.print(sys, DEC);
//    Serial.print(F(" "));
//    Serial.print(gyro, DEC);
//    Serial.print(F(" "));
//    Serial.print(accel, DEC);
//    Serial.print(F(" "));
//    Serial.println(mag, DEC);
}

static void doBNOTest() {
//    print_bno();


}

static uint32_t g_lastBNOTestTime = 0;

#endif


static void updateGPSPosition() {
    // TODO: feed gps the serial input from whatever provides that.
    // Something like:
//    while (g_gpsSerial.available() > 0) {
//        g_gps.encode(g_gpsSerial.read());
//    }
    
#if DEBUG
    if (g_gps.location.isUpdated()) {
        double distanceKm = TinyGPSPlus::distanceBetween(
                                    g_gps.location.lat(),
                                    g_gps.location.lng(),
                                    g_towerLat,
                                    g_towerLong) / 1000.0;
    
        double courseTo = TinyGPSPlus::courseTo(
                         g_gps.location.lat(),
                         g_gps.location.lng(),
                         g_towerLat,
                         g_towerLong);

        Serial.printf("starting lat: %f long %f\r\n", g_gps.location.lat(), g_gps.location.lng());
        Serial.printf("Distance to Tower: %f km or %f miles\r\n", distanceKm, distanceKm*0.62137119);
        Serial.printf("Course to Tower: %f degrees\r\n", courseTo);
        Serial.print("Human directions: ");
        Serial.println(TinyGPSPlus::cardinal(courseTo));
    }
#endif
}

void loop() {
#if  DEBUG // BNO test...
    if (millis() - g_lastBNOTestTime >= 100) {
        doBNOTest();
        g_lastBNOTestTime = millis();
    }
#endif
    
    updateGPSPosition();
    
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
                updateDirectionalPoint();
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

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
#include "EEPROM.h"
#include "SoftwareSerial.h"

///////////////// PARAMETERS YOU CAN TWEAK
#define WAIT_FADE_DURATION (5*1000) // Duration the fade takes when in the wiating state, in ms (5 seconds)
#define WAIT_TIME_BEFORE_COLOR_CHANGE (WAIT_FADE_DURATION*2) // Go to the next rainbow color after this amount of time in MS. IE: every 3 seconds the hue changes. This should be much bigger than the last value..

// How long to do the initial color flash when initially moved
#define INITIAL_MOVE_DURATION (2*1000) // in ms. X seconds
#define WAIT_TIME_BEFORE_GOING_TO_SLEEP (3*1000) // in ms. X seconds with no movement, and then go to sleep mode again (soft glow)

#define ACCELEROMETER_DIFF_TO_CONSIDER_MOVED 0.3 // Smaller values make it more sensitive; larger values make it less sensitive

#define BNO_SENSORID_EEPROM_ADDRESS 0
#define BNO_CALIBRATION_EEPROM_ADDRESS (BNO_SENSORID_EEPROM_ADDRESS + sizeof(long))

#define ALWAYS_CALIBRATE_ON_START 1 // If 0, it only calibrates if we didn't save and restore calibration data

const double g_towerLat = 40.783979;
const double g_towerLong = -119.214196;

// If this is set; it will feed an initial GPS position for testing.
#define TEST_GPS 1

///////////

// Non static globals
LEDPatterns g_patterns(NUM_LEDS);
CRGB *g_LEDs;
CRGB *g_LEDsPastEnd;
Adafruit_BNO055 g_bno = Adafruit_BNO055();
TinyGPSPlus g_gps;
SoftwareSerial g_gpsSoftwareSerial(GPS_RX_PIN, GPS_TX_PIN);

static uint32_t g_lastTimeInMS = 0; // in milliseconds
static bool g_isDaytime = true;


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
    // We'll check the accelerometer to see if the thing was hit/moved;
    imu::Vector<3> accel = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if ((fabs(accel.x() - g_lastAccelValue.x()) >= ACCELEROMETER_DIFF_TO_CONSIDER_MOVED) ||
        (fabs(accel.y() - g_lastAccelValue.y()) >= ACCELEROMETER_DIFF_TO_CONSIDER_MOVED) ||
        (fabs(accel.z() - g_lastAccelValue.z()) >= ACCELEROMETER_DIFF_TO_CONSIDER_MOVED))
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

#if DEBUG
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");
    
    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");
    
    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");
    
    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);
    
    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    g_bno.getCalibration(&system, &gyro, &accel, &mag);
    
    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }
    
    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    g_bno.getSystemStatus(&system_status, &self_test_results, &system_error);
    
    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}
#endif

// return true if calibration is stored in EEPROM, else false
static bool loadBNOCalibration() {
    /*
     *  Look for the sensor's unique ID at the beginning oF EEPROM.
     *  This isn't foolproof, but it's better than nothing.
     */
    long bnoID;
    EEPROM.get(BNO_SENSORID_EEPROM_ADDRESS, bnoID);

    sensor_t sensor;
    g_bno.getSensor(&sensor);
    bool result = bnoID == sensor.sensor_id;
    if (result) {
        adafruit_bno055_offsets_t calibrationData;
        EEPROM.get(BNO_CALIBRATION_EEPROM_ADDRESS, calibrationData);
        g_bno.setSensorOffsets(calibrationData);
#if DEBUG
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        displaySensorOffsets(calibrationData);
        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        Serial.println("\n\nCalibration data loaded into BNO055");
#endif
    }
}

#define BNO055_SAMPLERATE_DELAY_MS (100)

static bool bnoIsCalibrated () {
    // Not sure why, but the accel never calibrates for me, so ignore it..
    
    uint8_t system, gyro, accel, mag;
    g_bno.getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || /*accel < 3 || */mag < 3)
        return false;
    return true;

}

static void calibrateBNO() {
    // Flash the LEDs red when calibrating..
    if (!bnoIsCalibrated()) {
        g_patterns.setPatternDuration(500);
        g_patterns.setPatternColor(CRGB::Pink);
        g_patterns.setPatternType(LEDPatternTypeBlink);
//        displaySensorStatus();
    }
    
    while (!bnoIsCalibrated())
    {
        sensors_event_t event;
        g_bno.getEvent(&event);
#if DEBUG
        Serial.print(" X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);
        Serial.println();
        displayCalStatus();
#endif
        /* Wait the specified delay before requesting new data */
        delay(BNO055_SAMPLERATE_DELAY_MS);
        g_patterns.show();
    }
}

static void saveBNOCalibration() {
    adafruit_bno055_offsets_t newCalib;
    g_bno.getSensorOffsets(newCalib);
#if DEBUG
    displaySensorOffsets(newCalib);
#endif
    sensor_t sensor;
    g_bno.getSensor(&sensor);
    long bnoID = sensor.sensor_id;
    
    EEPROM.put(BNO_SENSORID_EEPROM_ADDRESS, bnoID);
    const int calibrationDataAddress = BNO_SENSORID_EEPROM_ADDRESS + sizeof(long); // past the sensorID address
    EEPROM.put(BNO_CALIBRATION_EEPROM_ADDRESS, newCalib);
}

static void initializeBNO() {
    if (g_bno.begin()) {
        bool isCalibrated = loadBNOCalibration();
        
        delay(500); // It is slow to initialize ; wait half a second

        // Crystal must be configured AFTER loading calibration data into BNO055.
        g_bno.setExtCrystalUse(true); // corbin: why do we do this?

        g_lastAccelValue = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        
        // Always test calibration on setup? Not sure if we have to..
        if (ALWAYS_CALIBRATE_ON_START || !isCalibrated) {
            calibrateBNO(); // thing never seems to calibrate!
            // After calibrated...store the data in EEPROM
            saveBNOCalibration();
        }
        
    } else {
        g_patterns.flashThreeTimes(CRGB::Purple); // Flash purple for no BNO
#if DEBUG
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        delay(1000);
#endif
    }
}

int checksum(const char *s) {
    int c = 0;
    
    while(*s)
        c ^= *s++;
    
    return c;
}

static void initializeGPS() {
    g_gpsSoftwareSerial.begin(GPS_SERIAL_BAUD);
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
}

static inline void setupOffPins() {
    pinMode(PIN_OFF0, OUTPUT);
    digitalWrite(PIN_OFF0, LOW); // Going high kills power
    
    pinMode(PIN_OFF1, OUTPUT);
    digitalWrite(PIN_OFF1, LOW); // Going high kills power
    
    pinMode(PIN_OFF2, OUTPUT);
    digitalWrite(PIN_OFF2, LOW); // Going high kills power
}

static inline void setupPhotoTransistorPin() {
    pinMode(PHOTO_TRANISTOR_PIN, INPUT_PULLUP);
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
    FastLED.setBrightness(g_isDaytime ? BRIGHTNESS_AT_DAYTIME : BRIGHTNESS_AT_NIGHT);
    //    FastLED.setMaxPowerInVoltsAndMilliamps(5.0, 18000);
    g_patterns.flashThreeTimes(CRGB::Green);
    
    setupOffPins();
    setupPhotoTransistorPin();
    
    initializeBall();
    initializeBNO();
    initializeGPS();
    
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
        g_lastTimeInMS = millis();
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
    
    g_ballState = CDBallStateDirectionalPoint;
    
    // Clear the LEDs and set our pattern to be "nothing" so we can manually update individual ones with FastLED
    g_patterns.setPatternType(LEDPatternTypeDoNothing);
    
    updateDirectionalPoint();
}

static void updateDirectionalPoint() {
    // Get an update from the sensor
    sensors_event_t event;
    g_bno.getEvent(&event);

    TinyGPSLocation location = g_gps.location;
    if (location.isValid()) {
        // Figure out the target direction to the tower, based on our current GPS location and the tower's lat/long
        double targetDirectionInDegrees = TinyGPSPlus::courseTo(g_gps.location.lat(),
                                                g_gps.location.lng(),
                                                g_towerLat,
                                                g_towerLong);
        doDirectionalPointWithOrientation(targetDirectionInDegrees);
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

static void updateGPSPosition() {
    while (g_gpsSoftwareSerial.available() > 0) {
        g_gps.encode(g_gpsSoftwareSerial.read());
    }
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

void updateDaytimeStatus() {
    static int g_darkCounter = 0;
    if (digitalRead(PHOTO_TRANISTOR_PIN)) { // 0 for light, 1 for dark
        g_darkCounter++;
    } else {
        g_darkCounter--;
    }
    
    if (g_darkCounter >= 6) {   // 6 * 10s = 1 min to change over
        g_darkCounter--;          // go down so dark doesn't roll over
        g_isDaytime = false;
    }
    else if (g_darkCounter <= -6) { // 6 * 10s = 1 min to change over
        g_darkCounter++;             // go up so dark doesnt' roll under
        g_isDaytime = true;
    }
}



void loop() {    
    updateGPSPosition();
    
    EVERY_N_SECONDS(10) {
        updateDaytimeStatus();
        if (g_isDaytime) {
            // Maye change brightness?
            FastLED.setBrightness(BRIGHTNESS_AT_DAYTIME);
        } else {
            FastLED.setBrightness(BRIGHTNESS_AT_NIGHT);
        }
    }
    
//    EVERY_N_MILLISECONDS(100) {
//        batt = analogRead(VS_PIN);
//    }
    
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

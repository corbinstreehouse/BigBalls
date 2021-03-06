# BigBalls

##

**Accounting For BNO Offset/Orientation**

1. Point the ball with north (the intersection of pentagons 1-4) directly on north (use your phone), and the "up" pentagon set pointing directly up (pentagons 17-20). See "2018-08-13 ball faces renumbered.skp".
2. Run the code with DEBUG=1 or PRINT_OFFSET_QUAT=1
3. Copy the resulting offset quaternion that is output to the Serial, such as: ``imu::Quaternion offsetQuat = imu::Quaternion(0.210022, -0.222900, -0.668518, -0.677734);``
4. Open PlasticBall.cpp, find doDirectionalPointWithOrientation() at the bottom, and replace this line (the unit quaternion) with the one output above for your offset:
``    imu::Quaternion offsetQuat = imu::Quaternion(1.0, 0.0, 0.0, 0.0);``

**LED Test**

Set this to have it highlight the pentagons in order on start (in PlasticBall.cpp):

```// If this is 1, we will do a test on the start and highlight each pentagon in order so you can verify it
#define HIGHLIGHT_PENTAGONS_ON_START 0
```


**GPS Tower Reporting**

``sendGPSDataToTower()`` will send the GPS position and battery voltages to the tower every X seconds. Comment it out or update as needed:

```
Serial1.printf("Plastic Location: %f %f\r\n", location.lat(), location.lng());
for (int i = 0; i < 3; i++) {
  Serial1.printf("Plastic V%d: %f\r\n", i, g_batteryVoltages[i]);
}
```

**Voltage checking**

Update the following code to the pins that will be hooked up to SENSE the battery voltage (not directly to the input voltage, that will fry things):

`static const int VOLTAGE_READ_PINS[3] = { 14, 14, 14 };`

When a voltage is low, it will shut that battery off with these pins hooked up to the switch:

`static const int VOLTAGE_SHUTOFF_PINS[3] = { 16, 21, 14 };`

These **have** to be changed; I don't know what they should be based on the hardware. The first one is assumed to be the teeny's voltage supply; before shutting the teensy off, it will shut off the other batteries.

other values that need changing:

`#define MIN_BATTERY_VOLTAGE  6.3 // VOLTS, for 2-cell lipo`

Set the above value to whatever voltage your min voltage should be (probably higher for more cells).

The following values can also be changed based on analyzed values for resistors:

```#define REF_VOLTAGE 3.3 // TODO: this could be measured
#define RESISTOR_Z1_VALUE 100.0 // kOhm
#define RESISTOR_Z2_VALUE 15.0 // kOhm   (with 10uF filter cap in parallel)
```





## GUI use of git

Download a git client to make it easy to share source code changes. Try this:

https://desktop.github.com

Make a free account on github and sign in on the app.

Click “Clone a Repository” and find "corbinstreehouse/BigBalls"

Give it a Local Path on your hard drive and clone it.

Then open the BigBalls folder up in GitHub’s desktop client.





## Terminal use of git

If you don't want to use the GUI, you can use Terminal.

Update submodules by doing this from the terminal in the directory of the project:

`git submodule init`
`git submodule update`

You have to to a get submodule update each time you want to pull changes.

## Using Xcode

To use Xcode, install Arduino; it must be installed in /Applications. Install "teensyduino" (the teensy add on for Arduino). Build/Make/Upload targets should work to do each of those things. Make does a compile without any other changes. Build is a full rebuild. Upload starts the Teensy gui.

## Using Arduino

To use Arduino, copy the Libraries submodules to your Libraries folder. Then open BigBalls/BigBalls.ino and build.


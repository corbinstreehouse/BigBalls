# BigBalls

Update submodules by doing this from the terminal in the directory of the project:

`git submodule init`
`git submodule update`

To use Xcode, install Arduino; it must be installed in /Applications. Install "teensyduino" (the teensy add on for Arduino). Build/Upload targets should work.

To use Arduino, copy the Libraries submodules to your Libraries foldre. Then open BigBalls.ino and build.

See "Ball Orientation Side View" and "Ball Orientation Top View" to get a feel for how I'm orienting things.

An LED group is a group of LEDs on one pentagon. I think we are having 8 x 8 squares in each group (ie: 32 LEDs).

Where four pentagons intersect, we will have a "Ball Point".  Based on each point, we can identify an individual pentagon, such as "North West Up".  Wire the strips from #1 starting  in the "North West Up" group to "North east up" to "south east up" to "south west up", thus completing the "up" point. 

Test Change
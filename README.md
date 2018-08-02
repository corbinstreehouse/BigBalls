# BigBalls

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


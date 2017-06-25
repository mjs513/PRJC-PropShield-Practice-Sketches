
# PRJC-PropShield-Practice-Sketches
Repository of PropShield Sketchs

The Prop Shield is a combination of audio amplifier, LED driver and 10-dof motion sensor with 8MB of flash memory developed and sold by PRJC for their Teensy line of microprocessors.  For more information see https://www.pjrc.com/store/prop_shield.html.

This repository is a series of sketches that I used to test the Prop Shield with Motion Sensors.  Let me warn you that I did not develop any of the sketches in this library.  Most of them were developed by other users on the PRJC forum or are exmaple sketches from a couple of the associated libraries that you can load with Teensyduino.

This readme will document these sketches and my notes in learning a little about the audio and LED capabilities of the Prop Shield.

AUDIO
You will need the Teensy Audio,SD, SerialFlash libraries to run the audio sketches.  So lets start with some basic info that I learned:

1. All .mp3 and .wav files should be converted to 16-bit 44100hz files.  The audio library supports other formats as well: aac, flac and raw.  I used Audacity to do the conversion from mono 22khz.
2. As stated on the product page you have to make sure you turn on the ampifer.  This is done via the following lines in Setup in the Arduino sketch:
```c++
    void setup() {
      pinMode(5, OUTPUT);
      digitalWrite(5, HIGH); // turn on the amplifier
      delay(10);             // allow time to wake up
```
3. Go through the [Audio Library tutorial](https://www.pjrc.com/teensy/td_libs_Audio.html) and learn to use their [Audio Design Tool](https://www.pjrc.com/teensy/gui/index.html) if would definitely help.

    a.  To play mp3, wav or raw files you need to include one of the following lines in the header of sketch:
```c++
      #include <play_sd_mp3.h>
      #include <play_sd_wav.h>
      #include <play_sd_raw.h>
```
    b. In general to use SerialFlash, SD, Audio together the following lines should be included before headers in (a).
    
```c++
      #include <Audio.h>
      #include <Wire.h>
      #include <SPI.h>
      #include <SD.h>
      #include <SerialFlash.h>
```

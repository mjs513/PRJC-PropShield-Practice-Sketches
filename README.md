# PRJC-PropShield-Practice-Sketches
Repository of PropShield Sketchs

The Prop Shield is a combination of audio amplifier, LED driver and 10-dof motion sensor with 8MB of flash memory developed and sold by PRJC for their Teensy line of microprocessors.  For more information see https://www.pjrc.com/store/prop_shield.html.

This repository is a series of sketches that I used to test the Prop Shield with Motion Sensors.  Let me warn you that I did not develop any of the sketches in this library.  Most of them were developed by other users on the PRJC forum or are exmaple sketches from a couple of the associated libraries that you can load with Teensyduino.

This readme will document these sketches and my notes in learning a little about the audio and LED capabilities of the Prop Shield.

To help me get through the examples and explainings things to me I posted of the PRJC forum and @manitou really helped me.  Here is a link to that discussion:
https://forum.pjrc.com/threads/45015-Can-not-get-PropShield-to-Play-MP3-s-with-MP3Player

From Prop-Shield Beta Test Thread on the PRJC Forum I found the following entries interesting:
1. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=99662&viewfull=1#post99662 - FastLED linked to yaw angle from NXP motion sensors, posted by mortonkopf on Teensyforum
2. https://forum.pjrc.com/threads/27059-MP3-Player-Lib-with-example?p=101731&viewfull=1#post101731 - original play all from Frank B
3. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=99702&viewfull=1#post99702 - moving average of gyro drift
4. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=99891&viewfull=1#post99891 - SensorStatisics.ino by Ben. The Program calculates the mean and variance of the 9 sensor values and of the magnitudes of acceleration and magnetic field strength
5. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=99957&viewfull=1#post99957 -savings stats for plotting, Jbeale
6. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=99971&viewfull=1#post99971 - by defragster, nice speaker for propshield
7. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=100272&viewfull=1#post100272  , https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=100323&viewfull=1#post100323  interesting post by PaulStoffregen on orientation
8. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=100441&viewfull=1#post100441,  teensy+propsheild addition for processing sketch.  See following posts as well
9. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=100477&viewfull=1#post100477 = unused com ports
10. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=100665&viewfull=1#post100665 by wozzy, Prop Shield + APA102 + FastLED 
11. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=100876&viewfull=1#post100876 mod by KurtE to wozzy sketch
12. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=101105&viewfull=1#post101105
13. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=101152&viewfull=1#post101152 Attempt at making a Prop shield variant of the WavFilePlayer , MichaelMeissner
14. https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=101188&viewfull=1#post101188 - sound effect files post by MM

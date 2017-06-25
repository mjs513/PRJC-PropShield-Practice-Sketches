// Simple MP3 file player example
//
// Requires the prop-shield
// This example code is in the public domain.

#include <Audio.h>
#include <SerialFlash.h>
#include <play_sd_mp3.h>
//#include <play_sd_aac.h>


// GUItool: Setting up audio
AudioPlaySdMp3     playMp31;        //xy=154,422
AudioOutputAnalog  audioOutput;
AudioConnection    patchCord1(playMp31, audioOutput);

#define PROP_AMP_ENABLE    5
#define FLASH_CHIP_SELECT  6

void setup() {
  AudioMemory(20); //4
  analogReference(EXTERNAL);  //plays really loud, may get hissing
                              //if your speakers can't handle it.
  delay(2000);

  // Start SD Card
  if (!(SD.begin(BUILTIN_SDCARD))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  //Start Amplifier
  pinMode(PROP_AMP_ENABLE , OUTPUT);
  digitalWrite(PROP_AMP_ENABLE , HIGH); 
}

void playFile(const char *filename)
{

  File ff = SD.open(filename);
  Serial.print("Playing file: ");
  Serial.println(filename);  
  
  // Start playing the file.  This sketch continues to
  // run while the file plays.
  playMp31.play(filename);

  // Simply wait for the file to finish playing.
  while (playMp31.isPlaying()) { yield(); }
}


void loop() {
  playFile("odd1.mp3");
  delay(1000);
}

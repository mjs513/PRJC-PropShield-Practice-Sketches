// Requires the prop-shield
// This example code is in the public domain.

#include <string.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <play_sd_mp3.h>
#include <play_sd_aac.h>
#include <play_sd_flac.h>

// GUItool: begin automatically generated code
AudioPlaySdMp3           playMp3; //xy=154,422
AudioPlaySdWav           playWav; //xy=154,422
AudioPlaySdRaw           playRaw; //xy=154,422
AudioPlaySdAac           playAac; //xy=154,422
AudioPlaySdFlac          playFlac;
AudioMixer4              mixer1;         //xy=323,326
AudioMixer4              mixer2;         //xy=323,326
AudioMixer4              mixer3;
AudioOutputAnalog        dac1;           //xy=445,393
AudioConnection          patchCord1(playMp3, 0, mixer1, 0);
AudioConnection          patchCord2(playMp3, 1, mixer1, 1);
AudioConnection          patchCord3(playWav, 0, mixer1, 2);
AudioConnection          patchCord4(playWav, 1, mixer1, 3);
AudioConnection          patchCord5(playAac, 0, mixer2, 0);
AudioConnection          patchCord6(playAac, 1, mixer2, 1);
AudioConnection          patchCord7(playRaw, 0, mixer2, 2);
AudioConnection          patchCord8(playFlac, 0, mixer3, 0);
AudioConnection          patchCord9(playFlac, 1, mixer3, 1);
AudioConnection          patchCord10(mixer1, dac1);
AudioConnection          patchCord11(mixer2, dac1);
AudioConnection          patchCord12(mixer3, dac1);

#define PROP_AMP_ENABLE    5
#define FLASH_CHIP_SELECT  6

float volume = 0.5f;
File root, entry;

void setup() {
  
  AudioMemory(40);
  dac1.analogReference(EXTERNAL);  //plays really loud, may get static
  
  delay(100); 

  if (!(SD.begin(BUILTIN_SDCARD))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  Serial.println("initialization done.");
  root = SD.open("/");

  //Start Amplifier
  pinMode(PROP_AMP_ENABLE , OUTPUT);
  digitalWrite(PROP_AMP_ENABLE , HIGH);
}

void playFile(const char *filename)
{
  int filetype;

  filetype = 0;
  if (strstr(filename, ".MP3") != NULL) {
      filetype = 1;
  } else if (strstr(filename, ".WAV") != NULL ) {
      filetype = 2;
  } else if (strstr(filename, ".AAC") != NULL ) {
      filetype = 3;
  } else if (strstr(filename, ".RAW") != NULL ) {
      filetype = 4;
  } else if (strstr(filename, ".FLA") != NULL ) {
      filetype = 5;
  } else
      filetype = 0;

  if (filetype > 0) {
    Serial.print("Playing file: ");
    Serial.println(filename);
    
    switch (filetype) {
      case 1 :
        mixer1.gain(0, volume);
        mixer1.gain(1, volume);
        mixer1.gain(2, 0.0f);
        mixer1.gain(3, 0.0f);
        playMp3.play(filename);
        delay(5);
        while (playMp3.isPlaying()) {
        }
        break;
      case 2 :
        mixer1.gain(0, 0.0f);
        mixer1.gain(1, 0.0f);
        mixer1.gain(2, volume);
        mixer1.gain(3, volume);
        playWav.play(filename);
        delay(5);
        while (playWav.isPlaying()) {
        }
        break;
      case 3 :
        mixer2.gain(0, volume);
        mixer2.gain(1, volume);
        mixer2.gain(2, 0.0f);
        playAac.play(filename);
        delay(5);
        while (playAac.isPlaying()) {
        }
        break;
      case 4 :
        mixer2.gain(0, 0.0f);
        mixer2.gain(1, 0.0f);
        mixer2.gain(2, volume);
        playRaw.play(filename);
        delay(5);
        while (playRaw.isPlaying()) {
        }
        break;
      case 5 :
        mixer3.gain(0, volume);
        mixer3.gain(1, volume);
        playFlac.play(filename);
        delay(5);
        while (playFlac.isPlaying()) {
        }
        break;
    }
  }
}

void loop() {
  playAll(root);
}

void playAll(File dir){
  char filename[64];
  char filnam[64];
  
   while(true) {
     entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       // rewind to begining of directory and start over
       dir.rewindDirectory();
       break;
     }
     //Serial.print(entry.name());
     if (entry.isDirectory()) {
       //Serial.println("Directory/");
       //do nothing for now
       Serial.println(entry.name());
       playAll(entry);
     } else {
       // files have sizes, directories do not
       //Serial.print("\t\t");
       //Serial.println(entry.size(), DEC);
       // files have sizes, directories do not
       strcpy(filename, dir.name());
       strcat(filename, "/");
       strcat(filename, strcpy(filnam, entry.name()));
       playFile(filename);
     }
   entry.close();
 }
}


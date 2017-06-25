// AAC + MP3 file player example
//
// Requires the prop-shield
// This example code is in the public domain.

#include <string.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <play_sd_mp3.h>
#include <play_sd_wav.h>
#include <play_sd_aac.h>

// GUItool: begin automatically generated code
//AudioPlaySdWav           playSdWav1;     //xy=154,422
AudioPlaySdMp3           playMp3; //xy=154,422
AudioPlaySdAac           playAac; //xy=154,422
AudioPlaySdWav           playWav; //xy=154,422
AudioMixer4              mixer1;         //xy=323,326
AudioMixer4              mixer2;         //xy=323,326
AudioOutputAnalog        dac0;           //xy=445,393
AudioConnection          patchCord1(playMp3, 0, mixer1, 0);
AudioConnection          patchCord2(playMp3, 1, mixer1, 1);
AudioConnection          patchCord3(playAac, 0, mixer1, 2);
AudioConnection          patchCord4(playAac, 1, mixer1, 3);
AudioConnection          patchCord5(playWav, 0, mixer2, 0);
AudioConnection          patchCord6(playWav, 1, mixer2, 1);
AudioConnection          patchCord7(mixer1, dac0);
// GUItool: end automatically generated code

#define PROP_AMP_ENABLE    5
#define FLASH_CHIP_SELECT  6

float volume = 0.5f;

void setup() {
  
  AudioMemory(10);
  dac0.analogReference(EXTERNAL);  //plays really loud, may get hissing
                              //if your speakers can't handle it.

  delay(2000); 

  // Start SerialFlash
  if (!SerialFlash.begin(FLASH_CHIP_SELECT)) {
    while (1)
    {
      Serial.println ("Cannot access SPI Flash chip");
      delay (1000);
    }
  }

  //Start Amplifier
  pinMode(PROP_AMP_ENABLE , OUTPUT);
  digitalWrite(PROP_AMP_ENABLE , HIGH);

}

void playFile(const char *filename)
{
  int filetype;
  uint32_t sz, pos;

  filetype = 0;
  if (strstr(filename, ".aac") != NULL) {
      filetype = 1;
  } else if (strstr(filename, ".mp3") != NULL ) {
      filetype = 2;
  } else if (strstr(filename, ".wav") != NULL ) {
      filetype = 3;
  } else
    filetype = 0;
  
  if (filetype > 0) {

    Serial.print("Playing file: ");
    Serial.println(filename);
    
    SerialFlashFile ff = SerialFlash.open(filename);
    sz = ff.size();
    pos = ff.getFlashAddress();

    switch (filetype) {
      case 1 :
        mixer1.gain(0, 0.0f);
        mixer1.gain(1, 0.0f);
        mixer1.gain(2, volume);
        mixer1.gain(3, volume);
        playAac.play(pos, sz);
        while (playAac.isPlaying()) {
          yield();
        }
        break;
      case 2 :
        mixer1.gain(0, volume);
        mixer1.gain(1, volume);
        mixer1.gain(2, 0.0f);
        mixer1.gain(3, 0.0f);
        playMp3.play(pos, sz);
        while (playMp3.isPlaying()) {
          yield();
        }
        break;
      case 3 :
        mixer2.gain(0, volume);
        mixer2.gain(1, volume);
        while (playWav.isPlaying()) {
          yield();
        }
        break;
    }
  }
}


void loop() {

  char filename[64];
  uint32_t filesize;

  if (!playAac.isPlaying() && !playMp3.isPlaying() && !playWav.isPlaying()) {
    if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
      playFile(filename);
    } else {
      SerialFlash.opendir();
    }
  }
}

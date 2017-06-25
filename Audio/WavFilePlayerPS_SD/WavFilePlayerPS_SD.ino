// Simple WAV file player example
//
// Three types of output may be used, by configuring the code below.
//
//   1: Digital I2S - Normally used with the audio shield:
//         http://www.pjrc.com/store/teensy3_audio.html
//
//   2: Digital S/PDIF - Connect pin 22 to a S/PDIF transmitter
//         https://www.oshpark.com/shared_projects/KcDBKHta
//
//   3: Analog DAC - Connect the DAC pin to an amplified speaker
//         http://www.pjrc.com/teensy/gui/?info=AudioOutputAnalog
//
// To configure the output type, first uncomment one of the three
// output objects.  If not using the audio shield, comment out
// the sgtl5000_1 lines in setup(), so it does not wait forever
// trying to configure the SGTL5000 codec chip.
//
// The SD card may connect to different pins, depending on the
// hardware you are using.  Uncomment or configure the SD card
// pins to match your hardware.
//
// Data files to put on your SD card can be downloaded here:
//   http://www.pjrc.com/teensy/td_libs_AudioDataFiles.html
//
// This example code is in the public domain.
// This file was tailored to work with a Teensy 3.5 with a PropShield

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

//AudioPlaySdWav           playWav1;
//AudioOutputAnalog        audioOutput;
//AudioConnection          patchCord1(playWav1, 0, audioOutput, 0);
//AudioConnection          patchCord2(playWav1, 1, audioOutput, 1);
// GUItool: begin automatically generated code
AudioPlaySdWav           playWav1;     //xy=151,266
AudioMixer4              mixer1;         //xy=323,326
AudioOutputAnalog        dac0;           //xy=445,393
AudioConnection          patchCord1(playWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playWav1, 1, mixer1, 1);
AudioConnection          patchCord3(mixer1, dac0);
// GUItool: end automatically generated code


// Use these with the Teensy 3.5 & 3.6 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used


#define PROP_AMP_ENABLE    5
#define FLASH_CHIP_SELECT  6
//#define FLASH_CHIP_SELECT 21  // Arduino 101 built-in SPI Flash

void setup() {
  Serial.begin(9600);

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(20);
  dac0.analogReference(EXTERNAL);

  //Set volume
  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);

  if (!(SD.begin(BUILTIN_SDCARD))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  
  //Start Amplifier
  pinMode(PROP_AMP_ENABLE, OUTPUT);
  digitalWrite(PROP_AMP_ENABLE, HIGH);    // Enable Amplifier

}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);

  // Start playing the file.  This sketch continues to
  // run while the file plays.
  playWav1.play(filename);

  // A brief delay for the library read WAV info
  delay(5);

  // Simply wait for the file to finish playing.
  while (playWav1.isPlaying()) {
    }
}


void loop() {
  playFile("ODD1.WAV");  // filenames are always uppercase 8.3 format
  delay(2500);
  
}


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

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <play_sd_wav.h>

// GUItool: begin automatically generated code
AudioPlaySdWav           playWav1;     //xy=151,266
AudioMixer4              mixer1;         //xy=323,326
AudioOutputAnalog        audioOutput;           //xy=445,393
AudioConnection          patchCord1(playWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playWav1, 1, mixer1, 1);
AudioConnection          patchCord3(mixer1, audioOutput);
// GUItool: end automatically generated code

#define PROP_AMP_ENABLE    5
#define FLASH_CHIP_SELECT  6
//#define FLASH_CHIP_SELECT 21  // Arduino 101 built-in SPI Flash


void setup() {
  Serial.begin(9600);

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(10);
  dac0.analogReference(EXTERNAL);  //plays really loud, may get hissing
                              //if your speakers can't handle it.

  //Set volume
  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);

  // Start SerialFlash
  if (!SerialFlash.begin(FLASH_CHIP_SELECT)) {
    while (1) {
      Serial.println ("Cannot access SPI Flash chip");
      delay (1000);
    }
  }

  //Start Amplifier - Mandatory to hear sound from the propshield
  //Don't forget to connect pin 21 to audio-in and
  //Gnd to Agnd
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
    yield();
    }
}

void loop() {
  playFile("odd1.wav");  // filenames are always uppercase 8.3 format
  delay(1500);
}


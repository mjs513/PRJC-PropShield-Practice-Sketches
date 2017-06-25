// prop shield audio from raw
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

AudioPlaySerialflashRaw playRaw1;
AudioOutputAnalog  audioOutput;
AudioConnection    patchCord1(playRaw1, audioOutput);

#define PROP_AMP_ENABLE   5
#define FLASH_CHIP_SELECT           6

void setup() {
  Serial.begin(9600);

  // wait up to 3 seconds for the Serial device to become available
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 3000))
    ;

  Serial.println ("Start prop shield wav player");

  // Enable the amplifier on the prop shield
  pinMode(PROP_AMP_ENABLE, OUTPUT);
  digitalWrite(PROP_AMP_ENABLE, HIGH);

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(8);

  // Start SerialFlash
  if (!SerialFlash.begin(FLASH_CHIP_SELECT)) {
    while (1)
    {
      Serial.println ("Cannot access SPI Flash chip");
      delay (1000);
    }
  }
}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);

  // Start playing the file.  This sketch continues to
  // run while the file plays.
  playRaw1.play(filename);

  // A brief delay for the library read WAV info
  delay(5);

  // Simply wait for the file to finish playing.
  while (playRaw1.isPlaying()) {
    // uncomment these lines if you audio shield
    // has the optional volume pot soldered
    //float vol = analogRead(15);
    //vol = vol / 1024;
    // sgtl5000_1.volume(vol);
  }
}


void loop() {
  playFile("odd.raw");  // filenames are always uppercase 8.3 format

  delay(1500);
}

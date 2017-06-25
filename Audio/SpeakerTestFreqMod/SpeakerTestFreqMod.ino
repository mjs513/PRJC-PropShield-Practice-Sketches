#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Audio.h>
#include <SerialFlash.h>

// from: https://forum.pjrc.com/threads/33328-Prop-Shield-Beta-Test?p=99236&viewfull=1#post99236
// modified by Michael Meissner

// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine1;          //xy=180,469
AudioOutputAnalog        dac1;           //xy=380,468
AudioConnection          patchCord1(sine1, dac1);
// GUItool: end automatically generated code

int freq = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Setting up");
  pinMode(5, OUTPUT);
  digitalWrite(5, 1);    // Enable Amplified.
  AudioMemory(12);
  sine1.amplitude(1.0);
  sine1.frequency(freq);
  Serial.println("Send + to increase freq, - to decrease freq, a to turn off amp, A to turn on amp, or num for freq.");
}

void loop()
{
  if (Serial.available()) {
    int ch = Serial.read();
    switch (ch)
      {
      case '+':
  freq += 100;
  sine1.frequency(freq);
  Serial.print ("New frequency ");
  Serial.println (freq);
  break;

      case '-':
  freq -= 100;
  sine1.frequency(freq);
  Serial.print ("New frequency ");
  Serial.println (freq);
  break;

      case 'a':
  digitalWrite (5, 0);
  Serial.println ("Amp off");
  break;

      case 'A':
  digitalWrite (5, 1);
  Serial.println ("Amp on");
  break;

      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
  {
    bool looping = true;
    freq = ch - '0';
    do
      {
        if (Serial.available ())
    {
      ch = Serial.read ();
      if (ch >= '0' && ch <= '9')
        freq = (freq * 10) + (ch - '0');
      else
        looping = false;
    }
      }
    while (looping);
  }

  Serial.print ("New frequency ");
  Serial.println (freq);
  sine1.frequency(freq);
  break;

      case ' ':
      case '\n':
      case '\t':
      case '\r':
  break;

      default:
  Serial.print ("Unknown character '");
  Serial.print ((char) ch);
  Serial.print ("'");
      }
  }
}

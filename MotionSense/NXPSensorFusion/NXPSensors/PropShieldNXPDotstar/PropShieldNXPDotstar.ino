
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#include <FastLED.h>

#define NUM_LEDS 10 //number of leds in strip length on one side
#define DATA_PIN 11
#define CLOCK_PIN 13

CRGB leds[NUM_LEDS];


NXPMotionSense imu;
NXPSensorFusion filter;

void setup() {
  Serial.begin(9600);
  imu.begin();
  filter.begin();
    // FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN>(leds, NUM_LEDS);
     FastLED.addLeds<DOTSTAR, RGB>(leds, NUM_LEDS);
    pinMode(7, OUTPUT); 
     digitalWrite(7, HIGH);
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  }

 //   roll = filter.getRoll();
 //   pitch = filter.getPitch();
    heading = filter.getYaw();
  
  if(heading<(170)){
  for (int x=0;x<4;x++){
       leds[x]=CRGB(0,55,255);
     FastLED.show();
    }
}
  if(heading>(190)){
  for (int x=7;x<10;x++){
       leds[x]=CRGB(200,155,0);
     FastLED.show();
    }
}
if(heading>(170)&& heading<190){
  for (int x=5;x<7;x++){
       leds[x]=CRGB(0,155,0);
     FastLED.show();
    }  
}

  for (int x=0;x<NUM_LEDS;x++){
       leds[x]=CRGB(0,0,0);
     FastLED.show();
    }
}


// Decide when to print
bool readyToPrint() {
  static unsigned long nowMillis;
  static unsigned long thenMillis;

  // If the Processing visualization sketch is sending "s"
  // then send new data each time it wants to redraw
  while (Serial.available()) {
    int val = Serial.read();
    if (val == 's') {
      thenMillis = millis();
      return true;
    }
  }
  // Otherwise, print 8 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 125) {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}

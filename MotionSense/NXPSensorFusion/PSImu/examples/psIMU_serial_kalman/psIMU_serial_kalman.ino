/**
 * FreeIMU library serial communication protocol
*/

#include "i2c_t3.h"  
#include <SPI.h>
#include <psIMU.h>
#include "CommunicationUtils.h"
#include <NXPSensorFusion.h>

psIMU myIMU = psIMU();
NXPSensorFusion filter;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// Pin definitions
int ledPin  = 13;  // Teensy led

int16_t magCount[3], accelCount[3], gyroCount[3];  // Stores the 12-bit signed value
int16_t raw_values[10];
float ypr[3],  values[10];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float ax, ay, az;       // Stores the real accel value in g's
float mx, my, mz;       // Stores the real mag value in G's
float gx, gy, gz;  // Stores the real accel value in g's
float gyrotemperature, acceltemperature;
float yaw, pitch, roll, heading;
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

int8_t gyrotempCount, acceltempCount;

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval


//The command from the PC
char cmd, cmd1;
char str[128];

void setup() {
  Serial.begin(57600);
  delay(5000);
  
   // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(1000);
 
  // Set up the interrupt pins, they're set as active high, push-pull
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  //Set calibration source, 0 = builtin, 1 = file
  myIMU.setCal(0);

  //Optional set gyro and accel configuration
  myIMU.setAccelFSR(AFS_2g);
  myIMU.setAccelODR(AODR_200HZ);
  myIMU.setAccelSensitivity(4096);
  myIMU.setGyroFSR(GFS_250DPS);
  myIMU.setGyroODR(GODR_200HZ);
  
  // Read the WHO_AM_I registers, this is a good test of communication
  // and runs calibration if connected
  if(myIMU.checkWAI() == 0){
    Serial.println("Could not connect to Prop Shield!");
    while(1) ; // Loop forever if communication doesn't happen}
  } else {
    //Calibrate Gyro
    Serial.println("Hold sensor flat and motionless for gyro calibration!");
    delay(2000);
    myIMU.calibGyro();
    Serial.println("Gyro calibration done!"); Serial.println();
   
	  if(myIMU.cal_file == 0){
  	  //Calibrate accelerometer
  	  Serial.println("Hold sensor flat and motionless for accel calibration!");
  	  delay(1000);
  	  myIMU.calibAccel();
  	  Serial.println("Accel calibration done!"); Serial.println();
  	
  	  //Calibrate Magetometer
  	  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  	  myIMU.calibMag();
  	  Serial.println("Mag Calibration done!"); Serial.println();
  	} 
  }
  
  Serial.println("Calibration Complete!");
  Serial.println("Initalizing Prop Shield IMU");
  myIMU.initPS();
  filter.begin(100); // 100 measurements per second

  Serial.println(); Serial.println("Ready to receive commands");
  delay(1000);
}

void readSerial(){
  if(Serial.available()) {
    cmd = Serial.read();
    if(cmd=='v') {
      cmd1 = '0';
      sprintf(str, "PropShield library");
      Serial.print(str);
      Serial.print('\n');
    }

    else if(cmd=='r') {
      cmd1 = '0';
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        //my3IMU.getUnfilteredRawValues(raw_values);
        myIMU.getRawValues(raw_values);
        sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8]);
        Serial.print(str);
        myIMU.getBaro();
        Serial.print(myIMU.temperature); Serial.print(",");
        Serial.print(myIMU.altimeter_setting_pressure_mb); Serial.print(",");
        Serial.print(millis()); Serial.print(",");
        Serial.println("\r\n");
     }
    }
	else if(cmd == 'w'){
		Serial.print(sizeof(int16_t));
    Serial.print("\r\n");
	}
    else if(cmd=='b') {
      cmd1 = '0';
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
		    myIMU.getRawValues(raw_values);
        writeArr(raw_values, 9, sizeof(int16_t));
        Serial.print("\r\n");
      }
    }
    else if(cmd=='p'){
      cmd1 = '0';
      //set sea level pressure
      long sea_press = Serial.parseInt();        
      myIMU.setSeaPress(sea_press/100.0);
      //Serial.println(sea_press);
    }
    else if(cmd == 'z'){
            cmd1 = 'z';
    }
    else if(cmd == 'C') { // check calibration values
      cmd1 = '0';
      Serial.print("acc offset: ");
      Serial.print(myIMU.accel_bias[0]);
      Serial.print(",");
      Serial.print(myIMU.accel_bias[1]);
      Serial.print(",");
      Serial.print(myIMU.accel_bias[2]);
      Serial.print("\n");
      
      Serial.print("magn offset: ");
      Serial.print(myIMU.mag_bias[0]);
      Serial.print(",");
      Serial.print(myIMU.mag_bias[1]);
      Serial.print(",");
      Serial.print(myIMU.mag_bias[2]);
      Serial.print("\n");
      
      Serial.print("acc scale: ");
      Serial.print(myIMU.accel_scale[0]);
      Serial.print(",");
      Serial.print(myIMU.accel_scale[1]);
      Serial.print(",");
      Serial.print(myIMU.accel_scale[2]);
      Serial.print("\n");
      
      Serial.print("magn scale: ");
      Serial.print(myIMU.mag_scale[0]);
      Serial.print(",");
      Serial.print(myIMU.mag_scale[1]);
      Serial.print(",");
      Serial.print(myIMU.mag_scale[2]);
      Serial.print("\n");
    }
  }
}

void loop() {
  readSerial();
  if(cmd1 == 'z') {

    // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
	  myIMU.getValues(values);
    ax = values[0];
    ay = values[1];
    az = values[2];
    gx = values[3];
    gy = values[4];
    gz = values[5];
    mx = values[6];
    my = values[7];
    mz = values[8];

    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
  
    // Update the Kalman filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    
   uint32_t delt_t = millis() - count;
   if (delt_t > 100) { // update LCD once per half-second independent of read rate

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll); 
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");+
    Serial.println("---------------------");
    
    digitalWrite(ledPin, !digitalRead(ledPin));
    count = millis(); 
    sumCount = 0;
    sum = 0;
   }         
  } 
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}

/**
 * FreeIMU library serial communication protocol
*/

#include "i2c_t3.h"  
#include <SPI.h>
#include <psIMU.h>
#include "CommunicationUtils.h"
#include <iCompass.h>

psIMU myIMU = psIMU();
iCompass maghead;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System) fo the
// Mahoney Gradient Descent Filter from the original paper
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

// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define twoKp 2.0f * 5.0f
#define twoKi 0.0f

// This is the free parameter for the Madgwick Quat Filter
# define betadef 0.25f


//Following lines defines Madgwicks Grad Descent Algorithm from his original paper
// Global system variables
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;   // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0;         // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

// Pin definitions
int ledPin  = 13;  // Teensy led

int16_t magCount[3], accelCount[3], gyroCount[3];  // Stores the 12-bit signed value
int16_t raw_values[10];
float ypr[3],  vals[13], value_array[20];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float gyrotemperature, acceltemperature;
float yaw, pitch, roll;
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
volatile float integralFBx,  integralFBy, integralFBz;

int instability_fix = 1;

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
  myIMU.setFileCal();

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
    digitalWrite(ledPin, HIGH);
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
    digitalWrite(ledPin, LOW); 
  }
  
  Serial.println("Calibration Complete!");
  Serial.println("Initalizing Prop Shield IMU");
  maghead = iCompass(0, 1, 0);
  myIMU.initPS();
  Serial.println(); Serial.println("Ready to receive commands");
  for(int i = 0; i < 10; i++){
	    digitalWrite(ledPin, HIGH);
      delay(250);
      digitalWrite(ledPin, LOW);
      delay(250);
  }
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
        #ifdef MPL3115
          myIMU.getBaro();
          Serial.print(myIMU.temperature); Serial.print(",");
          Serial.print(myIMU.altimeter_setting_pressure_mb); Serial.print(",");
        #endif
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
    else if(cmd == 'y'){
      cmd1 = 'y';
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
    myIMU.getValues(vals);
    //sprintf(str, "%f,%f,%f,%f,%f,%f,%f,%f,%f,", vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8]);
    //Serial.println(str);
    
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
  
    // Sensors x (y)-axis of the accelerometer/magnetometer is aligned with the x (y)-axis of the gyro on the prop shield;
    // All three sensors have the positive z-axis up.
    // The long axis of the prop shield is in the x-axis direction, which we will designate as North
    // Then x is North, -y is East, and -z is Down for a NED convention
    // Pass gyro rate as rad/s

    //MadgwickQuaternionUpdate(-vals[0], vals[1], vals[2], vals[3]*PI/180.0f, -vals[4]*PI/180.0f, -vals[5]*PI/180.0f, vals[6], -vals[7], -vals[8]);
    MahonyQuaternionUpdate(-vals[0], vals[1], vals[2], vals[3]*PI/180.0f, -vals[4]*PI/180.0f, -vals[5]*PI/180.0f, vals[6], -vals[7], -vals[8]);
    //MARGUpdateFilter(-vals[0], vals[1], vals[2], vals[3]*PI/180.0f, -vals[4]*PI/180.0f, -vals[5]*PI/180.0f, vals[6], -vals[7], -vals[8]);
    
    vals[9] = maghead.iheading(1, 0, 0, vals[0], vals[1], vals[2], vals[6], vals[7], vals[8]);

    myIMU.MotionDetect( vals );
	
    //val[10] = getEstAltitude(q, val, (1./sampleFreq));
	
   uint32_t delt_t = millis() - count;
   if (delt_t > 100) { // update LCD once per half-second independent of read rate	
    float val_array[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    val_array[7] = (vals[3] * PI/180.);	//gx
    val_array[8] = (vals[4] * PI/180.);	//gy
    val_array[9] = (vals[5] * PI/180.);	//gzz
    val_array[4] = (vals[0]);			//ax
    val_array[5] = (vals[1]);			//ay
    val_array[6] = (vals[2]);			//az
    val_array[10] = (vals[6]);			//mx
    val_array[11] = (vals[7]);			//my
    val_array[12] = (vals[8]);			//mz
    val_array[0] = q[0];
    val_array[1] = q[1];
    val_array[2] = q[2];
    val_array[3] = q[3];
    val_array[15] = 1./deltat;			//sampling frequency
    #ifdef MPL3115
      myIMU.getBaro();
      val_array[13] = myIMU.temperature;
      val_array[14] = myIMU.altimeter_setting_pressure_mb;
      val_array[17] = myIMU.altitude;		//in meters
    #else
      val_array[13] = 0.0;
      val_array[14] = 0.0;
      val_array[17] = 0.0;    //in meters
    #endif
    val_array[16] = vals[9];      //heading
    val_array[18] = vals[11];			////motion flag

    serialPrintFloatArr(val_array, 19);
    Serial.print('\n');
    
    digitalWrite(ledPin, !digitalRead(ledPin));
    count = millis(); 
    sumCount = 0;
    sum = 0;
   }         
  }

  if(cmd1 == 'y') {

    // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
    myIMU.getValues(vals);
    //sprintf(str, "%f,%f,%f,%f,%f,%f,%f,%f,%f,", vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8]);
    //Serial.println(str);
    
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
  
    // Sensors x (y)-axis of the accelerometer/magnetometer is aligned with the x (y)-axis of the gyro on the prop shield;
    // All three sensors have the positive z-axis up.
    // The long axis of the prop shield is in the x-axis direction, which we will designate as North
    // Then x is North, -y is East, and -z is Down for a NED convention
    // Pass gyro rate as rad/s

    //MadgwickQuaternionUpdate(-vals[0], vals[1], vals[2], vals[3]*PI/180.0f, -vals[4]*PI/180.0f, -vals[5]*PI/180.0f, vals[6], -vals[7], -vals[8]);
    MahonyQuaternionUpdate(-vals[0], vals[1], vals[2], vals[3]*PI/180.0f, -vals[4]*PI/180.0f, -vals[5]*PI/180.0f, vals[6], -vals[7], -vals[8]);
    //MARGUpdateFilter(-vals[0], vals[1], vals[2], vals[3]*PI/180.0f, -vals[4]*PI/180.0f, -vals[5]*PI/180.0f, vals[6], -vals[7], -vals[8]);
    
   uint32_t delt_t = millis() - count;
   if (delt_t > 100) { // data output rate
    yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
     
    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    //Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    //Serial.println("---------------------");
    
    digitalWrite(ledPin, !digitalRead(ledPin));
    count = millis(); 
    sumCount = 0;
    sum = 0;
   }         
  }
  delay(5);
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}


float invSqrt(float x) {
    if (instability_fix == 0)
    {
        union {
           float f;
           int32_t i;
        } y;

        y.f = x;
        y.i = 0x5f375a86 - (y.i >> 1);
        y.f = y.f * ( 1.5f - ( x * 0.5f * y.f * y.f ) );
        return y.f;
    }
    else if (instability_fix == 1)
    {
        /* close-to-optimal  method with low cost from
        http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
        uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
        float tmp = *(float*)&i;
        return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
    }
    else
    {
        /* optimal but expensive method: */
        return 1.0f / sqrt(x);
    }
}
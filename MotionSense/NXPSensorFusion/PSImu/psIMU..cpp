#include "psIMU.h"
#include "i2c_t3.h"  
#include <SPI.h>
#include <Arduino.h>
#include <Filter.h>

//Setup Motion Detect Averages
MovingAvarageFilter accnorm_avg(5);
MovingAvarageFilter accnorm_test_avg(7);
MovingAvarageFilter accnorm_var(7);	
MovingAvarageFilter motion_detect_ma(7);

//Setup accelerometer filter
//butter50hz2_0 mfilter_accx;
//butter50hz2_0 mfilter_accy;
//butter50hz2_0 mfilter_accz;
butter10hz0_3 mfilter_accx;
butter10hz0_3 mfilter_accy;
butter10hz0_3 mfilter_accz;

//#if HAS_MPU9150() || HAS_MPU9250()
//Set up Butterworth Filter for 9150 mag - more noisy than HMC5883L
//was butter50hz2_0, new values base on Mario Cannistrà suggestion
//he also used same filter on the gryo's which I am not using rigth now
butter100hz2_0  mfilter_mx;
butter100hz2_0  mfilter_my;
butter100hz2_0  mfilter_mz;

psIMU::psIMU(){
	// run builtin calibration
	cal_file = 0;
	type = 0;
}

void psIMU::setFileCal(){
	#undef calBuiltin
	#if defined(calBuiltin)
	  cal_file = 0;
	#else
	  cal_file = 1;
	  // get values from global variables of same name defined in calibration.h
	  #include "calibration.h"
	#endif
}

void psIMU::setAccelFSR(uint8_t range){
	accelFSR = range;
}

void psIMU::setAccelSensitivity(uint16_t range){
	accelsensitivity = range;
}

void psIMU::setAccelODR(uint8_t range){
	accelODR = range;
}

void psIMU::setGyroFSR(uint8_t range){
	gyroFSR = range;
}

void psIMU::setGyroODR(uint8_t range){
	gyroODR = range;
}

void psIMU::calibAccel(){
    getAres();                   // get accelerometer sensitivity (g/LSB) 
    FXOS8700CQReset();           // Start by resetting sensor device to default settings
	calibrateFXOS8700CQ();       // Calibrate the accelerometer
}

void psIMU::calibGyro(){
    getGres();                   // get gyro sensitivity (dps/LSB)	
    FXAS21000Reset();            // Start by resetting sensor device to default settings
    calibrateFXAS21000(gBias);   // Calculate gyro offset bias
}

void psIMU::calibMag(){
    mRes = 1.;                   // get magnetometer sensitivity (0.1 microTesla/LSB or 1 milliGauss/LSB)
	FXOS8700CQMagOffset();       // Determine magnetometer offsets-currently calculate the offsets dynamically
}

void psIMU::initPS(){
    initFXOS8700CQ();            // Initialize the accelerometer and magnetometer if communication is OK
    //accelMotionIntFXOS8700CQ();  // Configure motion interrupts
    //sleepModeFXOS8700CQ();       // Configure sleep mode
    //Serial.println("FXOS8700CQQ is online...");
    delay (1000);

    initFXAS21000();  // init the accelerometer if communication is OK
    //Serial.println("FXAS21000Q is online...");
    delay (1000);

    #ifdef MPL3115
		MPL3115A2Reset();                // Start off by resetting all registers to the default
		initRealTimeMPL3115A2();         // initialize the altimeter for realtime data acquisition if communication is OK
		MPL3115A2SampleRate(SAMPLERATE); // Set oversampling ratio
		MPL3115A2enableEventflags();     // Set data ready enable
		//Serial.println("MPL3115A2 event flags enabled...");
		delay (1000);
	#endif
	
	if(cal_file == 0){
		aBias[0] = 2*readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_X);
		aBias[1] = 2*readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Y);
		aBias[2] = 2*readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Z);
		//Serial.println("FXOS8700CQ accelerometer calibration results");
		//Serial.print("AxBias = "); Serial.print(aBias[0]); Serial.println(" mg");
		//Serial.print("AyBias = "); Serial.print(aBias[1]); Serial.println(" mg");
		//Serial.print("AzBias = "); Serial.print(aBias[2]); Serial.println(" mg");
		///delay(1000);

		mBias[0] = (int16_t)readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_LSB);
		mBias[1] = (int16_t)readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_LSB);
		mBias[2] = (int16_t)readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_LSB);
		//Serial.println("FXOS8700CQ magnetometer calibration results");
		//Serial.print("MxBias = "); Serial.print((int16_t)(((float)mBias[0])*mRes)); Serial.println(" milliGauss");
		//Serial.print("MyBias = "); Serial.print((int16_t)(((float)mBias[1])*mRes)); Serial.println(" milliGauss");
		//Serial.print("MzBias = "); Serial.print((int16_t)(((float)mBias[2])*mRes)); Serial.println(" milliGauss");
		//delay(1000);

		//Serial.println("FXAS21000Q calibration results");
		//Serial.print("GxBias = "); Serial.print(gBias[0], 2); Serial.println(" o/s");
		//Serial.print("GyBias = "); Serial.print(gBias[1], 2); Serial.println(" o/s");
		//Serial.print("GzBias = "); Serial.print(gBias[2], 2); Serial.println(" o/s");
		delay(1000);
	}
    //Serial.print("Oversampling Ratio is "); Serial.println(1<<SAMPLERATE);  
}

void psIMU::getRawValues(int16_t * raw_values){
	readAccelData(&raw_values[0]);
	readGyroData(&raw_values[3]);
	readMagData(&raw_values[6]);
	raw_values[9] = readGyroTempData();
}

void psIMU::getValues(float * values)
{
	if(cal_file == 0){
	  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
	  if(readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_DR_STATUS) & 0x08)  // When this bit set, all accel axes have new data
	  {
		readAccelData(accelCount);       // Read the x/y/z adc values
		values[0] = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
		values[1] = (float)accelCount[1]*aRes;  // also subtract averaged accelerometer biases
		values[2] = (float)accelCount[2]*aRes;  
	  }
	  
	  if(readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_DR_STATUS) & 0x08)  // When this bit set, all mag axes have new data
	  {
		readMagData(magCount);         // Read the x/y/z adc values
		values[6] = (float)magCount[0]*mRes;  // get actual milliGauss value 
		values[7] = (float)magCount[1]*mRes;   
		values[8] = (float)magCount[2]*mRes;  
	  }
		delay(10);
	  if(readByte(FXAS21000_ADDRESS, FXAS21000_DR_STATUS) & 0x08)  // When this bit set, all axes have new data
	  {
		readGyroData(gyroCount);  // Read the x/y/z adc values
		// Calculate the gyro value into actual degrees per second
		values[3] = (float)gyroCount[0]*gRes - gBias[0];  // get actual gyro value, this depends on scale being set
		values[4] = (float)gyroCount[1]*gRes - gBias[1];  
		values[5] = (float)gyroCount[2]*gRes - gBias[2];  
	  }
	} else {
	  if(readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_DR_STATUS) & 0x08)  // When this bit set, all accel axes have new data
	  {
		readAccelData(accelCount);       // Read the x/y/z adc values
		int axcnt = mfilter_accx.filter((float) accelCount[0]);
		int aycnt = mfilter_accy.filter((float) accelCount[1]);
		int azcnt = mfilter_accz.filter((float) accelCount[2]);
		
		values[0] = ((float)axcnt-accel_bias[0])/accel_scale[0];
		values[1] = ((float)aycnt-accel_bias[1])/accel_scale[1];
		values[2] = ((float)azcnt-accel_bias[2])/accel_scale[2];  
	  }

	  if(readByte(FXAS21000_ADDRESS, FXAS21000_DR_STATUS) & 0x08)  // When this bit set, all axes have new data
	  {
		readGyroData(gyroCount);  // Read the x/y/z adc values
		
		
		// Calculate the gyro value into actual degrees per second
		values[3] = (float)gyroCount[0]*gRes - gBias[0];  // get actual gyro value, this depends on scale being set
		values[4] = (float)gyroCount[1]*gRes - gBias[1];  
		values[5] = (float)gyroCount[2]*gRes - gBias[2];  
	  }
	  
	  if(readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_DR_STATUS) & 0x08)  // When this bit set, all mag axes have new data
	  {
		readMagData(magCount);         // Read the x/y/z adc values
		int mxcnt = mfilter_mx.filter((float) magCount[0]);
		int mycnt = mfilter_my.filter((float) magCount[1]);
		int mzcnt = mfilter_mz.filter((float) magCount[2]); 
		
		values[6] = ((float)mxcnt-mag_bias[0])/mag_scale[0];  // get actual milliGauss value 
		values[7] = ((float)mycnt-mag_bias[1])/mag_scale[1];   
		values[8] = ((float)mzcnt-mag_bias[2])/mag_scale[2];  
	  }
	  //delay(10);
	}
	
  //Serial.print("Accel-cnt:"); Serial.print(accelCount[0]); Serial.print(", ");
  //Serial.print(accelCount[2]); Serial.print(", ");Serial.println(accelCount[2]); 
  //Serial.print("Gyro-cnt:"); Serial.print(gyroCount[0]); Serial.print(", ");
  //Serial.print(gyroCount[2]); Serial.print(", ");Serial.println(gyroCount[2]);
  //Serial.print("Mag-cnt:"); Serial.print(magCount[0]); Serial.print(", ");
  //Serial.print(magCount[2]); Serial.print(", ");Serial.println(magCount[2]);   
  //Serial.println();
}

void psIMU::getBaro(){
    MPL3115A2ActiveAltimeterMode(); 
    MPL3115A2readAltitude();  // Read the altitude

    MPL3115A2ActiveBarometerMode(); 
    MPL3115A2readPressure();  // Read the pressure
    
    const int station_elevation_m = def_sea_press*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = pressure/100; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;
}

void psIMU::getYawPitchRoll(float * ypr){
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    ypr[0]   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    ypr[1] = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
    ypr[2]  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    ypr[1] *= 180.0f / PI;
    ypr[0]   *= 180.0f / PI; 
    ypr[0]   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(ypr[0] < 0) ypr[0]   += 360.0f; // Ensure yaw stays between 0 and 360
    ypr[2]  *= 180.0f / PI;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////Useful functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void psIMU::getAres() {
  switch (accelFSR)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 gs (00), 4 gs (01), 8 gs (10). 
    case AFS_2g:
          aRes = 2.0/8192.0;
          break;
    case AFS_4g:
          aRes = 4.0/8192.0;
          break;
    case AFS_8g:
          aRes = 8.0/8192.0;
          break;
  }
}

void psIMU::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t) (((rawData[0] << 8) | rawData[1])) >> 2; // 14-bit signed integer
  destination[1] = (int16_t) (((rawData[2] << 8) | rawData[3])) >> 2;
  destination[2] = (int16_t) (((rawData[4] << 8) | rawData[5])) >> 2;
}

void psIMU::readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t) (((rawData[0] << 8) | rawData[1])); // 16-bit signed integer
  destination[1] = (int16_t) (((rawData[2] << 8) | rawData[3]));
  destination[2] = (int16_t) (((rawData[4] << 8) | rawData[5]));
}


int8_t psIMU::readAccelTempData()
{
  return (int8_t) readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_TEMP);  // Read the 8-bit 2's complement data register 
}
  

void psIMU::calibrateFXOS8700CQ()
{
  uint16_t ii, fcount;
  int16_t temp[3];

  //Serial.println("Hold sensor flat and motionless for accel calibration!");
  
  // Clear all interrupts by reading the data output and F_STATUS registers
  readAccelData(temp);
  readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_STATUS);
  
  FXOS8700CQStandby();  // Must be in standby to change registers

  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, 0x03 << 3); // select 100 Hz ODR
  fcount = 400;                                        // sample for 4 seconds
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, accelFSR);   // select 2 g full scale

  FXOS8700CQActive();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
	  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
	  temp[0] = (int16_t) (((rawData[0] << 8) | rawData[1])) >> 2;
	  temp[1] = (int16_t) (((rawData[2] << 8) | rawData[3])) >> 2;
	  temp[2] = (int16_t) (((rawData[4] << 8) | rawData[5])) >> 2;
	  
	  accel_bias[0] += (int32_t) temp[0];
	  accel_bias[1] += (int32_t) temp[1];
	  accel_bias[2] += (int32_t) temp[2];
	  
	  delay(10);  // wait for a new data reading (100 Hz)
  }
 
  accel_bias[0] /= (int32_t) fcount; // get average values
  accel_bias[1] /= (int32_t) fcount;
  accel_bias[2] /= (int32_t) fcount;
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  rawData[0] = (-accel_bias[0]/8) & 0xFF; // get average values
  rawData[1] = (-accel_bias[1]/8) & 0xFF; // get average values
  rawData[2] = (-accel_bias[2]/8) & 0xFF; // get average values
  
  FXOS8700CQStandby();  // Must be in standby to change registers
  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_X, rawData[0]); // X-axis compensation  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Y, rawData[1]); // Y-axis compensation  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_OFF_Z, rawData[2]); // z-axis compensation 

  FXOS8700CQActive();  // Set to active to start reading

  //Serial.println("Accel calibration done!");

}
  
  
  // Set up sensor software reset
void psIMU::FXOS8700CQMagOffset() 
{
  uint16_t ii = 0, sample_count = 0;
  float dest1[3] = {0, 0, 0}, dest2[3] = {0, 0, 0};

  //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  FXOS8700CQStandby();  // Must be in standby to change registers
   
  // Clear all interrupts by reading the data output and F_STATUS registers
  readMagData(mag_temp);
  readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_STATUS);
  // Configure the magnetometer
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Enable auto-calibration, set oversampling, enable hybrid mode 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03); // Disable auto-calibration, set oversampling, enable hybrid mode 
 
  FXOS8700CQActive();  // Set to active to start collecting data
   
   sample_count = 512;
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(10);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    //Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    //Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    //Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;   
    dest1[2] = (float) mag_bias[2]*mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
    
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_MSB, ((mag_bias[0] << 1) & 0xFF00) >> 8); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_LSB, ((mag_bias[0] << 1) & 0x00FF)); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_MSB, ((mag_bias[1] << 1) & 0xFF00) >> 8); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_LSB, ((mag_bias[1] << 1) & 0x00FF)); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_MSB, ((mag_bias[2] << 1) & 0xFF00) >> 8); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_LSB, ((mag_bias[2] << 1) & 0x00FF)); 
  //Serial.println("Mag Calibration done!");

  FXOS8700CQActive();  // Set to active to start reading
}


void psIMU::sleepModeFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers

  // These settings have to do with setting up the sleep mode
  // Set Auto-WAKE sample frequency when the device is in sleep mode
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_ASLP_COUNT, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) & ~(0x04)); // clear bit 2
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) & ~(0x7C)); // clear bits 2, 3, 4, 5, and 6
     // select wake on transient, orientation change, pulse, freefall/motion detect, or acceleration vector magnitude
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) |  (0x7C)); 
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) & ~(0x80)); // clear bit 7
     writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
 
     FXOS8700CQActive();  // Set to active to start reading
}
  
  
// Set up sensor software reset
void psIMU::FXOS8700CQReset() 
{
	writeByte(FXOS8700CQ_ADDRESS,   FXOS8700CQ_CTRL_REG2, 0x40); // set reset bit to 1 to assert accel software reset to zero at end of boot process
	writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG2, 0x40); // set reset bit to 1 to assert mag software reset to zero at end of boot process
}


// Initialize the FXOS8700CQ registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXOS8700CQQ
// Feel free to modify any values, these are settings that work well for me.
void psIMU::initFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers

    // Configure the accelerometer
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, accelFSR);  // Choose the full scale range to 2, 4, or 8 g.
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) & ~(0x38)); // Clear the 3 data rate bits 5:3
    if (accelODR <= 7) writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1) | (accelODR << 3));      
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) & ~(0x03)); // clear bits 0 and 1
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG2) |  (0x02)); // select normal(00) or high resolution (10) mode
    
    // Configure the magnetometer
//    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Enable auto-calibration, set oversampling, enable hybrid mode 
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03); // Disable auto-calibration, set oversampling, enable hybrid mode 
    
    // Configure interrupts 1 and 2
    //writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
    //writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts    
    //writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
    //writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled  
    //writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG3, 0x00);	// Push-pull, active low interrupt
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG4, 0x01);	// Enable DRDY interrupt
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG5, 0x01);	// DRDY interrupt routed to INT1

    FXOS8700CQActive();  // Set to active to start reading
}


void psIMU::accelMotionIntFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers
  // This is adapted from Jim Lindblom's sketch originally found on http://www.sparkfun.com
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_CFG, 0x40);         // 1. Enable P/L
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_BF_ZCOMP, 0x04);    // 2. 29deg z-lock 
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_P_L_THS_REG, 0x84);    // 3. 45deg thresh, 14deg hyst 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PL_COUNT, 0x50);       // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_CFG, 0x58); // Set free fall OR motion flag on x and y axes
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set x/y/z/common threshold to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_X_MSB, 0x80); // Set x-axis threshold individually to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_X_LSB, 0x04);  
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Y_MSB, 0x80); // Set y-axis threshold individually to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Y_LSB, 0x04);  
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Z_MSB, 0x80); // Set z-axis threshold individually to 1.25 g
//  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_THS_Z_LSB, 0x14);  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_FFMT_COUNT, 0x08); // Set debounce to 0.08 s at 100 Hz

  // Use user-specified reference value before and after trigger event, enable accelerometer magnitude function  
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_CFG, 0x78); 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_THS_MSB, 0x80 | 0x0F); // Clear debounce counter when event falls below threshold; bit 4:0 MSB THS 
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_THS_LSB, 0xFF);        // Specify threshold at 0.3 g = 1228 * 0.244 g at FSR of +/-2 g
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_CNT, 0x01);            // Set debounce time to 80 x 1.25 ms (in high res mode) = 100 ms at 400 Hz (200 Hz hybrid mode)
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITX_MSB, 0x00);      // set x-axis vector magnitude reference at 0 g
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITX_LSB, 0x00);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITY_MSB, 0x00);      // set y-axis vector magnitude reference at 0 g
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITY_LSB, 0x00);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITZ_MSB, 0x10);      // set z-axis vector magnitude reference at +1 g = 4096 at +/- 2 g FSR
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_A_VECM_INITZ_LSB, 0x00);
  
  FXOS8700CQActive();  // Set to active to start reading
}


// Sets the FXOS8700CQ to standby mode.
// It must be in standby to change most register settings
void psIMU::FXOS8700CQStandby()
{
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, c & ~(0x01));
}


// Sets the FXOS8700CQ to active mode.
// Needs to be in this mode to output data
void psIMU::FXOS8700CQActive()
{
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, c | 0x01);
}

void psIMU::getGres() {
  switch (gyroFSR)
  {
   // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (11), 500 DPS (10), 1000 DPS (01), and 2000 DPS  (00). 
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;           
    case GFS_250DPS:
          gRes = 250.0/32768.0;
  }
}

void psIMU::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXAS21000_ADDRESS, FXAS21000_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t) (((rawData[0] << 8) | rawData[1])); // signed 16-bit integers
  destination[1] = (int16_t) (((rawData[2] << 8) | rawData[3]));
  destination[2] = (int16_t) (((rawData[4] << 8) | rawData[5]));
}

int8_t psIMU::readGyroTempData()
{
  return (int8_t) readByte(FXAS21000_ADDRESS, FXAS21000_TEMP);  // Read the 8-bit 2's complement data register 
}

 
void psIMU::calibrateFXAS21000(float * gBias)
{
  int32_t gyro_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];

  //  Serial.println("Hold sensor flat and motionless for gyro calibration!");
  
  // Clear all interrupts by reading the data output and STATUS registers
  readGyroData(temp);
  readByte(FXAS21000_ADDRESS, FXAS21000_STATUS);
  
  FXAS21000Standby();  // Must be in standby to change registers

  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, 0x04);   // select 50 Hz ODR
  fcount = 400;                                     // sample for 8 second
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG0, 0x03);   // select 250 deg/s full scale
  uint16_t gyrosensitivity = 131;                   // 131.072 LSB/deg/s

  FXAS21000Active();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(FXAS21000_ADDRESS, FXAS21000_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = (int16_t) (((rawData[0] << 8) | rawData[1]));
  temp[1] = (int16_t) (((rawData[2] << 8) | rawData[3]));
  temp[2] = (int16_t) (((rawData[4] << 8) | rawData[5]));
  
  gyro_bias[0] += (int32_t) temp[0];
  gyro_bias[1] += (int32_t) temp[1];
  gyro_bias[2] += (int32_t) temp[2];
  
  delay(25); // wait for next data sample at 50 Hz rate
  }
 
  gyro_bias[0] /= (int32_t) fcount; // get average values
  gyro_bias[1] /= (int32_t) fcount;
  gyro_bias[2] /= (int32_t) fcount;
  
  gBias[0] = (float)gyro_bias[0]/(float) gyrosensitivity; // get average values
  gBias[1] = (float)gyro_bias[1]/(float) gyrosensitivity; // get average values
  gBias[2] = (float)gyro_bias[2]/(float) gyrosensitivity; // get average values

  //Serial.println("Gyro calibration done!");

  FXAS21000Ready();  // Set to ready
}
  
  
// Set up sensor software reset
void psIMU::FXAS21000Reset() 
{
	writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, 0x20); // set reset bit to 1 to assert software reset to zero at end of boot process
	delay(100);
	while(!(readByte(FXAS21000_ADDRESS, FXAS21000_INT_SRC_FLAG) & 0x08))  { // wait for boot end flag to be set
	}
}



// Initialize the FXAS21000 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXAS21000Q
// Feel free to modify any values, these are settings that work well for me.
void psIMU::initFXAS21000()
{
  FXAS21000Standby();  // Must be in standby to change registers

  // Set up the full scale range to 200, 400, 800, or 1600 deg/s.
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG0, gyroFSR);      // write FSR

  // Setup the 3 data rate bits, 4:2
  if(gyroODR < 8) { // data rate can only be 0 to 7
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, gyroODR << 2); 
  }

  // Disable FIFO, route FIFO and rate threshold interrupts to INT2, enable data ready interrupt, route to INT1
  // Active HIGH, push-pull output driver on interrupts
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG2,  0x0E);
  
  // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_CFG, 0x07);         // enable rate threshold detection on all axes
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_COUNT, 0x04);       // set to 4 (can set up to 255)
        
  FXAS21000Active();  // Set to active to start reading
}


// Sets the FXAS21000 to standby mode.
// It must be in standby to change most register settings
void psIMU::FXAS21000Standby()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void psIMU::FXAS21000Ready()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c |   0x01);   // Set bit 0 to 1, ready mode; no data acquisition yet
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void psIMU::FXAS21000Active()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
 writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
 writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c |   0x02);   // Set bit 1 to 1, active mode; data acquisition enabled
}

void psIMU::MotionDetect(float * values) {
	
	float gyro[3];
	float accnorm;
	int accnorm_test, accnorm_var_test, omegax, omegay, omegaz, omega_test, motionDetect;
	
	gyro[0] = values[3] * PI/180;
	gyro[1] = values[4] * PI/180;
	gyro[2] = values[5] * PI/180;
	
    /*###################################################################
    #
    #   acceleration squared euclidean norm analysis
	#   
	#   some test values previously used:
	#       if((accnorm >=0.96) && (accnorm <= 0.99)){
	#										<= 0.995
    #
    ################################################################### */
    accnorm = (values[0]*values[0]+values[1]*values[1]+values[2]*values[2]);
    if((accnorm >=0.94) && (accnorm <= 1.04)){  
        accnorm_test = 0;
    } else {
        accnorm_test = 1; }
    
	/** take average of 5 to 10 points  **/
    float accnormavg = accnorm_avg.process(accnorm);
    float accnormtestavg = accnorm_test_avg.process(accnorm_test);

    /*####################################################################
    #
    #   squared norm analysis to determine suddenly changes signal
    #
    ##################################################################### */
    //accnorm_var.process(sq(accnorm-accnorm_avg.getAvg()));
	// was 0.0005
    if(accnorm_var.process(sq(accnorm-accnormavg)) < 0.0005) {
        accnorm_var_test = 0;
    } else {
        accnorm_var_test = 1; }

    /*###################################################################
    #
    #   angular rate analysis in order to disregard linear acceleration
    #
	#   other test values used: 0, 0.00215, 0.00215
    ################################################################### */
    if ((gyro[0] >=-0.015) && (gyro[0] <= 0.015)) {
        omegax = 0;
    } else {
        omegax = 1; }
        
    if((gyro[1] >= -0.015) && (gyro[1] <= 0.015)) {
        omegay = 0;
    } else {
        omegay = 1; }
        
    if((gyro[2] >= -0.015) && (gyro[2] <= 0.015)) {
        omegaz = 0;
    } else {
        omegaz = 1; }
        
    if ((omegax+omegay+omegaz) > 0) {
        omega_test = 1;
    } else {
        omega_test = 0; }

    /* 
	###################################################################
    #
    # combined movement detector
    #
    #################################################################### 
	*/
    if ((accnormtestavg + omega_test + accnorm_var_test) > 0) {
        motionDetect = 1;
    } else {
        motionDetect = 0; }

    /* 
	################################################################## 
	*/   
    
    //motion_detect_ma.process(motionDetect);
    
    if(motion_detect_ma.process(motionDetect) > 0.5) {
		values[11] = 1.0f;
    } else {
		values[11] = 0.0f;
	}
}


//===================================================
//MPL3115A2 Sensor Calls
//===================================================
void psIMU::MPL3115A2readAltitude() // Get altitude in meters and temperature in centigrade
{
  uint8_t rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers 

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
// while ((readByte(MPL3115A2_ADDRESS, MPL3115A2_STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
  
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Altitude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
  uint8_t msbA = rawData[0];
  uint8_t csbA = rawData[1];
  uint8_t lsbA = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4];

 // Calculate altitude 
  int16_t altitude_whole = ((int16_t)msbA << 8 | (int16_t)csbA ) ; // Construct signed 16-bit whole number altitude
 
  lsbA &= 0xF0; // Keep only bits 5 - 7, the fractional altitude
  lsbA >>= 4; // Shift to get the fractional altitude
  float altitude_frac = (float) lsbA/16.0; // Convert to fractional altitude in meters

  altitude = (float) (altitude_whole) + altitude_frac; // Combine whole and fractional parts to get entire pressure in Pascal

 // Calculate temperature 
  int16_t temperature_whole = ((int16_t)msbT << 8 | lsbT ) ; // Construct signed 16-bit whole number temperature
  temperature_whole >>= 8;
 
  lsbT &= 0xF0; // Keep only bits 5 - 7, the fractional temperature
  lsbT >>= 4; // Shift to get the fractional temperature
  float temperature_frac = (float) lsbT/16.0; // Convert to fractional temperature in Centigrade

  temperature = (float) (temperature_whole) + temperature_frac; // Combine whole and fractional parts to get entire temperature in Centigrade
}

void psIMU::MPL3115A2readPressure()
{
  uint8_t  rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 //while ((readByte(MPL3115A2_ADDRESS, MPL3115A2_STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
 
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Pressure bytes
  uint8_t msbP = rawData[0];
  uint8_t csbP = rawData[1];
  uint8_t lsbP = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4]; 
 
// Calculate pressure 
   int32_t pressure_whole =   ((int32_t)msbP << 16 |  (int32_t)csbP << 8 |  (int32_t)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

 // Calculate temperature 
  int16_t temperature_whole = ((int16_t)msbT << 8 | lsbT ) ; // Construct signed 16-bit whole number temperature
  temperature_whole >>= 8;

  lsbT &= 0xF0; // Keep only bits 5 - 7, the fractional temperature
  lsbT >>= 4; // Shift to get the fractional temperature
  float temperature_frac = (float) lsbT/16.0; // Convert to fractional temperature in Centigrade

  temperature = (float) (temperature_whole) + temperature_frac; // Combine whole and fractional parts to get entire temperature in Centigrade
}

/**
 * Sets sea level pressure
 * 
*/
void psIMU::setSeaPress(float sea_press_inp) {

	def_sea_press = sea_press_inp;
}

/*
=====================================================================================================
Define functions according to 
"Data Manipulation and Basic Settings of the MPL3115A2 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void psIMU::MPL3115A2toggleOneShot()
{
    MPL3115A2Active();  // Set to active to start reading
    uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
    c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void psIMU::MPL3115A2SampleRate(uint8_t samplerate)
{
  MPL3115A2Standby();  // Must be in standby to change registers

  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
  if(samplerate < 8) { // OSR between 1 and 7
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
  }
  
  MPL3115A2Active();  // Set to active to start reading
 }
 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 registers for FIFO mode
void psIMU::initFIFOMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x40);  // enable FIFO
  
  //  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // Clear FIFO mode
// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
//  writeByte(MPL3115A2_ADDRESS, F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 for realtime data collection 
void psIMU::initRealTimeMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x80);  
  
  //  Configure INT 1 for data ready, all other interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // disable FIFO mode
  
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step
void psIMU::MPL3115A2TimeStep(uint8_t ST_Value)
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2); // Read contents of register CTRL_REG2
 if (ST_Value <= 0xF) {
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
 }
 
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
 // Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
 // Bit 1 is event flag on new Pressure/Altitude data
 // Bit 0 is event flag on new Temperature data
void psIMU::MPL3115A2enableEventflags()
{
  MPL3115A2Standby();  // Must be in standby to change registers
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void psIMU::MPL3115A2ActiveAltimeterMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void psIMU::MPL3115A2ActiveBarometerMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115A2.
// It must be in standby to change most register settings
void psIMU::MPL3115A2Reset()
{
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, 0x04); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to standby mode.
// It must be in standby to change most register settings
void psIMU::MPL3115A2Standby()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to active mode.
// Needs to be in this mode to output data
void psIMU::MPL3115A2Active()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}

// simple function to scan for I2C devices on the bus
void psIMU::I2Cscan() 
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

uint8_t psIMU::checkWAI(){
  I2Cscan();
  byte c = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_WHO_AM_I);  // Read accel/mag WHO_AM_I register
  Serial.print("FXOS8700CQ I am 0x"); Serial.print(c, HEX); Serial.println(" I should be 0xC7");
  byte d = readByte(FXAS21000_ADDRESS, FXAS21000_WHO_AM_I);  // Read gyro WHO_AM_I register
  Serial.print("FXOS8700CQ I am 0x"); Serial.print(d, HEX); Serial.println(" I should be 0xD7");
  #ifdef MPL3115
	byte e = readByte(MPL3115A2_ADDRESS, MPL3115A2_WHO_AM_I);  // Read altimeter WHO_AM_I register
	Serial.print("MPL3115A2  I am 0x"); Serial.print(e, HEX); Serial.println(" I should be 0xC4");
  #endif
  
  #ifdef MPL3115
	if (c == 0xC7 && d == 0xD7 && e == 0xC4) // WHO_AM_I should always be 0xC7, 0xD1, and 0xC4
  #else
	if (c == 0xC7 && d == 0xD7 )
  #endif
  {	  
	return 1;
  } else
  {
	return 0 ; 
    //Serial.println("Could not connect to Prop Shield!");
    //Serial.print("0x"); Serial.println(c, HEX); 
    //Serial.print("0x"); Serial.println(d, HEX); 
    //Serial.print("0x"); Serial.println(e, HEX);
    //while(1) ; // Loop forever if communication doesn't happen
  }
}

// I2C read/write functions 

void psIMU::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t psIMU::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void psIMU::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
//  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(I2C_NOSTOP);             // Send the Tx buffer, but send a restart to keep connection alive  uint8_t i = 0;
        uint8_t i = 0;
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer

}

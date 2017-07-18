#ifndef psIMU_h
#define psIMU_h

#include <inttypes.h>
#include <math.h>

//uncomment for MPL3115 pressure sensor
#define MPL3115

// Define registers per FXOS8700CQ, Document Number: FXOS8700CQ 
// http://cache.freescale.com/files/sensors/doc/data_sheet/FXOS8700CQ.pdf
// Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
// Freescale Semiconductor Data Sheet
#define FXOS8700CQ_STATUS           0x00
#define FXOS8700CQ_DR_STATUS        0x00
#define FXOS8700CQ_F_STATUS         0x00
#define FXOS8700CQ_OUT_X_MSB        0x01    
#define FXOS8700CQ_OUT_X_LSB        0x02
#define FXOS8700CQ_OUT_Y_MSB        0x03
#define FXOS8700CQ_OUT_Y_LSB        0x04
#define FXOS8700CQ_OUT_Z_MSB        0x05
#define FXOS8700CQ_OUT_Z_LSB        0x06
#define FXOS8700CQ_F_SETUP          0x09
#define FXOS8700CQ_TRIG_CFG         0x0A
#define FXOS8700CQ_SYSMOD           0x0B
#define FXOS8700CQ_INT_SOURCE       0x0C
#define FXOS8700CQ_WHO_AM_I         0x0D   
#define FXOS8700CQ_XYZ_DATA_CFG     0x0E
#define FXOS8700CQ_HP_FILTER_CUTOFF 0x0F
#define FXOS8700CQ_PL_STATUS        0x10
#define FXOS8700CQ_PL_CFG           0x11
#define FXOS8700CQ_PL_COUNT         0x12
#define FXOS8700CQ_PL_BF_ZCOMP      0x13
#define FXOS8700CQ_P_L_THS_REG      0x14
#define FXOS8700CQ_A_FFMT_CFG       0x15
#define FXOS8700CQ_A_FFMT_SRC       0x16
#define FXOS8700CQ_A_FFMT_THS       0x17
#define FXOS8700CQ_A_FFMT_COUNT     0x18
#define FXOS8700CQ_TRANSIENT_CFG    0x1D
#define FXOS8700CQ_TRANSIENT_SRC    0x1E
#define FXOS8700CQ_TRANSIENT_THS    0x1F
#define FXOS8700CQ_TRANSIENT_COUNT  0x20
#define FXOS8700CQ_PULSE_CFG        0x21
#define FXOS8700CQ_PULSE_SRC        0x22
#define FXOS8700CQ_PULSE_THSX       0x23
#define FXOS8700CQ_PULSE_THSY       0x24
#define FXOS8700CQ_PULSE_THSZ       0x25
#define FXOS8700CQ_PULSE_TMLT       0x26
#define FXOS8700CQ_PULSE_LTCY       0x27
#define FXOS8700CQ_PULSE_WIND       0x28
#define FXOS8700CQ_ASLP_COUNT       0x29
#define FXOS8700CQ_CTRL_REG1        0x2A
#define FXOS8700CQ_CTRL_REG2        0x2B
#define FXOS8700CQ_CTRL_REG3        0x2C
#define FXOS8700CQ_CTRL_REG4        0x2D
#define FXOS8700CQ_CTRL_REG5        0x2E
#define FXOS8700CQ_A_OFF_X          0x2F
#define FXOS8700CQ_A_OFF_Y          0x30
#define FXOS8700CQ_A_OFF_Z          0x31
#define FXOS8700CQ_M_DR_STATUS      0x32
#define FXOS8700CQ_M_OUT_X_MSB      0x33    
#define FXOS8700CQ_M_OUT_X_LSB      0x34
#define FXOS8700CQ_M_OUT_Y_MSB      0x35
#define FXOS8700CQ_M_OUT_Y_LSB      0x36
#define FXOS8700CQ_M_OUT_Z_MSB      0x37
#define FXOS8700CQ_M_OUT_Z_LSB      0x38
#define FXOS8700CQ_CMP_OUT_X_MSB    0x39    
#define FXOS8700CQ_CMP_OUT_X_LSB    0x3A
#define FXOS8700CQ_CMP_OUT_Y_MSB    0x3B
#define FXOS8700CQ_CMP_OUT_Y_LSB    0x3C
#define FXOS8700CQ_CMP_OUT_Z_MSB    0x3D
#define FXOS8700CQ_CMP_OUT_Z_LSB    0x3E
#define FXOS8700CQ_M_OFF_X_MSB      0x3F    
#define FXOS8700CQ_M_OFF_X_LSB      0x40
#define FXOS8700CQ_M_OFF_Y_MSB      0x41
#define FXOS8700CQ_M_OFF_Y_LSB      0x42
#define FXOS8700CQ_M_OFF_Z_MSB      0x43
#define FXOS8700CQ_M_OFF_Z_LSB      0x44
#define FXOS8700CQ_MAX_X_MSB        0x45   
#define FXOS8700CQ_MAX_X_LSB        0x46
#define FXOS8700CQ_MAX_Y_MSB        0x47
#define FXOS8700CQ_MAX_Y_LSB        0x48
#define FXOS8700CQ_MAX_Z_MSB        0x49
#define FXOS8700CQ_MAX_Z_LSB        0x4A
#define FXOS8700CQ_MIN_X_MSB        0x4B   
#define FXOS8700CQ_MIN_X_LSB        0x4C
#define FXOS8700CQ_MIN_Y_MSB        0x4D
#define FXOS8700CQ_MIN_Y_LSB        0x4E
#define FXOS8700CQ_MIN_Z_MSB        0x4F
#define FXOS8700CQ_MIN_Z_LSB        0x50
#define FXOS8700CQ_TEMP             0x51
#define FXOS8700CQ_M_THS_CFG        0x52
#define FXOS8700CQ_M_THS_SRC        0x53
#define FXOS8700CQ_M_THS_X_MSB      0x54   
#define FXOS8700CQ_M_THS_X_LSB      0x55
#define FXOS8700CQ_M_THS_Y_MSB      0x56
#define FXOS8700CQ_M_THS_Y_LSB      0x57
#define FXOS8700CQ_M_THS_Z_MSB      0x58
#define FXOS8700CQ_M_THS_Z_LSB      0x59
#define FXOS8700CQ_M_THS_COUNT      0x5A
#define FXOS8700CQ_M_CTRL_REG1      0x5B
#define FXOS8700CQ_M_CTRL_REG2      0x5C
#define FXOS8700CQ_M_CTRL_REG3      0x5D
#define FXOS8700CQ_M_INT_SRC        0x5E
#define FXOS8700CQ_A_VECM_CFG       0x5F
#define FXOS8700CQ_A_VECM_THS_MSB   0x60
#define FXOS8700CQ_A_VECM_THS_LSB   0x61
#define FXOS8700CQ_A_VECM_CNT       0x62
#define FXOS8700CQ_A_VECM_INITX_MSB 0x63   
#define FXOS8700CQ_A_VECM_INITX_LSB 0x64
#define FXOS8700CQ_A_VECM_INITY_MSB 0x65
#define FXOS8700CQ_A_VECM_INITY_LSB 0x66
#define FXOS8700CQ_A_VECM_INITZ_MSB 0x67
#define FXOS8700CQ_A_VECM_INITZ_LSB 0x68
#define FXOS8700CQ_M_VECM_CFG       0x69
#define FXOS8700CQ_M_VECM_THS_MSB   0x6A
#define FXOS8700CQ_M_VECM_THS_LSB   0x6B
#define FXOS8700CQ_M_VECM_CNT       0x6C
#define FXOS8700CQ_M_VECM_INITX_MSB 0x6D   
#define FXOS8700CQ_M_VECM_INITX_LSB 0x6E
#define FXOS8700CQ_M_VECM_INITY_MSB 0x6F
#define FXOS8700CQ_M_VECM_INITY_LSB 0x70
#define FXOS8700CQ_M_VECM_INITZ_MSB 0x71
#define FXOS8700CQ_M_VECM_INITZ_LSB 0x72
#define FXOS8700CQ_A_FFMT_THS_X_MSB 0x73
#define FXOS8700CQ_A_FFMT_THS_X_LSB 0x74
#define FXOS8700CQ_A_FFMT_THS_Y_MSB 0x75
#define FXOS8700CQ_A_FFMT_THS_Y_LSB 0x76
#define FXOS8700CQ_A_FFMT_THS_Z_MSB 0x77
#define FXOS8700CQ_A_FFMT_THS_Z_LSB 0x78

// Define registers per Freescale Semiconductor, Inc.
// FXAS21000 Data Sheet: Advance Information Rev 1.1, 10/2013 3-Axis, 14-bit Digital MEMS Gyroscope
// http://cache.freescale.com/files/sensors/doc/data_sheet/FXAS21000.pdf
// Freescale Semiconductor Data Sheet
#define FXAS21000_STATUS           0x00
#define FXAS21000_OUT_X_MSB        0x01    
#define FXAS21000_OUT_X_LSB        0x02
#define FXAS21000_OUT_Y_MSB        0x03
#define FXAS21000_OUT_Y_LSB        0x04
#define FXAS21000_OUT_Z_MSB        0x05
#define FXAS21000_OUT_Z_LSB        0x06
#define FXAS21000_DR_STATUS        0x07
#define FXAS21000_F_STATUS         0x08
#define FXAS21000_F_SETUP          0x09
#define FXAS21000_F_EVENT          0x0A
#define FXAS21000_INT_SRC_FLAG     0x0B
#define FXAS21000_WHO_AM_I         0x0C   // Should return 0xD7
#define FXAS21000_CTRL_REG0        0x0D
#define FXAS21000_RT_CFG           0x0E   
#define FXAS21000_RT_SRC           0x0F
#define FXAS21000_RT_THS           0x10
#define FXAS21000_RT_COUNT         0x11
#define FXAS21000_TEMP             0x12
#define FXAS21000_CTRL_REG1        0x13
#define FXAS21000_CTRL_REG2        0x14
#define FXAS21000_CTRL_REG3        0x15


// Freescale Semiconductor MPL3115A2 Precision I2C altimeter 
// http://www.nxp.com/files/sensors/doc/data_sheet/MPL3115A2.pdf
// Register defines courtesy A. Weiss and Nathan Seidle, SparkFun Electronics
#define MPL3115A2_STATUS           0x00
#define MPL3115A2_OUT_P_MSB        0x01
#define MPL3115A2_OUT_P_CSB        0x02
#define MPL3115A2_OUT_P_LSB        0x03
#define MPL3115A2_OUT_T_MSB        0x04
#define MPL3115A2_OUT_T_LSB        0x05
#define MPL3115A2_DR_STATUS        0x06
#define MPL3115A2_OUT_P_DELTA_MSB  0x07
#define MPL3115A2_OUT_P_DELTA_CSB  0x08
#define MPL3115A2_OUT_P_DELTA_LSB  0x09
#define MPL3115A2_OUT_T_DELTA_MSB  0x0A
#define MPL3115A2_OUT_T_DELTA_LSB  0x0B
#define MPL3115A2_WHO_AM_I         0x0C
#define MPL3115A2_F_STATUS         0x0D
#define MPL3115A2_F_DATA           0x0E
#define MPL3115A2_F_SETUP          0x0F
#define MPL3115A2_TIME_DLY         0x10
#define MPL3115A2_SYSMOD           0x11
#define MPL3115A2_INT_SOURCE       0x12
#define MPL3115A2_PT_DATA_CFG      0x13
#define MPL3115A2_BAR_IN_MSB       0x14 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_BAR_IN_LSB       0x15 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_P_TGT_MSB        0x16
#define MPL3115A2_P_TGT_LSB        0x17
#define MPL3115A2_T_TGT            0x18
#define MPL3115A2_P_WND_MSB        0x19
#define MPL3115A2_P_WND_LSB        0x1A
#define MPL3115A2_T_WND            0x1B
#define MPL3115A2_P_MIN_MSB        0x1C
#define MPL3115A2_P_MIN_CSB        0x1D
#define MPL3115A2_P_MIN_LSB        0x1E
#define MPL3115A2_T_MIN_MSB        0x1F
#define MPL3115A2_T_MIN_LSB        0x20
#define MPL3115A2_P_MAX_MSB        0x21
#define MPL3115A2_P_MAX_CSB        0x22
#define MPL3115A2_P_MAX_LSB        0x23
#define MPL3115A2_T_MAX_MSB        0x24
#define MPL3115A2_T_MAX_LSB        0x25
#define MPL3115A2_CTRL_REG1        0x26
#define MPL3115A2_CTRL_REG2        0x27
#define MPL3115A2_CTRL_REG3        0x28
#define MPL3115A2_CTRL_REG4        0x29
#define MPL3115A2_CTRL_REG5        0x2A
#define MPL3115A2_OFF_P            0x2B
#define MPL3115A2_OFF_T            0x2C
#define MPL3115A2_OFF_H            0x2D

#define FXOS8700CQ_ADDRESS 0x1E
#define FXAS21000_ADDRESS  0x20
#define MPL3115A2_ADDRESS  0x60  

// Set initial input parameters
enum accelFSR {
  AFS_2g = 0,
  AFS_4g,
  AFS_8g
};

enum accelODR {
  AODR_800HZ = 0, // 200 Hz
  AODR_400HZ,
  AODR_200HZ,
  AODR_100HZ,
  AODR_50HZ,
  AODR_12_5HZ, // 12.5 Hz, etc.
  AODR_6_25HZ,
  AODR_1_56HZ
};

enum magOSR {
  MOSR_0 = 0,  // oversample ratio 2 at 50 and 200 Hz ODR
  MOSR_1,
  MOSR_2,
  MOSR_3,
  MOSR_4,
  MOSR_5,  
  MOSR_6,
  MOSR_7      // oversample ratio 8 at 200 Hz ODR, 32 at 50 HZ ODR
};

enum SAMPLERATE {
  OS_6ms = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  OS_10ms,
  OS_18ms,
  OS_34ms,
  OS_66ms,
  OS_130ms, // 130 ms oversampling interval, 2^5 = 32 oversample ratio
  OS_258ms,
  OS_512ms
};

enum ST_VALUE {
  ATS_1s = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  ATS_2s,
  ATS_4s,
  ATS_8s,
  ATS_16s,
  ATS_32s,
  ATS_64s, // 2^6 = 64 s interval between up to 32 FIFO samples for half an hour of data logging
  ATS_128s,
  ATS_256s,
  ATS_512s,
  ATS_1024s,
  ATS_2048s,
  ATS_4096s,
  ATS_8192s,
  ATS_16384s,
  ATS_32768s  // 2^15 = 32768 s interval between up to 32 FIFO samples = 12 days of data logging!
};

// Set initial input parameters
enum gyroFSR {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS
};

enum gyroODR {
  GODR_800HZ = 0, // 800 Hz
  GODR_400HZ,
  GODR_200HZ,
  GODR_100HZ,
  GODR_50HZ,
  GODR_25HZ, // 25 Hz, etc.
  GODR_12_5HZ,
  GODR_12_55HZ
};


class psIMU
{
	public:
		psIMU();
		void calibAccel();
		void calibMag();
		void calibGyro();
		void initPS();
		void getRawValues(int16_t * raw_values);
		void getValues(float * values);
		void MotionDetect(float * values);
		void getBaro();
		void getYawPitchRoll(float * ypr);
		void getAres();
		void readAccelData(int16_t * destination);
		void readMagData(int16_t * destination);
		int8_t readAccelTempData();
		void calibrateFXOS8700CQ();
		void FXOS8700CQMagOffset() ;
		void sleepModeFXOS8700CQ();
		void FXOS8700CQReset();
		void initFXOS8700CQ();
		void accelMotionIntFXOS8700CQ();
		void FXOS8700CQStandby();
		void FXOS8700CQActive();
		void getGres();
		void readGyroData(int16_t * destination);
		int8_t readGyroTempData();
		void calibrateFXAS21000(float * gBias);
		void FXAS21000Reset();
		void initFXAS21000();
		void FXAS21000Standby();
		void FXAS21000Ready();
		void FXAS21000Active();
		void MPL3115A2readAltitude();
		void MPL3115A2readPressure();
		void MPL3115A2toggleOneShot();
		void MPL3115A2SampleRate(uint8_t samplerate);
		void initFIFOMPL3115A2();
		void initRealTimeMPL3115A2();
		void MPL3115A2TimeStep(uint8_t ST_Value);
		void MPL3115A2enableEventflags();
		void MPL3115A2ActiveAltimeterMode();
		void MPL3115A2ActiveBarometerMode();
		void MPL3115A2Reset();
		void MPL3115A2Standby();
		void MPL3115A2Active();
		void I2Cscan();
		uint8_t checkWAI();
		void setAccelFSR(uint8_t range);
		void setAccelSensitivity(uint16_t range);
		void setAccelODR(uint8_t range);
		void setGyroFSR(uint8_t range);
		void setGyroODR(uint8_t range);
		void setFileCal();
		void setSeaPress(float sea_press_inp);
		
		// Specify FXAS21000 gyro settings
		uint8_t gyroFSR = GFS_250DPS;
		uint8_t gyroODR = GODR_200HZ;
		float gRes, gBias[3] = {0, 0, 0}; // scale resolutions per LSB for the sensors

		bool sleepMode = false;
		
		// Specify MPL3115 Altimeter settings
		uint8_t SAMPLERATE = OS_34ms;
		uint8_t ST_VALUE   = ATS_4s;
		int AltimeterMode  = 0;        // use to choose between altimeter and barometer modes for FIFO data
		float pressure, temperature, altitude, altimeter_setting_pressure_mb;
		float def_sea_press = 1050.0;
		
		// Specify FXOS8700CQ accel/mag settings
		uint8_t accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
		uint16_t accelsensitivity = 4096;                    // 4096 LSB/g
		uint8_t accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
		uint8_t   magOSR = MOSR_5;     // Choose magnetometer oversample rate
		float aRes, mRes, aBias[3] = {0, 0, 0}, mBias[3] = {0, 0, 0};      // Scale resolutions per LSB for the sensor

		float rt;
		int16_t magCount[3], accelCount[3], gyroCount[3];  // Stores the 12-bit signed value
		float q[4];    // vector to hold quaternion
		uint8_t cal_file = 0;
		
		int16_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
		int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
		int32_t accel_bias[3] = {0, 0, 0}, accel_scale[3] = {0, 0, 0};
		uint8_t type = 0;
		
		int instability_fix = 1;
		
		#define calBuiltin
		
		int zeroMotioncount = 0;
		
	private:
		void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
		uint8_t readByte(uint8_t address, uint8_t subAddress);
		void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
};

#endif








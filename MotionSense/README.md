The MotionSense directory contains sketches and a library for accessing the Prop Shield Motion Sensors (IMU).  Teensy_prop_shield is a Teensy sketch developed by Kris Winer and is stand-alone, no libs need or the Prop Shield MotionCal, the sketch was originally posted at https://github.com/kriswiner/Teensy_Prop_Shield.  The psIMU folder is a library that I developed based off of Kris Winer's sketch with a few bells a whistles added:
1. Most of the variables are now accessible directly from the sketch
2. I added several additional function calls for the accelerometer and gyro settings
3. I added an option to use a calibration file and modified the library to accept the FreeIMU library calibration data for the accelerometer and the magnetometer.  The gyro initialization is done automatically at start up.
4. I modified the FreeIMU Calibration GUI to fix a couple of error messages and created a standalone WIndows version of the application.  This is located in the dist directory of the CalibrationGUi directory.  I also added selection of the baudrate as a dropdown from menu bar of the application.
Two examples have been developed.  The psIMU_serial.ino sketch illustrates many of the library.  The second, psIMU_serial_kalman.ino, uses the Kalman Filter developed by Paul S. and uses the new library.

NEW FUNCTIONS:
1.  getRawValues - will get the raw values of the motion sensors plus the barometric pressure and temperature.  In the sketch if you type 'r' in the command line it will print the raw values to the serial monitor.
2.	getValues - gives you access to the calibratied values and is used to get the data for the filters
3.  getBaro() - called to get the barometric pressure and temperature and altitude
4.  setAccelFSR - allows you to set the accelerometer full scale range (see the psIMU.h file for allowable settings)
5.  setAccelSensitivity - sets the accelerometer sensitivity, 4096 for 2g, 2048 for 4g and 1024 for 8g
6.  setAccelODR - sets accelerometer ODR (see psIMU.h for allowable settings)
7.	setGyroFSR -  sets gyro Full Scale range
8.  setGyroODR - sets gyro ODR
9.	setFileCal() - if this is called from the setup it will automatically load the calibration data from the calibration GUI.

CALIBRATION GUI

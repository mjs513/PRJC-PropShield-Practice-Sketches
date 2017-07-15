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
7.	setGyroFSR -  sets gyro Full Scale Range
8.  setGyroODR - sets gyro ODR
9.	setFileCal() - if this is called from the setup it will automatically load the calibration data from the calibration GUI.

CALIBRATION GUI

The Calibration GUI is found in the NXPmotionsense/CalibrationGUI directory and can be downloaded to any where on you machine.  It is based on the FreeIMU Calibration GUI with fixes.  How to use can be found at: https://github.com/mjs513/FreeIMU-Updates/wiki/04.-FreeIMU-Calibration and works in exactly the same way.  When you save the calibration.h file (this is a must, save to EEPROM has not been implemented in the library yet) you need to save it to the psIMU library folder on your machine.

The source files that work with Python 2.7 are in the top level directory.  The standalone version (don't need python installed) is located in the dist folder of the directory.  Double click on the cal_gui.exe file and it will load and run on any windows machine.  I don't have a mac so i could not convert it.

HOW TO USE

After you load the psIMU_serial.ino sketch into the Teensy it will automatically do a I2SCAN and check the sensor "who am i" registers.  If all is well it will continue on to the calibration function.  If you call the setFileCal() funPction it will only do the gryo calibration.  If do not it will enter the builtin function calibration for the prop shield as in Kris Winer's original sketch.  Either way the LED light will remain on during calibration and turn off when finished.  When the IMU has finished intialializing you will see the LED blink 10 times letting you know its ready to receive commands either from the serial monitor or from the processing sketches which we will talk about later.  The available commands are as follows:
1. 'v' = prints out 
2. 'r' = will print out about 20 records of raw values from the sensors including pressure and temperature
3. 'C' = will print out current calibration values.
4. 'y' = will continuously stream orientation values for yaw, pitch and roll until you enter another command in the serial command line, this is also used for the modified version of the teensy visualizer
5. 'z' = will continuously stream encoder data for the FreeIMU_cube_Odo_MulltwiiType Processing
6. 'b' = binary data used only by the calibration GUI.

PROCESSING GUI's

Two GUIs are provided for your use.
1. The first is the Teensy Visualizer developed by Woozy.  I just modified for use with this library.  All functions are still the same.
2. The second is the FreeIMU_cube_Odo_MulltwiiType GUI that I developed for use with the FreeIMU library.  On starting you have to be in the setup tab and you have three things you can do.  Set current sea level presures and write data to a file.  File name and contents have to modified from within the sketch.  After you hit start go to the FreeIMU tab.  There you will see a blank window for plotting various values from the GUI.  Click start to start plotting.  Deselect what is plotted by clicking on the colored box next to the variable name.

TODO:
1. Add EEPROM support


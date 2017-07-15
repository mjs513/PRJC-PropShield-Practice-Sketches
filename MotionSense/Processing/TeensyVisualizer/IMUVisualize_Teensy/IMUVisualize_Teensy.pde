import processing.serial.*;
import java.io.FileWriter; 
import java.io.*;
FileWriter fw;
BufferedWriter bw;
Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

float yawOffset = 0.0;

boolean showHelp = false;
boolean showFPS = false;
boolean showUpdateRate = false;
boolean createLogFile = false;
boolean appendLogFile = false;
boolean showYPR = false; //show Yaw, Pitch, Roll

int updates = 0;
int updateMillis = 0;
int updatesPerSec = 0;

String fileName;

void setup()
{
  size(600, 600, P3D);

  // if you have only ONE serial port active
  //myPort = new Serial(this, Serial.list()[0], 9600); // if you have only ONE serial port active

  // if you know the serial port name
    myPort = new Serial(this, "COM3", 57600);       // Windows "COM#:"
  //myPort = new Serial(this, "COM5:", 9600);        // Windows "COM#:"
  //myPort = new Serial(this, "\\\\.\\COM41", 9600); // Windows, COM10 or higher
  //myPort = new Serial(this, "/dev/ttyACM0", 9600);   // Linux "/dev/ttyACM#"
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac "/dev/cu.usbmodem######"

  //textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
  
  println("Waiting IMU..");

  myDelay(2000);

  while (myPort.available() != 0) {
    println(myPort.available());
      myPort.write("v");
    while ( myPort.readString() == null) {
      //myPort.write("1");
      myDelay(1000);
    }
  }
  myPort.write("y");
}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  if (millis() - updateMillis >= 1000) {
    updatesPerSec = updates;
    updateMillis = millis();
    updates = 0;
  }
  background(31,31,63); // set background to Dark Blue
  lights();
  ambientLight(31, 31, 31);
  lightSpecular(255, 255, 255);
  directionalLight(10, 10, 10, 0, 0, -1);
  specular(127, 127, 127);
  shininess(5.0);
  //smooth(5);

  translate(width/2, height/2); // set position to centre
  //scale(2);
  pushMatrix(); // begin object

  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(-pitch));
  float s2 = sin(radians(-pitch));
  float c3 = cos(radians((-yaw-yawOffset)%360.0));
  float s3 = sin(radians((-yaw-yawOffset)%360.0));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  scale(2);
  drawPropShield();
  drawTeensy();
  //drawText();

  popMatrix(); // end of object
  
  scale(1);
  drawTextM(); //after popMatrix so text won't rotate

  // Print values to console
  print(roll);
  print("\t");
  print(-pitch);
  print("\t");
  print((-yaw-yawOffset)%360.0);
  println();
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 4 && list[0].equals("Orientation:")) {
        yaw = float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll
        updates++;
      }
    }
  } while (message != null);
}

void keyPressed()
{
  //Only act on characters A-Z, a-z (ASCII 65-90, 97-122)
  if((key >= 'A' && key <= 'Z') || (key >= 'a' && key <= 'z')) {
    int keyAscii = key;
    //convert to lowercase
    if (keyAscii <= 'Z') {
      keyAscii += 32;
    }
    switch (keyAscii) {
      
      case 'h':
        showHelp = !showHelp;
        break;
      case 'f':
        showFPS = !showFPS;
        break;
      case 'u':
        showUpdateRate = !showUpdateRate;
        break;
      case 'n':
        showYPR = !showYPR;
        break;
      case 'l':
        createLogFile = !createLogFile;
        appendLogFile = !appendLogFile;
        break;    
      case 'z':
        yawOffset -= 1.0f;
        yawOffset = yawOffset % 360.f;
        break;
      case 'x':
        yawOffset += 1.0f;
        yawOffset = yawOffset % 360.f; //java's modulo is IEEE 754 compatible
        // and will return positive remainder: -5 % 360 = 355
        // https://docs.oracle.com/javase/specs/jls/se7/html/jls-15.html#jls-15.17.3
        break;
    }
  }
}

void drawTextM() {
     textSize(16);
  if (showHelp) {
    
    //white-transparent background
    translate(0,0,159);
    fill(255,192);
    rectMode(CENTER);
    rect(0,0,500,400);
    translate(0,0,-159); //back to origin
    
    //text
    fill(0);
    text("press H to show/hide Help", -190, -120, 160);
    text("press F to show/hide Fps", -190,-80,160);
    text("press U to show/hide orientation Update rate", -190, -40, 160);
    text("press N to show/hide yaw, pitch and roll Numbers", -190, 0, 160);
    text("press L to open to start/stop data Logging", -190, 40, 160);
    text("press Z or X to adjust yaw to match view angle", -190, 80, 160);
  }
  
  if (showFPS) {
    fill(255,255,255);
    text("FPS:",-190,160,130);
    text((int)frameRate, -150,160,130);
  }
  
  if (showUpdateRate) {
    fill(255,255,255);
    text("Updates per sec:",20,160,130);
    text(updatesPerSec,160,160,130);
  }
  
  if (showYPR) {
    fill(255,255,255);
    text("Yaw",80,-190,130);
    text(nfp(yaw,3,2),130,-190,130);
    text("Pitch",80,-160,130);
    text(nfp(-pitch,3,2),130,-160,130);
    text("Roll",80,-130,130);
    text(nfp(roll,3,2),130,-130,130);
  }
  
  if (createLogFile) {
  fill(255,255,255);
  text("CREATE",-80,-190,130);
  }
  
  if (appendLogFile) {
  fill(255,255,255);
  text("APPEND",-80,-160,130);      
  }
   
  
}

void drawPropShield()
{
  // 3D art by Benjamin Rheinland
  stroke(0,90,0); // black outline
  fill(0, 127, 0); // fill color PCB green
  box(190, 6, 70); // PCB base shape

  fill(255, 215, 0); // gold color
  noStroke();

  //draw 14 contacts on Y- side
  translate(65, 0, 30);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(-10, 0, 0); // set new position
  }

  //draw 14 contacts on Y+ side
  translate(10, 0, -60);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(10, 0, 0); // set position
  }

  //draw 5 contacts on X+ side (DAC, 3v3, gnd)
  translate(-10,0,10);
  for (int i=0; i<5; i++) {
    sphere(4.5);
    translate(0,0,10);
  }

  //draw 4 contacts on X+ side (G C D 5)
  translate(25,0,-15);
  for (int i=0; i<4; i++) {
    sphere(4.5);
    translate(0,0,-10);
  }

  //draw 4 contacts on X- side (5V - + GND)
  translate(-180,0,10);
  for (int i=0; i<4; i++) {
    sphere(4.5);
    translate(0,0,10);
  }

  //draw audio amp IC
  stroke(128);
  fill(24);    //Epoxy color
  translate(30,-6,-25);
  box(13,6,13);

  //draw pressure sensor IC
  stroke(64);
  translate(32,0,0);
  fill(192);
  box(10,6,18);

  //draw gyroscope IC
  stroke(128);
  translate(27,0,0);
  fill(24);
  box(16,6,16);

  //draw flash memory IC
  translate(40,0,-15);
  box(20,6,20);

  //draw accelerometer/magnetometer IC
  translate(-5,0,25);
  box(12,6,12);

  //draw 5V level shifter ICs
  translate(42.5,2,0);
  box(6,4,8);
  translate(0,0,-20);
  box(6,4,8);
  
  //reset position to zero
  translate(-76.5,4,10);
}

void drawTeensy() { 
  /* Draw Teensy 3.x Board */
  translate(0, -26.5, 0); // set position of Teensy 3.x
  stroke(0,90,0); // black outline
  fill(0, 127, 0); // fill color PCB green Teensy
  //stroke(40, 0, 60); // set outline colour
  //fill(90, 0, 120); // set fill colour Osh Park Purble
  box(140, 6, 70); // draw Arduino board base shape
  
  /* Draw MCU */
  translate(20, -4, 0); // set position to other edge of Teensy box
  stroke(50, 50, 50); // set outline colour
  fill(0, 0, 0); // set fill colour
  box(40, 2, 40); // draw MCU
  
   fill(255, 215, 0); // gold color
  noStroke();

  //draw 14 contacts on Y- side
  translate(45, 4, 30);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(-10, 0, 0); // set new position
  }

  //draw 14 contacts on Y+ side
  translate(10, 0, -60);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(10, 0, 0); // set position
  }

  //draw 5 contacts on X+ side (DAC, 3v3, gnd)
  translate(-10,0,10);
  for (int i=0; i<5; i++) {
    sphere(4.5);
    translate(0,0,10);
  }
    
  /* Draw USB Cable */
  translate(-125, -6, -30); // set position
  stroke(40, 0, 60); // set outline colour
  fill(200, 200, 200); // set fill colour
  box(25, 8, 35); // draw USB Connector
  
  /* Draw Headers */
  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, 19, 32); // set position to edge
  box(140, 20, 6); // draw pin header as box

  translate(0, 0, -64); // set position to other edge
  box(140, 20, 6); // draw other pin header as box
}

void drawText() {
  textSize(24);
  fill(130, 0, 0, 200);
  //text("Madgwick", -50, 60, 30);
  //text("Mahony", -40, 60, 30);
  //text("NXP Motion Sense", -100, 60, 30);
  //text("TEENSY 3.1 &", -90, 60, 30);
  text("PROP SHIELD", -90, 80, 30);
}


void createLogFile() { // Press a key to save the data
  try {
    //fileName = "C:/Temp/IMUDATA_" + year() + "_" + month() + "_" + day() + "_" + hour() + "_" + minute() + "_" + second() + "_" + millis() + ".csv";
    fileName = "C:/Temp/IMUDATA.csv";
    File file =new File(fileName);
    file.createNewFile();
    FileWriter fw = new FileWriter(file, false);  ///true = append
    BufferedWriter bw = new BufferedWriter(fw);
    PrintWriter pw = new PrintWriter(bw);
 
    pw.println("Yaw, Pitch, Roll");
    pw.println("deg, deg, deg");
    pw.close();
  }
  catch(IOException ioe) {
    System.out.println("Exception ");
    ioe.printStackTrace();
  }
}

void appendLogFile() { // Press a key to save the data
  try {
    File file =new File("C:/Temp/IMUDATA.csv + String.valueOf(year())");
    // chemin = dataPath;
    // positions.txt== your file;
 
    if (!file.exists()) {
      file.createNewFile();
    }
 
    FileWriter fw = new FileWriter(file, true);///true = append
    BufferedWriter bw = new BufferedWriter(fw);
    PrintWriter pw = new PrintWriter(bw);
 
    pw.print(yaw);
    pw.print(", ");
    pw.print(pitch);
    pw.print(", ");
    pw.println(roll); 
    pw.close();
  }
  catch(IOException ioe) {
    System.out.println("Exception ");
    ioe.printStackTrace();
  }
}

void myDelay(int time) {
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}
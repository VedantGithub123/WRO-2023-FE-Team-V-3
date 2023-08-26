// Imports libraries used
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Pixy2.h>

// Defines registries for the gyro sensor
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24


// Creates the Chassis class which includes steering and driving
class Chassis
{
public:
  Chassis(int e1, int i1, int i2, int steerPort)
  {
    enA = e1;
    in1 = i1;
    in2 = i2;

    sPort = steerPort;

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

  }
  void move(int speed)
  {
    speed = speed / abs(speed) * min(255, abs(speed));
    analogWrite(enA, abs(speed));
    if (speed == 0)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    } else if (speed > 0)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
  }
  void steer(int angle)
  {
    steering.write(angle / abs(angle) * min(abs(angle), 70) + 94);
  }
  void attachServo()
  {
    steering.attach(sPort);
  }
  Servo steering;

  int enA;
  int in1;
  int in2;
  int sPort;
};

// Creates the rgbSensor class which is used for the rgb sensor
class rgbSensor
{
public:
  int getColor()
  {
    float red, green, blue;

    tcs.setInterrupt(false);

    tcs.getRGB(&red, &green, &blue);

    tcs.setInterrupt(true);

    int col = 0;  //0: White, 1: Orange, 2: Blue
    if (blue > 90 && red < 60) { col = 2; }
    if (red > 80 && blue < 60) { col = 1; }
    return col;
  }

  void setup()
  {
    tcs.begin();
  }

  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
};

// Creates the IRSensors class which is used to read the IR sensor values
class IRSensors
{
public:
  IRSensors(int ports[])
  {
    for (int i = 0; i < 4; i++)
    {
      irPorts[i] = ports[i];
    }
  }

  int getDistance(int port)
  {
    port = max(min(port, 3), 0);
    if (irPorts[port]==A1)
    {
      return (11.63417 + (241.6444 - 11.63417)/pow((1 + (analogRead(irPorts[port])*5.0/1023.0/0.5075672)), 1.868922))*1.5;
    }else if (port==3)
    {
      return 1-digitalRead(port);
    }
    return analogRead(irPorts[port])>200;
  }

  int getDistanceClose(int port)
  {
    port = max(min(port, 3), 0);
    if (irPorts[port]==A1)
    {
      return (11.63417 + (241.6444 - 11.63417)/pow((1 + (analogRead(irPorts[port])*5.0/1023.0/0.5075672)), 1.868922))*1.5;
    }else if (port==3)
    {
      return 1-digitalRead(port);
    }
    return analogRead(irPorts[port])>700;
  }

  void setup()
  {
    for (int i : irPorts)
    {
      if (i<14)
      {
        pinMode(i, INPUT);
      }
    }
  }
  int irPorts[4];
};

// Creates the Gyro class which is used to measure our heading
class Gyro
{
public:

  void setup()
  {
    writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
    writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);
    writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
    writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
    delay(1);
  }

  void writeRegister(int deviceAddress, byte address, byte val)
  {
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission();
  }

  int readRegister(int deviceAddress, byte address)
  {
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 1);
    delayMicroseconds(1);
    v = Wire.read();
    return v;
  }

  float getGyroChange()
  {
    byte zMSB = readRegister(L3G4200D_Address, 0x2D);
    byte zLSB = readRegister(L3G4200D_Address, 0x2C);
    return ((zMSB << 8) | zLSB);
  }

  float getAngle()
  {
    return angle;
  }

  void calibrate()
  {
    for (int i = 0; i < 6000; i++)
    {
      drift += getGyroChange();
    }
    drift /= 6000;
    prevTime = micros();
  }

  void updateGyro()
  {
    angle += (micros() - prevTime) / 1000000.0 * (getGyroChange() - drift) / -14.286 * 9/8;
    prevTime = micros();
    Serial.println("hello");
  }

  float angle = 0;
  unsigned long int prevTime = micros();
  float drift = 0;
  int L3G4200D_Address = 105;
};

// Creates the Camera class which is used to get certain blocks from the PixyCam 2.1
class Camera
{
public:
  void setup()
  {
    pixy.init();
  }

  Block getClosest()
  {
    pixy.ccc.getBlocks();

    int lowInd = 0;
    int lowVal = pixy.ccc.blocks[0].m_y;
    if (pixy.ccc.numBlocks)
    {
      for (int i = 1; i<pixy.ccc.numBlocks; i++)
      {
        if (pixy.ccc.blocks[i].m_y>lowVal)
        {
          lowVal = pixy.ccc.blocks[i].m_y;
          lowInd = i;
        }
      }
    }

    return pixy.ccc.blocks[0];
  }

  int getObjectNum()
  {
    pixy.ccc.getBlocks();
    return pixy.ccc.numBlocks;
  }

  Pixy2 pixy;
};


// Constructs an instance of the classes
Chassis chassis(5, 6, 7, 9); // For movement
rgbSensor rgbSense; // For sensing lines on the mat
const int irPorts[4] = {A0, A1, A2, 3}; // Ports for the IR sensors
IRSensors irSensors(irPorts); // For detecting distance in the front and sides
Gyro gyro; // For detecting the angle of our robot
Camera camera; // For getting the obstacles
const int buttonPort = 8; // Pin for the pushbutton

int speed = 0; // Defines the speed for the robot and is used for acceleration
int cornerCount = 0; // Number of corners passed
unsigned long int endTime = 10000000000; // 
int dir = 0; // Stores the direction of our robot: | 0: Undecided | 1: Clockwise | 2: Counterclockwise |
int cornerScanDelay = 0; // Stores time of last line detected for time between corners
bool cornerDetected = false; // Boolean to store if the line has been detected
float steer = 0.0;
Block prevObj; // Stores the last blcok
Block prevObj2; // Stores the last passed block
Block closeBlock;
int target = 0; // Stores the target position of the blocks
float err = 0; // Stores the error for following the object
const float kP = -0.35; // Stores the kP for following the object

const int CURVE = 0; // Amount to curve when going straight
int objDelay = 0;

int targetAngle = 0; // Angle to turn gyro

void setup()
{
  Serial.begin(9600); // Starts the serial monitor for debugging

  // Sets the pin mode for the button
  pinMode(buttonPort, INPUT);
  // Sets up the electronic components
  Serial.println("ABC");
  chassis.attachServo();
  chassis.steer(60);
  delay(500);
  rgbSense.setup();
  irSensors.setup();
  gyro.setup();
  gyro.calibrate();
  camera.setup();
  // delay(1000);
  chassis.steer(0); // Aligns steering when everything is ready

  // Start the program once the button is pressed
  while (!digitalRead(buttonPort))
  {
    // camera.getClosest().print();
    gyro.updateGyro();
    // Serial.println(gyro.getAngle());
  }
}

void delay_2(int s){
  unsigned long int x = millis();
  while (millis()-x<s){
    gyro.updateGyro();
  }
}

void open()
{
  speed = 255; // Sets speed to 255

  steer = -1; // Steering is set to straight

  if (dir==0)
  { // Rewrites direction to the color of the sensor if the direction is not defined
    dir = rgbSense.getColor();
  }

  if(cornerDetected && irSensors.getDistance(1)<40)
  { // Starts turning when the corner is detected and the robot is close to the wall
    
    // Turns and sets the bias based on which direction the robot is moving
    if (dir==1)
    {
      chassis.steer(30);
    }else
    {
      chassis.steer(-30);
    }

    // Waits until the front IR sensor does not see anything
    while (irSensors.getDistance(1)<50)
    {
      // gyro.updateGyro();
    }

    cornerDetected = false; // Resets cornerDetected variable until the line is seen again
  }

  if (rgbSense.getColor()==dir && dir!=0 && millis()-cornerScanDelay>2000)
  { // Checks if the line is seen and if the time after the last scan was greater than 2000ms
    delay(60);
    cornerCount++; // Increases the amount of corners we have passed
    if (cornerCount==12)
    { // If the gyro angle says 3 laps are finished, stop the robot after some time
      endTime = millis()+3500;
    }

    cornerDetected = true; // Flags the cornerDetected variable as true
    
    cornerScanDelay = millis(); // Updates cornerScanDelay with the current time
  }

  if (irSensors.getDistance(0))
  {
    steer = 30;
  }else if (irSensors.getDistance(2))
  {
    steer = -30;
  }

  if (millis()<endTime)
  { // If the time is not finished, keep moving the robot
    chassis.steer(steer);
    chassis.move(speed);
  }else
  { // If the time is up, stop the robot
    chassis.move(0);
    chassis.steer(0);
    delay(100000000);
  }
}



void obstacle_2()
{

  speed = 255; // Sets speed to 255

  steer = -1; // Steering is set to straight

  closeBlock = camera.getClosest(); // Gets closest block from the camera

  if (closeBlock.m_y>180 && closeBlock.m_signature<=2)
  { // Set the most recent block to the old block
    prevObj2 = closeBlock;
    objDelay = millis();
  }

  if (dir==0)
  { // Rewrites direction to the color of the sensor if the direction is not defined
    dir = rgbSense.getColor();
  }

  if (closeBlock.m_signature<=2 && (millis() - objDelay > 500 || closeBlock.m_y>190))
  { 
    if (closeBlock.m_signature==1)
    { // Sets the target for the block position on the left based on how far it is if it is red
      target = (207-closeBlock.m_y)/2+5;
    }else
    { // Sets the target for the block position on the right based on how far it is if it is green
      target = 310.0-(207-closeBlock.m_y)/2;
    }
    err = target - (int)closeBlock.m_x; // Sets the error to the difference between the current position and the target
    steer = err*kP; // Gets steering value
    if (steer>30){
      steer = 30;
    }else if (steer<-30){
      steer = -30;
    }
  }
  else
  {
    err = targetAngle - gyro.getAngle();
    if(abs(err) > 10)
    {
      steer = err*4.5;
      if (steer>30){
        steer = 30;
      }else if (steer<-30){
        steer = -30;
      }
    }
    delay(1);
  }

  if (rgbSense.getColor()==dir && dir>0 && millis()-cornerScanDelay>1000)
  { // Checks if the color is the same is the direction to travel and if it has been 1 second past the start or the reversing of the 3rd lap
    cornerCount++; // Increments 1 to cornerCount
    if (cornerCount>=12)
    { // If all the corners are passed, set the program to stop after 3.8 seconds
      endTime = millis()+3800;
    }
    if (cornerCount==8 && prevObj2.m_signature==1)
    { // If 2 laps are finished and the last object was red, run the turning sequence
      targetAngle += (dir == 1 ? 180 : -180) ;
      dir = 3-dir; // sets the direction to the opposite way
      while (irSensors.getDistance(1)>40){
        gyro.updateGyro();
      } // Get close to the front wall
      // Turn for 1.5 seconds
      chassis.steer(-40);
      while (abs(gyro.getAngle())<abs(targetAngle)){
        gyro.updateGyro();
      }
      cornerCount++;
      cornerScanDelay = millis();
    }else
    {
      int check = 0;
      int check2 = 0; 
      for (int i = 0; i<6; i++){
        int blockSignature = camera.getClosest().m_signature;
        if (blockSignature==3-dir){
          check++;
        }
        else if (blockSignature==dir){
          check2++;
        }
      }
      targetAngle += (dir == 1 ? 90 : -90) ;
      if (check>2)
      { // If the block in front says that you need to go from the outside of the lap, run the following code
        unsigned long int prevTime = millis();
        while ((irSensors.getDistance(1)>40 || millis()-prevTime<1000) && millis()-prevTime<2500)
        { // Follow the block until the robot is close to the wall
          closeBlock = camera.getClosest();
          if (closeBlock.m_signature<=2){
            if (closeBlock.m_signature==1)
            { // Sets the target for the block position on the left based on how far it is if it is red
              target = (207-closeBlock.m_y)/2+15;
              target = 0;
            }else
            { // Sets the target for the block position on the right based on how far it is if it is green
              target = 300.0-(207-closeBlock.m_y)/2;
              target = 315;
            }
            err = target - (int)closeBlock.m_x; // Sets the error to the difference between the current position and the target
            steer = err*kP; // Gets steering value
            if (steer>30){
              steer = 30;
            }else if (steer<-30){
              steer = -30;
            }
            if (closeBlock.m_signature<=2)
            {
              prevObj2 = closeBlock;
            }
          }else{
            steer = 0;
          }
          chassis.steer(steer);
          gyro.updateGyro();
        }
        // Turn until the robot sees nothing
        chassis.steer((1.5-dir)*80);
        while (irSensors.getDistance(1)<155){
          gyro.updateGyro();
        }
      }
      else if (check2 > 2){
        // If the robot needs to turn on the inside, go forward a bit and then turn
        chassis.steer(0);
        delay_2(400);
        chassis.steer((1.5-dir)*80);
        int prevTime = millis();
        // gyro.angle = 0;
        while (abs(gyro.getAngle())<(abs(targetAngle) - 30) && !irSensors.getDistanceClose(0) && !irSensors.getDistanceClose(2)){
          gyro.updateGyro();
        }
        // // Curve a bit to the center
        // chassis.steer(0);
        // delay_2(200);
        // chassis.steer((dir-1.5)*50);
        // delay_2(300);
        // chassis.steer(0);
      }
      else
      { // If the robot sees nothing, turn
        chassis.steer((1.5-dir)*90);
        int prevTime = millis();
        // gyro.angle = 0;
        while (abs(gyro.getAngle())<(abs(targetAngle) - 10)  && !irSensors.getDistanceClose(0) && !irSensors.getDistanceClose(2)){
          gyro.updateGyro();
        }
      }
    }
  }

  if (irSensors.getDistance(0))
  {
    if(closeBlock.m_signature != 1)
    {
      steer = 20;
    }
    else if (irSensors.getDistanceClose(0)) 
      steer = 30;
  }else if (irSensors.getDistance(2))
  {
    if(closeBlock.m_signature != 2)
    {
      steer = -20;
    }
    else if (irSensors.getDistanceClose(2)) 
      steer = -30;
  }

  if (irSensors.getDistance(3))
  {
    chassis.steer(0);
    chassis.move(-255);
    delay_2(3000);
    chassis.move(255);
    endTime+=3000;
  }

  gyro.updateGyro();

  if (millis()<endTime)
  { // If the time is not finished, keep moving the robot
    chassis.steer(steer);
    chassis.move(speed);
  }else
  { // If the time is up, stop the robot
    chassis.move(0);
    chassis.steer(0);
    delay(10000000);
  }

}

void loop()
{
  // open();
  // challenge();
  obstacle_2();

}
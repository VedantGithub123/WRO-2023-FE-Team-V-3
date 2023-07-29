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
  Chassis(int e1, int i1, int i2, int e2, int i3, int i4, int steerPort)
  {
    enA = e1;
    in1 = i1;
    in2 = i2;

    enB = e2;
    in3 = i3;
    in4 = i4;

    sPort = steerPort;

    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

  }
  void move(int speed)
  {
    speed = speed / abs(speed) * min(255, abs(speed));
    analogWrite(enA, abs(speed));
    analogWrite(enB, abs(speed));
    if (speed == 0)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    } else if (speed > 0)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    } else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
  }
  void steer(int angle)
  {
    steering.write(angle / abs(angle) * min(abs(angle), 35) + 35);
  }
  void attachServo()
  {
    steering.attach(sPort);
  }
  Servo steering;

  int enA;
  int in1;
  int in2;
  int enB;
  int in3;
  int in4;
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

    delay(3);

    tcs.getRGB(&red, &green, &blue);

    tcs.setInterrupt(true);

    int col = 0;  //0: White, 1: Orange, 2: Blue
    if (blue > 95) { col = 2; }
    if (red > 85) { col = 1; }
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
    for (int i = 0; i < 6; i++)
    {
      irPorts[i] = ports[i];
    }
  }
  int getDistance(int port)
  {
    port = max(min(port, 5), 0);
    return (11.63417 + (241.6444 - 11.63417)/pow((1 + (analogRead(port)*5.0/1023.0/0.5075672)), 1.868922))*1.5;
  }
  int irPorts[6];
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
    delay(500);
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
    delay(1);
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
    for (int i = 0; i < 4000; i++)
    {
      drift += getGyroChange();
    }
    drift /= 4000;
    prevTime = micros();
  }

  void updateGyro()
  {
    angle += (micros() - prevTime) / 1000000.0 * (getGyroChange() - drift) / 2000.0 * -330;
    prevTime = micros();
    Serial.println("hello");
  }

  float angle = 0;
  unsigned long int prevTime = micros();
  float drift = 0;
  int L3G4200D_Address = 105;
};

class ColorBlock : Block 
{
  public:
    void getPosition(int &x, int &y)
    {
      x = this->m_x;
      y = this->m_y - (this->m_height / 2);
    }
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

    return pixy.ccc.blocks[lowInd];
  }

  Pixy2 pixy;
};

class USSensor
{
public:
  USSensor(int port1, int port2)
  {
    trig = port1;
    echo = port2;
  }

  void setup()
  {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
  }

  float getDistance()
  {
    digitalWrite(trig, LOW);
    delayMicroseconds(10);
    digitalWrite(trig, HIGH);
    delayMicroseconds(100);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH);
    float distance = duration * 0.034 / 2.0;
    if (distance==0)
    {
      distance=1000;
    }
    return distance;
  }

  int trig = 0; 
  int echo = 0;
};

// Constructs an instance of the classes
Chassis chassis(3, 8, 7, 6, 5, 4, 10); // For movement
rgbSensor rgbSense; // For sensing lines on the mat
const int irPorts[6] = {A0, A1, A2}; // Ports for the IR sensors
IRSensors irSensors(irPorts); // For detecting distance in the front and wall following
USSensor leftSensor(A3, 9); // For detecting distance on the left
USSensor rightSensor(A3, 0); // For detecting distance on the right
Gyro gyro; // For detecting the angle of our robot
Camera camera; // For getting the obstacles
const int buttonPort = 2; // Pin for the pushbutton

int speed = 0; // Defines the speed for the robot and is used for acceleration
unsigned long int prevTime = millis(); // Stores the previous time for wall following
const float kP = -0.18; // kP value for wall following
const float kD = 0.01; // kD value for wall following
const float kI = -0.0; // kI value for wall following
float integral = 0; // Holds the integral value for wall following
float prevErr = 0; // Holds preveious error for wall following
int cornerCount = 0; // Number of corners passed
unsigned long int endTime = 10000000000; // 
int dir = 0; // Stores the direction of our robot: | 0: Undecided | 1: Clockwise | 2: Counterclockwise |
float bias = 0; // Bias for the error to follow on the outer wall
int cornerScanDelay = 0; // Stores time of last line detected for time between corners
bool cornerDetected = false; // Boolean to store if the line has been detected

void setup() {
  Serial.begin(9600); // Starts the serial monitor for debugging

  // Sets the pin mode for the button
  pinMode(buttonPort, INPUT);

  // Sets up the electronic components
  chassis.attachServo();
  rgbSense.setup();
  leftSensor.setup();
  rightSensor.setup();
  gyro.setup();
  gyro.calibrate();
  camera.setup();
  chassis.steer(0); // Aligns steering when everything is ready

  // Start the program once the button is pressed
  while (!digitalRead(buttonPort))
  {
    delay(5);
  }

  prevTime = millis()-1; // Sets previous time to current time minus one to not divide by 0
}

void open(){

  // Accelerates robot
  speed+=10;
  speed = min(speed, (dir==0) ? 105 : 205); // Sets max speed to 105 until direction is defined

  float err = irSensors.getDistance(2)-irSensors.getDistance(0)+bias; // Gets error from IR sensors
  integral+=(millis()-prevTime)*err; // Adds to integral

  float steer = err*kP+(err-prevErr)/(millis()-prevTime)*kD+integral*kI; // Gets steering value

  if (dir==0)
  { // Rewrites direction to the color of the sensor if the direction is not defined
    dir = rgbSense.getColor();
  }

  if(cornerDetected && irSensors.getDistance(1)<50)
  { // Starts turning when the corner is detected and the robot is close to the wall
    
    // Turns and sets the bias based on which direction the robot is moving
    if (dir==1)
    {
      bias = 17;
      chassis.steer(35);
    }else
    {
      bias=-15;
      chassis.steer(-35);
    }

    // Waits until the front IR sensor does not see anything
    while (irSensors.getDistance(1)<100)
    {
      gyro.updateGyro();
    }

    cornerDetected = false; // Resets cornerDetected variable until the line is seen again
  }

  if (rgbSense.getColor()==dir && dir!=0 && millis() - cornerScanDelay > 500)
  { // Checks if the line is seen and if the time after the last scan was greater than 500ms

    cornerCount++; // Increases the amount of corners we have passed
    if (cornerCount>=12 && abs(gyro.getAngle()) > 1050)
    { // If the corners passed are more than 12 and the gyro angle says 3 laps are finished, stop the robot after some time
      endTime = millis()+2000;
    }

    cornerDetected = true; // Flags the cornerDetected variable as true
    
    cornerScanDelay = millis(); // Updates cornerScanDelay with the current time
  }

  prevErr = err; // Updates prevErr
  prevTime = millis(); // Updates prevTime


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

  gyro.updateGyro(); // Update the gyro
}

void challenge(){

}

void loop() {
  open();
  // challenge();

}
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
class Chassis {
public:
  Chassis(int e1, int i1, int i2, int e2, int i3, int i4, int steerPort) {
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
  void move(int speed) {
    speed = speed / abs(speed) * min(255, abs(speed));
    analogWrite(enA, abs(speed));
    analogWrite(enB, abs(speed));
    if (speed == 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    } else if (speed > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
  }
  void steer(int angle) {
    steering.write(angle / abs(angle) * min(abs(angle), 35) + 35);
    // steering.write(angle);
  }
  void attachServo() {
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
class rgbSensor {
public:
  int getColor() {
    float red, green, blue;

    tcs.setInterrupt(false);  // turn on LED

    delay(3);  // takes 50ms to read

    tcs.getRGB(&red, &green, &blue);

    tcs.setInterrupt(true);  // turn off LED

    int col = 0;  //0: White, 1: Orange, 2: Blue
    if (blue > 95) { col = 2; }
    if (red > 85) { col = 1; }
    return col;
  }

  void setup() {
    tcs.begin();
  }

  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
};

// Creates the IRSensors class which is used to read the IR sensor values
class IRSensors {
public:
  IRSensors(int ports[]) {
    for (int i = 0; i < 6; i++) {
      irPorts[i] = ports[i];
    };
  }
  int getDistance(int port) {
    port = max(min(port, 5), 0);
    return (11.63417 + (241.6444 - 11.63417)/pow((1 + (analogRead(port)*5.0/1023.0/0.5075672)), 1.868922))*1.5;
  }
  int irPorts[6];
};

// Creates the Gyro class which is used to measure our heading
class Gyro {
public:

  void setup() {
    writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
    writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);
    writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
    writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
    delay(1500);
  }

  void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission();
  }

  int readRegister(int deviceAddress, byte address) {
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 1);
    delay(1);
    v = Wire.read();
    return v;
  }

  float getGyroChange() {
    byte zMSB = readRegister(L3G4200D_Address, 0x2D);
    byte zLSB = readRegister(L3G4200D_Address, 0x2C);
    return ((zMSB << 8) | zLSB);
  }

  float getAngle() {
    return angle;
  }

  void calibrate() {
    for (int i = 0; i < 2000; i++) {
      drift += getGyroChange();
    }
    drift /= 2000;
    prevTime = micros();
  }

  void updateGyro() {
    angle += (micros() - prevTime) / 1000000.0 * (getGyroChange() - drift) / 2000.0 * -270;
    prevTime = micros();
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
class Camera {
public:
  void setup(){
    pixy.init();
  }

  Block getClosest(){
    pixy.ccc.getBlocks();

    int lowInd = 0;
    int lowVal = pixy.ccc.blocks[0].m_y;
    if (pixy.ccc.numBlocks){
      for (int i = 1; i>pixy.ccc.numBlocks; i++){
        if (pixy.ccc.blocks[i].m_y<lowVal){
          lowVal = pixy.ccc.blocks[i].m_y;
          lowInd = i;
        }
      }
    }

    return pixy.ccc.blocks[lowInd];
  }

  Pixy2 pixy;
};

class USSensor {
public:
  USSensor(int port1, int port2){
    trig = port1;
    echo = port2;
  }

  void setup(){
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
  }

  float getDistance(){
    digitalWrite(trig, LOW);
    delayMicroseconds(10);
    digitalWrite(trig, HIGH);
    delayMicroseconds(100);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH);
    float distance = duration * 0.034 / 2.0;
    if (distance==0){
      distance=1000;
    }
    return distance;
  }
  
  int trig = 0; 
  int echo = 0;
};

// Constructs an instance of the classes
Chassis chassis(3, 8, 7, 6, 5, 4, 10);
rgbSensor rgbSense;
int irPorts[6] = {A0, A1, A2};
IRSensors irSensors(irPorts);
USSensor leftSensor(A3, 9);
USSensor rightSensor(A3, 0);
Gyro gyro;
Camera camera;
// Defines the pin for the pushbutton
int buttonPort = 2;

int speed = 0;

unsigned long int prevTime = millis();

void setup() {
  Serial.begin(9600); // Starts the serial monitor for debugging

  // Sets the pin mode
  pinMode(buttonPort, INPUT);

  // Sets up the electronic components
  chassis.attachServo();
  rgbSense.setup();
  leftSensor.setup();
  rightSensor.setup();
  // camera.setup();
  gyro.setup();
  gyro.calibrate();
  
  chassis.steer(0);

  // Start the program once the button is pressed

  while (!digitalRead(buttonPort)){
    // chassis.move(255);
    // Serial.println(rgbSense.getColor());
    // Serial.println(leftSensor.getDistance());
    gyro.updateGyro();
    // Serial.println(abs(gyro.getAngle()));
    delay(1);
  }
  
  prevTime = millis()-1;
}

float kP = -0.2;
float kD = -0;
float kI = -0.0;
float integral = 0;
float prevErr = 0;
int cornerCount = 0;
unsigned long int endTime = 10000000000;
int dir = 0;

float bias = 0;

float err;
float steer;

void open(){

  speed+=20;
  speed = min(speed, 205);

  err = irSensors.getDistance(2)-irSensors.getDistance(0)+bias;
  integral+=(millis()-prevTime)*err;
  
  steer = err*kP+(err-prevErr)/(millis()-prevTime)*kD+integral*kI;

  if (dir==0){
    dir = rgbSense.getColor();
  }

  if (rgbSense.getColor()==dir && dir!=0 || dir > 0){
    
    cornerCount++;
    if (cornerCount>=12){
      endTime = millis()+4000;
    }
    USSensor ultrasonic = (dir==1) ? rightSensor : leftSensor;
    while (((ultrasonic.getDistance()<75) || (ultrasonic.getDistance()>1000))){
      err = irSensors.getDistance(2)-irSensors.getDistance(0)+bias;
      integral+=(millis()-prevTime)*err;
      steer = err*kP+(err-prevErr)/(millis()-prevTime)*kD+integral*kI;
      if (irSensors.getDistance(0)<25){
        steer = 20;
      }else if (irSensors.getDistance(2)<25){
        steer = -20;
      }
      chassis.steer(steer);
      prevTime = millis();
      prevErr = err;
      gyro.updateGyro();
    }
    chassis.steer(0);
    while (irSensors.getDistance(1)>70){gyro.updateGyro();}
    chassis.move(165);
    if (dir==1){
      bias = 17;
      chassis.steer(35);
    }else{
      bias=0;
      chassis.steer(-35);
    }

    do {
      gyro.updateGyro();
      Serial.println(gyro.getAngle());
    }while(abs(gyro.getAngle()) < (cornerCount*90)-20);
    
    chassis.move(205);
    // prevTime = millis();
    // while (millis()-prevTime<1000){
    //   gyro.updateGyro();
    // }
    // chassis.move(205);
    prevTime = millis()-1;
  }
  if (irSensors.getDistance(0)<30){
    steer = 20;
  }else if (irSensors.getDistance(2)<30){
    steer = -20;
  }

  prevErr = err;
  prevTime = millis();
  
  if (millis()<endTime){
    chassis.steer(steer);

    chassis.move(speed);
  }else{
    chassis.move(0);
    chassis.steer(0);
    delay(10000000);
  }
  gyro.updateGyro();
}

void challenge(){

  speed+=20;
  speed = min(speed, 205);

  if (dir==0){
    dir = rgbSense.getColor();
  }

  Block closeBlock = camera.getClosest();
  if (closeBlock.m_signature>2){
    steer = 0;
  }else{
    int target;
    if (closeBlock.m_signature==1){
      target = (207-closeBlock.m_y)/1.3;
    }else{
      target = -1*(207-closeBlock.m_y)/1.3+315;
    }
    err = closeBlock.m_x-target;
    steer = err*10.3;
  }

  if (leftSensor.getDistance()<5 || irSensors.getDistance(2)<25){
    steer = 20;
  }else if (rightSensor.getDistance()<5 || irSensors.getDistance(0)<25){
    steer = -20;
  }
  
  if (millis()<endTime){
    chassis.steer(steer);

    chassis.move(speed);
  }else{
    chassis.move(0);
    chassis.steer(0);
    delay(10000000);
  }
  delay(1);
}

void loop() {
  open();
  // challenge();

}
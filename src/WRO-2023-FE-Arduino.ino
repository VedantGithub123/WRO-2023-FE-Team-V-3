#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24



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
    } else if (speed < 0) {
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

class rgbSensor {
public:
  int getColor() {
    float red, green, blue;

    tcs.setInterrupt(false);  // turn on LED

    delay(60);  // takes 50ms to read

    tcs.getRGB(&red, &green, &blue);

    tcs.setInterrupt(true);  // turn off LED

    int col = 0;  //0: White, 1: Orange, 2: Blue
    if (blue > 80) { col = 2; }
    if (red > 80) { col = 1; }
    Serial.print(col);
    Serial.print("\n");
    return col;
  }

  void setup() {
    tcs.begin();
  }

  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
};

class IRSensors {
public:
  IRSensors(int ports[]) {
    for (int i = 0; i < 6; i++) {
      irPorts[i] = ports[i];
    };
  }
  int getDistance(int port) {
    port = max(min(port, 5), 0) + 14;
    return analogRead(port) * 130 / 1023 + 20;
  }
  int irPorts[6];
};

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
    delay(10);
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
    for (int i = 0; i < 200; i++) {
      drift += getGyroChange();
      delay(10);
    }
    drift /= 200;
    prevTime = millis();
  }

  void updateGyro() {
    angle += (millis() - prevTime) / 1000.0 * (getGyroChange() - drift) / 2000.0 * 360;
    prevTime = millis();
  }

  float angle = 0;
  unsigned long int prevTime = millis();
  float drift = 0;
  int L3G4200D_Address = 105;
};

Chassis chassis(9, 8, 7, 6, 5, 4, 13);
rgbSensor rgbSense;
int irPorts[6] = { A0, A1, A2, A3, A4, A5 };
IRSensors irSensors(irPorts);
Gyro gyro;

void setup() {
  Serial.begin(9600);
  
  chassis.attachServo();
  rgbSense.setup();
  // gyro.setup();
  // gyro.calibrate();

  chassis.steer(0);
  delay(1000);
  chassis.move(255);
  delay(1000);
  // while (rgbSense.getColor()!=2){
  //   chassis.move(255);
  // }
  // chassis.move(0);
  // delay(1000);
  // while (rgbSense.getColor()!=1){
  //   chassis.move(255);
  // }
  // chassis.move(0);
}

void loop() {
  // gyro.updateGyro();
  // delay(10);
}
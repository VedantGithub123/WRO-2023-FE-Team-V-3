Electrical
====

This directory consists of schematic diagrams demonstrating the connections between different components as well as datasheets for the components. It also consists of the reasoning behind the choice of components.

## Parts Explanation

| Part Name | Explanation | Image | Datasheet |
| ----------- | ----------- | ----------- | ----------- |
| 1.5V Rechargeable Lithium Ion Battery | We are using this battery to power all of our electronics as it is compact and easy to get. They are also efficient for us as they are rechargeable and do not have to be replaced. | <img alt="image" src="https://github.com/user-attachments/assets/002c3b0a-4d27-4460-bba1-95bd421bd0cf" /> | <img alt="image" src="https://github.com/user-attachments/assets/eec69255-ed3a-4646-8c84-cee22d00a063" /> |
| DC 6V Micro Gear Box Speed Reduction Motor, 300RPM | We are using this specific motor to drive the robot as it contains a gearbox that can reach a maximum speed of 300RPM. The compact size of these motors, coupled with a high RPM, allowed for our smaller compact design to succeed. | <img alt="image" src="https://github.com/user-attachments/assets/c83f4aa0-e80b-4144-8081-84eb509f2d6c" /> | <img width="862" height="618" alt="image" src="https://github.com/user-attachments/assets/6855160d-0e40-474e-af2c-287a416f55d9" /> |
| CJMCU34725 TCS34725 RGB Light Color Sensor | Using this RGB color sensor allows our robot to sense the blue and orange lines at the corners of the mat. This allows us to keep track of how many laps the robot has completed and indicate when the robot needs to turn. | <img alt="image" src="https://github.com/user-attachments/assets/5ef196ab-ebb1-46c1-a03f-45d3362d52dd" /> | [Color Sensor Info](https://www.waveshare.com/w/upload/b/bb/TCS34725_Color_Sensor_user_manual_en.pdf) |
| Arduino Nano | In order to process the information from the sensors and make decisions based on the running code, we are using the Arduino Nano, which is programmed with the Arduino IDE. This board is easy to use, and there are many libraries available and on demand. The board contains 14 digital input/output pins, enabling us to connect a multitude of devices. It also has eight analog pins, of which two are used for I2C communication. | <img width="863" height="647" alt="image" src="https://github.com/user-attachments/assets/04c7d474-26b0-4a6f-be25-7cab0e49f8b1" /> | [Arduino Nano Info](https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf) |
| ES08MA Metal Analog Servo | In order to steer our vehicle, we use a servo motor with 2.0 Kilogram-force centimetres of torque and a speed of 0.10 sec/60°. Since steering will require a reasonably large amount of torque, this motor provides more than enough for our application. Furthermore, this servo motor can turn quickly, which allows us to make incredibly sharp turns. | <img alt="image" src="https://github.com/user-attachments/assets/94113ba6-0094-4033-8e6b-5b146b72a888" /> | [Servo Info](https://m.media-amazon.com/images/I/61ZU3A84tYS._AC_SL1000_.jpg) |
| GP2Y0A02YK0F IR Range Sensor | The IR sensors provide the necessary information on the distance to the walls at an extremely fast rate. These specific sensors sense distance accurately from 20 to 150cm. We use one of these sensors to detect how close we are to any wall in front of our robot. We use the other two to follow the wall to make sure or robot is in the middle of the track | <img alt="image" src="https://github.com/user-attachments/assets/43c7b334-afb5-45ef-b92a-9cfaef0b0fe7" /> | [GP2Y0A02YK0F IR Range Sensor Info](https://cdn.robotshop.com/media/d/dem/rb-dem-02/pdf/datasheet-gp2y0a02yk0f.pdf) |
| GP2Y0A41SK0F IR Range Sensor | We use these sensors to measure the distance to any walls on the sides of our robot. Since we need sensors with a smaller range to detect when we are getting close to the wall, we are using these ones. | <img alt="image" src="https://github.com/user-attachments/assets/29fd987c-ccc2-4ff4-9c4a-38896d2a8a89" /> | [GP2Y0A41SK0F IR Sensor Info](https://www.pololu.com/file/0J713/GP2Y0A41SK0F.pdf) |
| HW-201 IR Sensor | In order to detect if our robot is going to crash into any object in the front, we use these sensors as a fail-safe to make sure our robot will still work. | <img alt="image" src="https://github.com/user-attachments/assets/77a22f8a-0885-4a82-bd19-7010d6b82a49" /> | [HW-201 IR Sensor Info](https://www.circuits-diy.com/hw201-infrared-ir-sensor-module/) |
| L298N Dual H-Bridge Motor Driver | To effectively power our motors, we are using the L298N motor driver, where we are able to control the direction and speed of both motors using Pulse Width Modulation (PWM). | <img alt="image" src="https://github.com/user-attachments/assets/b37065ca-9433-4ca0-a4e3-d85a8d3ef765" /> | [Motor Driver Info](http://www.handsontec.com/dataspecs/L298N%20Motor%20Driver.pdf) |
| L3G4200D Triple Axis Gyro Angular Velocity Sensor Module | We use a gyro angular velocity sensor to determine our vehicle's angular direction and accurately measure the change in angle per turn. To get accurate angle measurements from the gyro, we integrate the yaw angular velocity after calibrating the gyro to determine the robot's angle relative to its starting position.  | <img alt="image" src="https://github.com/user-attachments/assets/b95a0d49-5566-43d3-8427-d31013e839d7" /> | [Gyro Info](https://www.elecrow.com/download/L3G4200_AN3393.pdf) |
| PixyCam Pixy 2.1 Robot Vision Image Sensor | Since the Arduino Nano does not have enough computing power to process images, we are outsourcing the processing to the Pixy 2.1. This camera is able to provide the microcontroller with the necessary information to plan an appropriate path for the robot to take. | <img alt="image" src="https://github.com/user-attachments/assets/a4274ee3-ac2f-449d-aa8b-4e5ea3e3ded2" /> | [Pixy 2.1 Info](https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start) |

## Parts Bill Of Materials (BOM)
| Part Name | Quantity |
| ----------| -------- |
| 1.5V Rechargeable Lithium Ion Battery | 4 |
| DC 6V Micro Gear Box Speed Reduction Motor, 300RPM | 2 |
| CJMCU34725 TCS34725 RGB Light Color Sensor | 1 |
| Arduino Nano | 1 |
| ES08MA Metal Analog Servo | 1 |
| GP2Y0A02YK0F IR Range Sensor | 3 |
| GP2Y0A41SK0F IR Range Sensor | 2 |
| HW-201 IR Sensor | 1 |
| L298N Dual H-Bridge Motor Driver | 1 |
| L3G4200D Triple Axis Gyro Angular Velocity Sensor Module | 1 |
| PixyCam Pixy 2.1 Robot Vision Image Sensor | 1 |

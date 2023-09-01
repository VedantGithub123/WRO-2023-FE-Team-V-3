Electrical
====

This directory consists of schematic diagrams demonstrating the connections between different components as well as datasheets for the components. It also consists of the reasoning behind the choice of components.

## Parts Explanation

| Part Name | Explanation | Image | Datasheet |
| ----------- | ----------- | ----------- | ----------- |
| 1.5V Rechargeable Lithium Ion Battery | We are using this battery to power all of our electronics as it is compact and easy to get. They are also efficient for us as they are rechargeable and do not have to be replaced. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/139581102/0919130e-14e3-4f3f-b849-b53199521329) | [Battery Info](https://github.com/VedantGithub123/WRO-2023-FE/assets/139581102/3fdd861e-e929-4bd7-b8a7-6bc95675c747) |
| DC 6V Micro Gear Box Speed Reduction Motor, 300RPM | We are using this specific motor to drive the robot as it contains a gearbox that can reach a maximum speed of 300RPM. The compact size of these motors, coupled with a high RPM, allowed for our smaller compact design to succeed. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/139581102/96c5c912-6344-44ef-a4a4-8ac8d9f5ac15) | [DC Motor Info](https://github.com/VedantGithub123/WRO-2023-FE/assets/139581102/1d758aaa-bec6-41d7-8299-02d6930ba4b8) |
| CJMCU34725 TCS34725 RGB Light Color Sensor | Using this RGB color sensor allows our robot to sense the blue and orange lines at the corners of the mat. This allows us to keep track of how many laps the robot has completed and indicate when the robot needs to turn. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/11de23ca-59b6-454f-bb24-97d71231661f) | [Color Sensor Info](https://www.waveshare.com/w/upload/b/bb/TCS34725_Color_Sensor_user_manual_en.pdf) |
| Arduino Nano | In order to process the information from the sensors and make decisions based on the running code, we are using the Arduino Nano, which is programmed with the Arduino IDE. This board is easy to use, and there are many libraries available and on demand. The board contains 14 digital input/output pins, enabling us to connect a multitude of devices. It also has eight analog pins, of which two are used for I2C communication. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/139581102/8b7d66f1-2f6f-426f-8819-cc5c2578c9bd)| [Arduino Nano Info](https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf) |
| ES08MA Metal Analog Servo | In order to steer our vehicle, we use a servo motor with 2.0 Kilogram-force centimetres of torque and a speed of 0.10 sec/60°. Since steering will require a reasonably large amount of torque, this motor provides more than enough for our application. Furthermore, this servo motor can turn quickly, which allows us to make incredibly sharp turns. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/c786181c-acbb-454b-9b61-efc980529729) | [Servo Info](https://m.media-amazon.com/images/I/61ZU3A84tYS._AC_SL1000_.jpg) |
| GP2Y0A02YK0F IR Range Sensor | The IR sensors provide the necessary information on the distance to the walls at an extremely fast rate. These specific sensors sense distance accurately from 20 to 150cm. We use one of these sensors to detect how close we are to any wall in front of our robot. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/b6f9fde8-1bf3-4b70-9356-bc7b88c42bec) | [GP2Y0A02YK0F IR Range Sensor Info](https://cdn.robotshop.com/media/d/dem/rb-dem-02/pdf/datasheet-gp2y0a02yk0f.pdf) |
| GP2Y0A41SK0F IR Range Sensor | We use these sensors to measure the distance to any walls on the sides of our robot. Since we need sensors with a smaller range to detect when we are getting close to the wall, we are using these ones. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/4b96375d-76f8-457a-a853-81a0a6c2b61f) | [GP2Y0A41SK0F IR Sensor Info](https://www.pololu.com/file/0J713/GP2Y0A41SK0F.pdf) |
| HW-201 IR Sensor | In order to detect if our robot is going to crash into any object in the front, we use these sensors as a fail-safe to make sure our robot will still work. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/139581102/d9d72ff4-c947-445f-bc9c-464090f76cd6) | [HW-201 IR Sensor Info](https://www.circuits-diy.com/hw201-infrared-ir-sensor-module/) |
| L298N Dual H-Bridge Motor Driver | To effectively power our motors, we are using the L298N motor driver, where we are able to control the direction and speed of both motors using Pulse Width Modulation (PWM). | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/bab43380-a87a-4e54-83f2-e24c08ec1acd) | [Motor Driver Info](http://www.handsontec.com/dataspecs/L298N%20Motor%20Driver.pdf) |
| L3G4200D Triple Axis Gyro Angular Velocity Sensor Module | We use a gyro angular velocity sensor to determine our vehicle's angular direction and accurately measure the change in angle per turn. To get accurate angle measurements from the gyro, we integrate the yaw angular velocity after calibrating the gyro to determine the robot's angle relative to its starting position.  | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/1ede8942-381b-404f-8901-00fa2bd3f43b) | [Gyro Info](https://www.elecrow.com/download/L3G4200_AN3393.pdf) |
| PixyCam Pixy 2.1 Robot Vision Image Sensor | Since the Arduino Nano does not have enough computing power to process images, we are outsourcing the processing to the Pixy 2.1. This camera is able to provide the microcontroller with the necessary information to plan an appropriate path for the robot to take. | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/d2c488d6-f942-47e1-92da-26b894877c1c) | [Pixy 2.1 Info](https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start) |

## Parts Bill Of Materials (BOM)
| Part Name | Quantity |
| ----------| -------- |
| 1.5V Rechargeable Lithium Ion Battery | 4 |
| DC 6V Micro Gear Box Speed Reduction Motor, 300RPM | 2 |
| CJMCU34725 TCS34725 RGB Light Color Sensor | 1 |
| Arduino Nano | 1 |
| ES08MA Metal Analog Servo | 1 |
| GP2Y0A02YK0F IR Range Sensor | 1 |
| GP2Y0A41SK0F IR Range Sensor | 2 |
| HW-201 IR Sensor | 1 |
| L298N Dual H-Bridge Motor Driver | 1 |
| L3G4200D Triple Axis Gyro Angular Velocity Sensor Module | 1 |
| PixyCam Pixy 2.1 Robot Vision Image Sensor | 1 |

Electrical
====

This directory consists of schematic diagrams demonstrating the connections between different components as well as datasheets for the components. It also consists of the reasoning behaind the choice of components.

## Parts Explanation

| Part Name | Explanation | Image | Datasheet |
| ----------- | ----------- | ----------- | ----------- |
| 12V 3000mAh Lithium-ion Battery | We are using this battery to power all of our electronics since it can provide differnt voltages based on which terminals you connect to | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/0284d87a-9817-40a4-b8a0-7336db2fe3f1) | [Battery Info](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/5f33a5b1-f7bc-4229-8f9a-2fd0eec7c836) |
| 22mm Planetary Gear DC Motor w/ 12V Encoder, 185RPM | We are using this motor to move the robot since it has a gearbox to keep a reasonable speed as well as having an encoder | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/b1ea0b79-4e47-4835-a2d0-d0387937bbf2) | [DC Motor Info](https://cdn.robotshop.com/rbm/a00a7635-653b-4220-aac9-b0c23c5c5e2c/1/1031a402-eb67-4102-8300-9d3d54739abd/3eac6dd4_22pg-2230-en-motor.pdf) |
| CJMCU34725 TCS34725 RGB Light Color Sensor | In order to detect the lines at the corners of the mat, we are using a color sensor so we know when the robot is on the lines | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/11de23ca-59b6-454f-bb24-97d71231661f) | [Color Sensor Info](https://www.waveshare.com/w/upload/b/bb/TCS34725_Color_Sensor_user_manual_en.pdf) |
| ELEGOO UNO R3 | In order to process the information from the sensors and make decisions, we are using the ELEGOO UNO R3 which is programmed with the Arduino IDE | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/e8186827-057a-4357-8644-723c58d3d15b) | [ELEGOO UNO R3 Info](https://epow0.org/~amki/car_kit/Datasheet/ELEGOO%20UNO%20R3%20Board.pdf) |
| ES08MA Metal Analog Servo | In order to steer our vehicle, we decided to use a servo motor since it has torque which is required for this application | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/c786181c-acbb-454b-9b61-efc980529729) | [Servo Info](https://m.media-amazon.com/images/I/61ZU3A84tYS._AC_SL1000_.jpg) |
| GP2Y0A02YK0F IR Range Sensor | Since we need to know the distance to the walls, we need distance sensors which are able to sense for a large distance which is why we decided to use these ones which can sense up to 150cm | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/b6f9fde8-1bf3-4b70-9356-bc7b88c42bec) | [IR Sensor Info](https://cdn.robotshop.com/media/d/dem/rb-dem-02/pdf/datasheet-gp2y0a02yk0f.pdf) |
| L298N Dual H-Bridge Motor Driver | To effectively power our motors, we are using the L298N motor driver where we are able to control the direction and speed of both motors using Pulse Width Modulation (PWM). We also use the 5V output to power our servo motor since the output may vary and our servo has a large input voltage range | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/bab43380-a87a-4e54-83f2-e24c08ec1acd) | [Motor Driver Info](http://www.handsontec.com/dataspecs/L298N%20Motor%20Driver.pdf) |
| L3G4200D Triple Axis Gyro Angular Velocity Sensor Module | We are using an gyro to know which direction we are going in and how to change the vehicle's movement to reach the desired angle | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/1ede8942-381b-404f-8901-00fa2bd3f43b) | [Gyro Info](https://www.elecrow.com/download/L3G4200_AN3393.pdf) |
| L7805 Voltage Regulator | Our battery voltage is 12V so we need a voltage regulator to consistently keep the voltage at 5V to power the ELEGOO UNO R3 and the IR sensors | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/d3883378-43b8-4f2f-a01d-a3641c594a4e) | [Voltage Regulator Info](https://www.st.com/resource/en/datasheet/l78.pdf) |
| PixyCam Pixy 2.1 Robot Vision Image Sensor | We are using the Pixy 2.1 with the ELEGOO UNO R3 since it is able to identify multiple objects of multiple signatures which will be useful for our robot to plan it's route | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/d2c488d6-f942-47e1-92da-26b894877c1c) | [Pixy 2.1 Info](https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start) |

## Parts Bill Of Materials (BOM)
| Part Name | Quantity |
| ----------| -------- |
| 12V 3000mAh Lithium-ion Battery | 1 |
| 22mm Planetary Gear DC Motor w/ 12V Encoder, 185RPM | 2 |
| CJMCU34725 TCS34725 RGB Light Color Sensor | 1 |
| ELEGOO UNO R3 | 1 |
| ES08MA Metal Analog Servo | 1 |
| GP2Y0A02YK0F IR Range Sensor | 5 |
| L298N Dual H-Bridge Motor Driver | 1 |
| L3G4200D Triple Axis Gyro Angular Velocity Sensor Module | 1 |
| L7805 Voltage Regulator | 1 |
| PixyCam Pixy 2.1 Robot Vision Image Sensor | 1 |
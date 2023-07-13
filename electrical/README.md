Electrical
====

This directory consists of schematic diagrams demonstrating the connections between different components as well as datasheets for the components. It also consists of the reasoning behaind the choice of components.

## Parts Explanation

| Part Name | Explanation | Image | Datasheet |
| ----------- | ----------- | ----------- | ----------- |
| ES08MA Metal Analog Servo | In order to steer our vehicle, we decided to use a servo motor since it has torque which is required for this application | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/c786181c-acbb-454b-9b61-efc980529729) | [Servo Info](https://m.media-amazon.com/images/I/61ZU3A84tYS._AC_SL1000_.jpg) |
| GP2Y0A02YK0F IR Range Sensor | Since we need to know the distance to the walls, we need distance sensors which are able to sense for a large distance which is why we decided to use these ones which can sense up to 150cm | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/b6f9fde8-1bf3-4b70-9356-bc7b88c42bec) | [IR Sensor Info](https://cdn.robotshop.com/media/d/dem/rb-dem-02/pdf/datasheet-gp2y0a02yk0f.pdf) |
| 12V 3000mAh Lithium-ion Battery | We are using this battery to power all of our electronics since it can provide differnt voltages based on which terminals you connect to | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/0284d87a-9817-40a4-b8a0-7336db2fe3f1) | [Battery Info](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/5f33a5b1-f7bc-4229-8f9a-2fd0eec7c836) |
| ADXL345 3 axis Digital Acceleration of Gravity tilt Module | We are using an IMU to know which direction we are going in and how to change the vehicle's movement to reach the desired angle | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/227d6a4b-fab0-4d91-902d-5878aaf5c588) | [IMU Info](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf) |
| CJMCU34725 TCS34725 RGB Light Color Sensor | In order to detect the lines at the corners of the mat, we are using a color sensor so we know when the robot is on the lines | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/11de23ca-59b6-454f-bb24-97d71231661f) | [Color Sensor Info](https://www.waveshare.com/w/upload/b/bb/TCS34725_Color_Sensor_user_manual_en.pdf) |
| 22mm Planetary Gear DC Motor w/ 12V Encoder, 185RPM | We are using this motor to move the robot since it has a gearbox to keep a reasonable speed as well as having an encoder | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/b1ea0b79-4e47-4835-a2d0-d0387937bbf2) | [DC Motor Info](https://cdn.robotshop.com/rbm/a00a7635-653b-4220-aac9-b0c23c5c5e2c/1/1031a402-eb67-4102-8300-9d3d54739abd/3eac6dd4_22pg-2230-en-motor.pdf) |
| Raspberry Pi 3 B+ | We are using the raspberry pi to make the decisions based on the input from the sensors | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/d682e5cb-1d2b-4169-ac11-fc0122c03fe2) | [Raspberry Pi Info](https://datasheets.raspberrypi.com/rpi3/raspberry-pi-3-b-plus-product-brief.pdf?_gl=1*10zyxwh*_ga*MTI5MzQxMDYzNi4xNjg2NTk5MzYx*_ga_22FD70LWDS*MTY4OTI2NDAyNy42LjEuMTY4OTI2NDA4My4wLjAuMA..) |

## Parts Bill Of Materials (BOM)
| Part Name | Quantity |
| ----------| -------- |
| ES08MA Metal Analog Servo | 1 |
| GP2Y0A02YK0F IR Range Sensor | 5 |
| 12V 3000mAh Lithium-ion Battery | 1 |
| ADXL345 3 axis Digital Acceleration of Gravity tilt Module | 1 |
| CJMCU34725 TCS34725 RGB Light Color Sensor | 1 |
| 22mm Planetary Gear DC Motor w/ 12V Encoder, 185RPM | 2 |
| Raspberry Pi 3 B+ | 1 |
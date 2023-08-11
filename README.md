WRO Future Engineers Team V^3 Engineering Documentation
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the 2023 season. More information on each subcomponent of the robot is placed in the README.md files in each folder.

## Content
* `build` contains documentation about our chassis and material choice
* `electrical` contains schematic diagrams demonstrating the connections between different electromechanical components. It also includes the available datasheets for the aforementioned components and it also has a markdown file explaining the reasoning for each component.
* `models` contains the files used by 3D printers to produce the vehicle elements.
* `photos` contains two folders where one contains the team photos and the other contains the robot photos
* `src` contains code of control software for all components which were programmed to participate in the competition
* `strategy` contains documentation and diagrams explaining our approach to the problem
* `video` contains the video.md file with the link to a video where driving demonstration exists

## Brief Solution Description
After multiple iterations, we learned many lessons about what our robot should accomplish. As a result, we came upon our final design choice as shown in the image below:
![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/5507bb85-d2db-4309-833f-36882c81d8dc)
This robot is a four-wheeled vehicle which consists of a rear-wheeled drive and a four-bar steering mechanism at the front of the chassis. We have designed our robot to be as maneuverable as possible by decreasing the maximum turning radius and size of our robot which allows us to effortlessly avoid obstacles. The brains of our vehicle is the Arduino Nano which gets input from multiple compatible sensors such as:
- Pixycam 2.1
- TCS34725 RGB Sensor
- HW-201 IR Sensors
- GP2Y0A02YK0F IR Range Sensor
- L3G4200D 3-Axis Gyro
More information on the electrical components of our robot can be found in the electrical folder.
To power our driving wheels we use two 6V motors which are controlled by the L298N motor controller and our steering mechanism is controlled by a ES08MA metal gear micro servo.
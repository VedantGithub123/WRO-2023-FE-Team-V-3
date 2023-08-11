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
After going through multiple iterations and prototypes, we learned various lessons and redefined what our robot should be able to accomplish. Our goals were to (add goals under):

We originally had designed a larger robot with the usage of many more sensors and parts. However, its large nature caused it to be difficult to maneuver and control. All of our efforts were in vain as we faced many hardships and problems when trying to program that robot. Our robot’s turning radius was too large and was unable to maneuver around the obstacles while our sensors were placed inconveniently and were unable to correctly and quickly construct an efficient path for our robot to take. We realized that many parts of our robot were unnecessary, especially the sheer size of it, and we ultimately decided to do a complete redesign. This newer robot is much smaller and more compact allowing it to be agile and easier to program. We have decreased the maximum turning radius by reducing the robot’s size enabling it to effortlessly avoid obstacles and choose the quickest path to complete the challenge. This robot is a four-wheeled vehicle that consists of a real-wheeled drive and a four-bar steering mechanism at the front of the chassis. The chassis and frame is almost entirely made of LEGO Technic parts and any remaining additions and part adapters (i.e., motor adapters, sensors, etc.) were designed on the SolidWorks CAD Software and were 3D printed. The main controller of our vehicle is the Arduino Nano board. We chose this new board instead of the ELEGOO UNO R3 that our old robot was using as it provided a similar number of ports and functionality while having a small and compact structure that could fit on the robot. The Nano board gets input from multiple compatible and reliable sensors such as:
- Pixycam 2.1
- TCS34725 RGB Sensor
- HW-201 IR Sensors
- GP2Y0A02YK0F IR Range Sensor
- L3G4200D 3-Axis Gyro
More information on the electrical components of our robot can be found in the electrical folder.

To power our driving wheels we use two 6V motors which are controlled by the L298N motor controller. Our steering mechanism is controlled by a ES08MA metal gear micro servo. All in all, this robot is a statement to our team’s adaptability and problem solving skills as we were able to completely redesign and program a new and more compact robot after learning from our previous mistakes. We hope that this robot and our hard work will lead us to success and we look forward to competing!

Our current robot is pictured below:
![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/5507bb85-d2db-4309-833f-36882c81d8dc)
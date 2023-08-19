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

## Rubric Requirements
* `Mobility Management` is found in `build`, `electrical`, and `models`
* `Power and Sense Managament` is found in `electrical`
* `Obstacle Management` is found in `strategy` and `src`
* `Pictures - Team and Vehicle` is found in `photos`
* `Performance Videos` is found in `video`
* `Engineering Factor` is found in `build` and `README`


### Who We Are
Team V^3 is an aspiring group of high school and university students who aim to be leaders in the future of autonomous vehicles. By taking the initiative to take part in the WRO Future Engineers challenge, we hope to gain experience in this field of engineering and problem-solving. We have been exposed to many robotic and logical challenges through Zebra Robotics, where we participated in many problems, such as WRO Robomissions, First LEGO League, and VEX Robotics Competition. Our goals for this season are the following:
- Gain knowledge in the field of autonomous vehicles
- Expose ourselves to the nuances of software organization (i.e. Github)
- Become proficient in the skills regarding electronic components
- Learn how to coordinate and work as a team to solve the problem with innovation and creativity

### Our Robot & Iterations
Our team has designed a four-wheel rear-driver autonomous vehicle to complete the 2023 WRO Future Engineers challenge. When making our robot, we wanted to be able to rapidly iterate and make changes whenever they were needed. To accomplish this, our chassis and frame are almost entirely made of LEGO Technic parts and any remaining additions and part adapters (i.e., motor adapters, sensors, etc.) were designed on SolidWorks and Fusion 360 CAD Software and were printed using PLA filament on Anycubic Kobra 3D printers. Our current robot is a small and compact vehicle causing it to be highly maneuverable, allowing us to avoid obstacles effortlessly. This vehicle is powered by 300rpm 6V Micro Gear Box DC motors, allowing us to keep a compact and lightweight robot. To steer this chassis, we are using the ES08MA Metal Analog Servo, which has an ample amount of torque and strength. This allows us to make sharp turns whenever needed.
Prior to making this robot, our team went through various different iterations and prototypes where we learnt many lessons about what our optimal robot should consist of. These lessons are the following:
- Be small and compact
- Effectively use sensors to identify our location and our ideal path
- Speed is not the first priority for this challenges
- Maneuverability is key to solving the obstacle challenge

One of these was a larger robot with the usage of many more sensors and parts. However, its large nature at 30x20x30cm caused it to be difficult to maneuver and control. All of our efforts were in vain as we faced many hardships and problems when trying to program that robot. Our robot’s turning radius was too large, and it was unable to maneuver around the obstacles while our sensors were placed inconveniently and were unable to correctly and quickly construct an efficient path for our robot to take. We realized that many parts of our robot were unnecessary, especially the sheer size of it, and we ultimately decided to do a complete redesign which led to our final robot design. A summary of the reasons for our redesign can be found below:

|   | Pros | Cons |
| - | ----------| ----------|
| Old Robot | <ul><li>Allows for the attachment of larger sensors</li><li>Faster robot due to the larger, more powerful motors</li></ul> | <ul><li>Bad maneuverability due to size, causing a larger turning radius</li><li>Excessive use of unnecessary sensors</li><li>Not enough ports due to the sheer number of sensors</li></ul> |
| New Robot | <ul><li>Smaller and more compact</li><li>Better maneuverability and sharper turning radius</li><li>More effective use of sensors and gets rid of any unnecessary parts</li></ul> | <ul><li>Slower speed due to less powerful motors</li><li>Restrictions to the size of sensors and parts we can use</li></ul> |

| Old Robot | New Robot |
| --------- | --------- |
| <img src="https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/857241b9-94ba-453f-89ac-cdbb4481c77c" width="300" height="200" /> | !<img src="https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/5507bb85-d2db-4309-833f-36882c81d8dc" width="300" height="200" /> |

### Electrical Components
Our chassis is controlled by the Arduino Nano microcontroller, which gets information from various sensors and uses said information to control the motors using an L298N motor controller board. The sensors that we are using for our vehicle are the following:
- Pixycam 2.1
- TCS34725 RGB Sensor

- HW-201 IR Sensors
- GP2Y0A02YK0F IR Range Sensor
- L3G4200D 3-Axis Gyro

We use the Pixycam 2.1 to identify the locations of the obstacles and effectively avoid them. The TCS34725 RGB sensor tells the microcontroller when it reaches a corner. Both IR sensors are used to detect the walls on the side and on the front. Finally, we use the L3G4200D gyro to know which direction we are facing. More information about our robot’s electrical components can be found in the README file in the electrical folder.

### Strategy and Code
For the open challenge, our sensors are strategically placed in ways which allow us to detect walls at a 45° or less angle. This allows us to adjust accurately based on the distance from the wall, ultimately providing an optimal path to solve this challenge. In order to measure the number of laps we do, we use the RGB sensor to count the number of lines passed and then our robot stops after some time once 12 lines have been measured. This is implemented in the code by using the following lines:

if (cornerCount==12)

{

  endTime = millis()+5000;
  
}

The cornerCount variable keeps track of how many corners the robot has passed during its run. The number is twelve as the square-shaped mat has 4 corners, and the robot must complete three laps.
We have two strategies for the obstacle challenge. The first of which only focuses on one block and a time and avoids it. The other strategy involves planning a route for each stretch of the lap and getting the robot to follow that path. Some code for the first challenge is the following:

if (closeBlock.m_signature==1)

{

target = (207-closeBlock.m_y)/1.3-15;

}else

{

target = 315.0-(207-closeBlock.m_y)/1.3+15;

}

err = -150.0*(closeBlock.m_x-target);

This code gets the closest block and sets the target position of that block onto either side of the robot based on the color of the block.

All in all, this robot is a statement to our team’s adaptability and problem solving skills as we were able to completely redesign and program a new and more compact robot after learning from our previous mistakes. We hope that this robot and our hard work will lead us to success and we look forward to competing!


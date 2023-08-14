Build
====

This directory consists of the different materials used to construct our robot along with the reasoning for choosing our drivetrain.

## Chassis Type & Explanation

Our robot is a four-wheeled vehicle which has a rear-wheel drive with steering in the front. We decided to implement a simple chassis since that would reduce the amount of complexity, and therefore chances of failure. In order to keep the drivetrain simple and efficient, we made sure not to have the steering mechanism on the same set of wheels as the driving wheels. This solves the problem of spacing as the steering motor and the drive motors do not have to be on the same set of wheels. Our decision to do this meant that we could only have two options for driving which were front-wheel drive with steering in the back or a rear-wheel drive with steering in the front. We ultimately chose a drivetrain that had real-wheel drive with front-wheel steering as that made the robot more familar to use, given our experience using other real-wheel driven LEGO robots.

| Rear-Wheel Drive | Front Wheel Drive |
| ---------------- | ----------------- |
| ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/a657e21e-20b9-46eb-b555-2f60db8776fa) | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/1d374418-1677-495a-9adb-77a0abb5c40a) |

## Steering

Our steering mechanism is a four-bar with free spinning wheels mounted on it. The motor will move the four-bar which changes the angle of the wheels, which in turn changes the direction that the vehicle is travelling in. This type of steering proved to be the most efficient for our purposes and was also the easiest version for us to implement as it only required a servo motor and a simple rig that connected the servo to the wheel axles. The rig was 3D printed along with the servo mount as one piece and had a LEGO axle-shaped cross hole that would turn along with the servo.

| Four-Bar Steering Mechanism |
| --------------------------- |
| ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/43159301-d6ed-4b82-bd15-9d167c74ce55) |

## Materials

When creating our robot, we wanted to be able to easily iterate and update our robot in many ways as we tested it. In order to achieve these goals, we decided to use LEGO Technic pieces to construct the majority of our robot. LEGO technic was the perfect choice for us since our robotics centre, Zebra Robotics, focuses on LEGO robotics and competitions related to that so we have an abundance of LEGO technic pieces. Our team members also have extensive experience in working with LEGO Technic parts and completing challenges with them. In order to attach the electronic components onto our chassis, we 3D printed mounts for the components, as they allowed us to smoothly integrate the non-LEGO parts with the LEGO parts.
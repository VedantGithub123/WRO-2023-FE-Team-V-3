Strategy
====

This directory consists of an explanation and diagrams of our strategy for both the open and obstacle rounds.

## Open Challenge Strategy

In the open challenge, the only changes to the field are the size of the interior walls. In order to combat these changes, we will be using IR distance sensors to identify the distance from the walls. By having multiple distance sensors, the robot can accurately caculate the distance to the surrounding walls. The robot can use these sensor readings to correct itself and make sure it goes parallel to the wall. The robot will know when to turn due to the usage of a RGB sensor to detect the colored lines in the corner. This will also allow us to keep track of the number of laps we complete so we know when to end. Below are diagrams and flow charts demonstrating the process.
| Straight Movement | Turning |
| ----------------- | ------- |
| ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/53158008-50e9-43d2-a8da-e41560a97f00) | ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/22a1995e-1c62-4540-9e58-ad45c3a5c97a) |

| Open Challenge Flowchart |
| ------------------------ |
| ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/2448b1ec-511b-4f77-8bc3-528df8e6e0b6) |

## Obstacle Challenge Strategy

For the obstacle challenge, we have two strategies to approaching the problem. The first one is to only consider the first obstacle and act according to that while the other is to plan a path for one side of the round and follow that. We are working with both strategies and testing both simultaneously to determine the better strategy. To detect the obstacles, we are using the Pixycam 2.1 which gives a list of objects to the Arduino Nano.

### Strategy 1

The first strategy to approach the obstacle challenge is to keep the closest color on the corresponding side of the robot. This is implemented by getting the x-position of the object on the camera and accordingly changing the steering to make sure the object is on the correct side. The turning and stopping will use the same logic as for the open challenge. In order to know how much to steer when we see the object, we will use a PID control loop to ensure that our movements are efficient.

| Obstacle Challenge Simple Approach |
| ---------------------------------- |
| ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/a151bb18-8517-4f28-9d21-015ce52a3ff2) |

### Strategy 2

Our second strategy for the obstacle challenge is to plan a path for each stretch of the mat and get the robot to follow that path. By having the current position, current heading, target position, and target heading, we can generate a polynomial to fit these paramenters. We do this with multiple target points to create a target path which we aim to follow and steer the robot so it aims to move in the direction of the next point. This path taks into account the position of the obstacles and maps points which avoid them. We then repeat this process until 3 rounds are over. 

| Obstacle Challenge Complicated Approach |
| ---------------------------------- |
| ![image](https://github.com/VedantGithub123/WRO-2023-FE/assets/112735969/36173d42-eb99-45da-80e0-ce6af80726a3) |
Strategy
====

This directory consists of an explanation and diagrams of our strategy for both the open and obstacle rounds.

## Open Challenge Strategy

In the open challenge, the only changes to the field are the size of the interior walls. In order to combat these changes, we will be using IR distance sensors to identify the distance from the walls. By having multiple distance sensors, the robot can accurately calculate the distance to the surrounding walls. The robot can use these sensor readings to correct itself and make sure it goes parallel to the wall. The robot will know when to turn due to the usage of a RGB sensor which detects the colored lines in the corner. This will also allow us to keep track of the number of laps we complete so we know when to end. Below are diagrams and flow charts demonstrating the process.
| Straight Movement | Turning |
| ----------------- | ------- |
| ![image](https://drive.google.com/uc?id=1sl-HCauvqxThJZm0eeGbrZbrhHrH50nJ) | ![image](https://drive.google.com/uc?id=1nIDDyOZn28JYcaWRNiBgwWf6LNt0vX17) |

| Open Challenge Flowchart |
| ------------------------ |
| ![image](https://drive.google.com/uc?id=1_4qiJSqqLnAqw7ilf54cnIb5HE8X9X17) |

## Obstacle Challenge Strategy

For the obstacle challenge, we have two strategies for approaching the problem. The first one is to only consider the first obstacle and act according to that, while the other is to plan a path for one side of the round and follow that. We are working with both strategies and testing both simultaneously to determine the better strategy. To detect the obstacles, we are using the Pixycam 2.1, which gives a list of objects to the Arduino Nano.

### Strategy 1

The first strategy to approach the obstacle challenge is to keep the closest color on the corresponding side of the robot. This is implemented by getting the x-position of the object on the camera and accordingly changing the steering to make sure the object is on the correct side. The turning and stopping will use the same logic as the open challenge. In order to know how much to steer when we see the object, we will use a proportional control loop to ensure that our movements are efficient.

| Obstacle Challenge Simple Approach |
| ---------------------------------- |
| ![image](https://drive.google.com/uc?id=18gsX1JlRYennzsx7GrNMAYbRCy7gM8yc) |

### Strategy 2

Our second strategy for the obstacle challenge is to plan a path for each stretch of the mat and get the robot to follow that path. By having the current position, current heading, target position, and target heading, we can generate a polynomial to fit these parameters. We do this with multiple target points to create a target path which we aim to follow and steer the robot so it aims to move in the direction of the next point. This path takes into account the position of the obstacles and maps points that avoid them. We then repeat this process until the three rounds are over. 

| Obstacle Challenge Complicated Approach |
| ---------------------------------- |
| ![image](https://drive.google.com/uc?id=1dFHU8QVgc9cSyHeG_vhY4EH8VAXhCVJk) |

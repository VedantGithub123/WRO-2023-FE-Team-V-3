Strategy
====

This directory consists of an explanation and diagrams of our strategy for both the open and obstacle rounds.

## Open Challenge Strategy

In the open challenge, the only changes to the field are the size of the interior walls. In order to combat these changes, we will be using IR distance sensors to identify the distance from the walls. By having multiple distance sensors, the robot can accurately calculate the distance to the surrounding walls. The robot can use these sensor readings to correct itself and make sure it goes parallel to the wall. The robot will know when to turn due to the usage of a RGB sensor which detects the colored lines in the corner. This will also allow us to keep track of the number of laps we complete so we know when to end. Below are diagrams and flow charts demonstrating the process.
| Straight Movement | Turning |
| ----------------- | ------- |
| <img alt="image" src="https://github.com/user-attachments/assets/995357b4-ba19-4a08-b12c-2040aa15ce63" /> | <img alt="image" src="https://github.com/user-attachments/assets/14e4b802-bea2-4fc7-9a74-8db0a99509a8" /> |

| Open Challenge Flowchart |
| ------------------------ |
| <img alt="image" src="https://github.com/user-attachments/assets/6486ae48-f1fe-42d2-b6af-0d932f899437" /> |

## Obstacle Challenge Strategy

For the obstacle challenge, we have two strategies for approaching the problem. The first one is to only consider the first obstacle and act according to that, while the other is to plan a path for one side of the round and follow that. We are working with both strategies and testing both simultaneously to determine the better strategy. To detect the obstacles, we are using the Pixycam 2.1, which gives a list of objects to the Arduino Nano.

### Strategy 1

The first strategy to approach the obstacle challenge is to keep the closest color on the corresponding side of the robot. This is implemented by getting the x-position of the object on the camera and accordingly changing the steering to make sure the object is on the correct side. The turning and stopping will use the same logic as the open challenge. In order to know how much to steer when we see the object, we will use a proportional control loop to ensure that our movements are efficient.

| Obstacle Challenge Simple Approach |
| ---------------------------------- |
| <img alt="image" src="https://github.com/user-attachments/assets/e24923ab-84af-4925-8dfa-6679ffdb0378" /> |

### Strategy 2

Our second strategy for the obstacle challenge is to plan a path for each stretch of the mat and get the robot to follow that path. By having the current position, current heading, target position, and target heading, we can generate a polynomial to fit these parameters. We do this with multiple target points to create a target path which we aim to follow and steer the robot so it aims to move in the direction of the next point. This path takes into account the position of the obstacles and maps points that avoid them. We then repeat this process until the three rounds are over. 

| Obstacle Challenge Complicated Approach |
| ---------------------------------- |
| <img alt="image" src="https://github.com/user-attachments/assets/df1b6b94-bf40-42fd-aa53-40bd621ac84a" /> |

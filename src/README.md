Control software
====

This directory contains the code required for our robot and explains it

## Software and Language
To program our robot, we use the Arduino IDE and code an Arduino Sketch based on C++. Compiling and uploading our code is a simple process, as there are two buttons to help verify and upload our code. The click of this button will automatically compile and upload our code to the Arduino Nano microcontroller.

## Organization of Code
Our code follows Object-Oriented Programming (OOP) conventions, where we utilize classes for each and every one of our robot's sensors and components. The libraries that we are using to interface with our electronic components are:
Servo.h (For controlling the servo motor)
Adafruit_TCS34725.h (For reading the RGB sensor)
Adafruit_Sensor.h (For reading the gyro sensor)
Wire.h (For reading the gyro sensor)
Pixy2.h (For reading the Pixy camera)
### Chassis Class
In the chassis class, five-member variables are utilized to control the driving motors and the steering. These variables are "enA", which is used to store the pin that is connected to the enable port on the motor controller, and "in1" and "in2", which are used to store the pins connected to the inputs on the motor controller. We also use "sPort," which holds the pin connected to the micro servo. Finally, we have a steering variable, an instance of the steering class imported from the Servo.h library, which is used to control the micro servo we are using.

The constructor for this class takes in four inputs which are the ports used for the chassis, as mentioned above. These inputs are then stored in the member variables for later use in the class instance. 

The first method in our class is the move function which takes in an integer speed and limits it from -255 to 255. It then goes through some logic determining which inputs to turn on or off to drive in the right direction. In the following code, we check if the speed is zero, positive, or negative and set the input pins accordingly.
`if (speed == 0) { digitalWrite(in1, LOW); digitalWrite(in2, LOW); } else if (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); } else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }`

The second method in our class is the steer function which steers our servo motor to the right angle. This method limits an angle variable to a specific range and then writes that range to the servo motor's input pin. The following code limits the range to -70 to 70, adding 94 as an offset.
`steering.write(angle / abs(angle) * min(abs(angle), 70) + 94);`

The third and final method in our class attaches the servo port to the instance of the Servo class defined in the chassis class. This is done by implementing the following code.
`steering.attach(sPort);`

### rgbSensor Class
The rgbSensor class interfaces with the TCS34725 RGB sensor using I2C (Inter-Integrated Circuit) communication to get the RGB values from the sensor. This class has one member variable, the "Adafruit_TCS34725 tcs", which controls the RGB sensor.

The first method in this class is the "getColor" function which returns the detected colour. The following code gets the RGB values and sets the corresponding variable to the values. `tcs.getRGB(&red, &green, &blue);` We then check the blue and red values to determine if the sensor is seeing blue, orange, or white in the following code. `if (blue > 90 && red < 60) { col = 2; } if (red > 80 && blue < 60) { col = 1; }`

The second method in this class is the "setup" function, which starts the TCS34725 sensor.

### IRSensors Class
This class is used to interface with the IR sensors that we are using. The member variables in this class are "int irPorts[4]" which is an array of four ports that hold the ports the sensors are connected to.

The constructor of this class takes in an array as input and assigns the values of the array to the irPorts array.

The first method of the class is the "getDistance" function which returns values based on which sensor it is using. If the sensor is the one in the front used for sensing long distances, we map the voltage to a function that returns the length. This function is implemented in the following code. `return (11.63417 + (241.6444 - 11.63417)/pow((1 + (analogRead(irPorts[port])*5.0/1023.0/0.5075672)), 1.868922))*1.5;` If the sensor is the one in the front and is used for short distances, we return one if it senses something and zero if not. Finally, if none of those are done, we return one if the sensor output voltage is larger than 0.93V and 0 if not.

The second method in this class is the "getDistanceClose" function which acts the same as the "getDistance" function but only returns one for the default case if the voltage is larger than 3.4V.

The third method in this class is the "getFarDistance" function which maps the voltage read from the IR sensor to a function to return the distance seen. This is done with the following code. `return ((138.672)/pow(1.0 + 5.56591*(analogRead(irPorts[port])/200.0), 1.24888)) + (-0.0340614 * pow(analogRead(irPorts[port])/200.0, 3));`

The final function in this class is the setup function which sets the pin modes for all the ports.

### Gyro Class
The Gyro class is used to interface with the L3G4200D gyro sensor through I2C communication. This class has four member variables which are the "angle", "prevTime", "drift", and "L3G4200D_Address".

The first method of the class is the "setup" function which writes to the gyro registers to set it up for use. This is done in the following code. `writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111); writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000); writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000); writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000); writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);`

The second method is the "writeRegister" function which utilizes the "Wire.h" library to write to a particular address.

The third method is the "readRegister" function which also utilizes the "Wire.h" library to read information from a particular address.

The fourth method is the "getGyroChange" function which gets the angular velocity of the gyro. This is done by using the "readRegister" function and shifting the bits to get the value as shown in the following code. ` return ((zMSB << 8) | zLSB);`

The fifth method in the Gyro class is the "getAngle" function which returns the robot's angle by returning the "angle" variable stored in the Gyro class.

The sixth method in the Gyro class is the calibrate function which takes 6000 sample inputs and finds the average. This is the drift of the gyro, which we store in the drift variable for later use. We do the calibration with the following code. `for (int i = 0; i < 6000; i++){ drift +=getGyroChange(); } drift /= 6000; prevTime = micros();`

The final function in the class is the "updateGyro" function which updates the angle of the gyro when called. It does this by multiplying the value of the gyro by the time since the last call and then by a constant and adding that to the angle variable. This is done in the following code. `angle += (micros() - prevTime) / 1000000.0 * (getGyroChange() - drift) / -14.286 * 9/8 * 90/95; prevTime = micros();`

### Camera Class
The Camera class interfaces with the PixyCam 2.1 through SPI (Serial Peripheral Interface) and gets the detected blocks. The member variable in this function is the "pixy" variable, an instance of the Pixy2 class provided by the "Pixy2.h" library. 

The first method in the class is the "setup" function which initializes the PixyCam 2.1. This is done by using the "pixy.init()" function provided in the library.

The second method in the Camera class is the "getClosest" function which returns the closest signature to the robot. The PixyCam 2.1 has the closest signature as the first element in the array of blocks, so we return that first signature as shown in the following code `pixy.ccc.getBlocks(); return pixy.ccc.blocks[0];`

The final method in the class is the "getClosestBlock" function which gets the closest signature which is green or red. This is done to ensure that we do not return the line. We do this by going through the array of blocks, and if the signature is less than or equal to two, we return that block.

## Setup Code
In the setup of our program, we have a multitude of global functions and variables that we use in our main program. 

Arduino requires us to have a `void setup()` function, which runs once when the Arduino is turned on and a `void loop` function, which runs repeatedly until the Arduino turns off. In our code, we keep the setup of all the sensors and then wait until the button is pressed in the `setup` function, and we use the `loop` function to run our code. The first function that we use in our code is the `void delay_2` function which takes in an integer value and delays for that amount of milliseconds while updating the gyro sensor. We use this function instead of the default `delay` function since it allows our gyro sensor to constantly update. We then have two separate functions for each challenge. The `void open` function contains the code for the open challenge, and the `void obstacle` contains the code for the obstacle challenge. We implement these functions in the `loop` function by commenting on which function we are not running and uncommenting the function that we are running.

The global variables that we have in our code are the following
| Variable Name | Type | Purpose |
| ------------- | ---- | ------- |
| chassis | Chassis | Creates an instance of the Chassis class for movement |
| rgbSense | rgbSensor | Creates an instance of the rgbSensor class for sensing lines |
| irPorts | int[4] | Stores the values of the IR sensors' ports |
| irSensors | IRSensors | Creates an instance of the IRSensors class to use the IR sensors |
| gyro | Gyro | Creates an instance of the Gyro class to measure the robot's angle |
| camera | Camera | Creates an instance of the Camera class to detect obstacles |
| buttonPort | int | Stores the port of the button |
| speed | int | Defines the speed for the robot and is used for acceleration |
| cornerCount | int | Stores the number of corners passed |
| endTime | unsigned long int | Stores the time when our robot should stop moving |
| dir | int | Stores the direction of the lap |
| cornerScanDelay | unsigned long int | Stores the time a corner was passed |
| cornerDetected | bool | Stores if a line has been detected |
| steer | float | Stores the steering value |
| prevObj | Block | Stores the previous block |
| closeBlock | Block | Stores the closest block |
| target | int | stores the target position for the blocks |
| err | float | Stores the error for following |
| kP | float | Stores the kP value for following |
| objDelay | int | Stores the time when an object is passed |
| targetAngle | int | Stores the target angle for gyro following |
| caughtOnWall | bool | Used to detect when to wall follow |

## Open Challenge Code
In the open challenge, we implement gyro and IR following, which uses readings from the sensors to ensure a straight path. Before we pass the first corner, the robot does not know which direction it is travelling in, so it gyro follows. We implement gyro following in the following code where it finds the difference between the target angle and the current angle and multiplies that error to get the steering. `err = targetAngle - gyro.getAngle(); steer = err*2.0;` To set our direction, we run this code constantly which checks if the direction is not set, it will reset it to whatever the RGB sensor is seeing. `if (dir==0) { dir = rgbSense.getColor(); }` The next part of our code checks if the RGB sensor sees the same color as the direction that the robot is travelling and it has been 2 seconds since the last line was seen and increments to cornerCount, sets the end time to after 3 seconds, flags the cornerDetected variable as true, changes targetAngle, and resets cornerScanDelay. We do this in the following code. `if (rgbSense.getColor()==dir && dir!=0 && millis()-cornerScanDelay>2000) { delay_2(60); cornerCount++; endTime = millis()+3000; cornerDetected = true; targetAngle += (dir == 1 ? 90 : -90) cornerScanDelay = millis(); }` We then have a code to check if the cornerDetected variable is true, the robot is close to the wall, and the robot is not wall following, and if all the conditions are true, the robot will take a turn based on which direction the robot is travelling. We then check if the robot is close to the walls, and if it is, we set the caughtOnWall variable to true so that the robot will start wall following. We then check if the robot is too close to the walls on the side and turn if so. We then check if we are supposed to wall follow, and wall follow if so. The wall following is done with the following code. `if(caughtOnWall) { if(dir == 1) { err = 20.0 - irSensors.getFarDistance(2); steer = err * -2.0; } else if(dir == 2)  { err = 20.0 - irSensors.getFarDistance(0); steer = err*2.0; } if (steer>15){ steer = 15; }else if (steer<-15){ steer = -15; } }` We then make sure that the steering is not greater than 30 degrees and we set the steering and speed if it is not time to stop the run.

## Obstacle Challenge Code
For the obstacle challenge, we use a variety of sensors, including the RGB sensor, IR sensors, gyro, and camera. Our code uses these sensors to avoid obstacles and turn effectively. This code is split into two main parts. The first part is the normal navigation, and the other part is the turning logic. In the normal navigation logic, we start off by checking if the time since the corner turned is less than 2 seconds and get the closest signature based on that. We then set the prevObj variable to the current block if the block is close enough to the robot in the following code. `if (closeBlock.m_y>150 && closeBlock.m_signature<=2) { prevObj = closeBlock; }` We then have the same direction detection logic as the open challenge. After that, we have the block following which we do if we see a block, not a line or nothing which is done in the following code. `if (closeBlock.m_signature<=2) {  if (closeBlock.m_signature==1) { target = (207-closeBlock.m_y)/1.5 + 5; }else { target = 310.0-(207-closeBlock.m_y)/1.5; } err = target - (int)closeBlock.m_x; steer = err*kP; if (steer>30){ steer = 30; }else if (steer<-30){ steer = -30; } }` If there is no block sensed, we run the wall following code which is done with the following code. `err = 60 - irSensors.getFarDistance(dir == 1 ? 0 : 2); steer = err * (dir == 1 ? 1.0 : -1.0) * 1.0; if (steer>10){ steer = 10; }else if (steer<-10){ steer = -10; }` We then have some code which makes sure we don't crash int anything on the side which is done by using the IR sensors to detect the distance from the sides and turn accordingly. We do this in the following code. `if (irSensors.getFarDistance(0) <= 15) { if(closeBlock.m_signature != 1) { steer = 20; } else if (irSensors.getFarDistance(0) <= 12)  steer = 30; }else if (irSensors.getFarDistance(2) <= 15) { if(closeBlock.m_signature != 2) { steer = -20; } else if (irSensors.getFarDistance(2) <= 12)  steer = -30; }` This code checks the side sensors and the current block that the robot is seeing to determine which direction and how much to turn. After this is done, we have an if statement to identify if our robot is going to crash from the front, and if so, the robot will move backwards for three seconds and resume the program. The last part of normal navigation is where the robot sets the steering and speed if the time is not up. For the turning logic, we have an if statement that checks if it is time to turn and runs the code accordingly. The first thing that happens in the if statement is that we increment to cornerCount and check if we need to set the stopping time. We then check if the last block passed was red, and the robot finished the second lap and ran the 180-degree turn where the robot uses the gyro to go close to the wall and turn. The gyro following is done in this code: `while (irSensors.getDistance(1)>50){ err = targetAngle + 170 - gyro.getAngle(); steer = err*2.0; if (steer>20){ steer = 20; }else if (steer<-20){ steer = -20; } delay_2(1); chassis.steer(steer); gyro.updateGyro(); }` and the turning amount changes based on the direction we are travelling in. If we are supposed to run a normal turn, we check if we need to turn on the outside, inside, or neither and run based on that. For all of these turning cases, we use the gyro sensor to turn. To determine which case to go into, the robot takes eight frames from the camera and checks if a block appears in at least 2 of those frames and based on the block's color, we determine which case to go into.

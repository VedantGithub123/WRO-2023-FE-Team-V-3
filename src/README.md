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
In the chassis class, five member variables are utilized to control the driving motors and the steering. These variables are "enA", which is used to store the pin that is connected to the enable port on the motor controller, and "in1" and "in2", which are used to store the pins connected to the inputs on the motor controller. We also use "sPort," which holds the pin connected to the micro servo. Finally, we have a steering variable, an instance of the steering class imported from the Servo.h library, which is used to control the micro servo we are using.

The constructor for this class takes in four inputs which are the ports used for the chassis, as mentioned above. These inputs are then stored in the member variables for later use in the class instance. 

The first method in our class is the move function which takes in an integer speed and limits it from -255 to 255. It then goes through some logic determining which inputs to turn on or off to drive in the right direction. In the following code, we check if the speed is zero, positive, or negative and sets the input pins accordingly.
`if (speed == 0) { digitalWrite(in1, LOW); digitalWrite(in2, LOW); } else if (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); } else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }`

The second method in our class is the steer function which steers our servo motor to the right angle. This method limits an angle variable to a specific range, then writes that range to the servo motor's input pin. The following code limits the range to -70 to 70, adding 94 as an offset.
`steering.write(angle / abs(angle) * min(abs(angle), 70) + 94);`

The third and final method in our class attaches the servo port to the instance of the Servo class defined in the chassis class. This is done by implementing the following code.
`steering.attach(sPort);`

### rgbSensor Class
The rgbSensor class interfaces with the TCS34725 RGB sensor using I2C (Inter-Integrated Circuit) communication to get the RGB values from the sensor. This class has one member variable, the "Adafruit_TCS34725 tcs", which controls the RGB sensor.

The first method in this class is the "getColor" function which returns the detected colour. The following code gets the RGB values and sets the corresponding variable to the values. `tcs.getRGB(&red, &green, &blue);` We then check the blue and red values to determine if the sensor is seeing blue, orange, or white in the following code. `if (blue > 90 && red < 60) { col = 2; } if (red > 80 && blue < 60) { col = 1; }`

The second method in this class is the "setup" function, which starts the TCS34725 sensor.

### IRSensors Class
This class is used to interface with the IR sensors that we are using. The member variables in this class are "int irPorts[4]" which is an array of four ports which hold the ports the sensors are connected to.

The constructor of this class takes in an array as input and assigns the values of the array to the irPorts array.

The first method of the class is the "getDistance" function which returns values based on which sensor it is using. If the sensor is the one in the front used for sensing long distances, we map the voltage to a function which returns the length. This function is implemented in the following code. `return (11.63417 + (241.6444 - 11.63417)/pow((1 + (analogRead(irPorts[port])*5.0/1023.0/0.5075672)), 1.868922))*1.5;` If the sensor is the one in the front and is used for short distances, we return one if it senses something and zero if not. Finally, if none of those are done, we return one if the sensor output voltage is larger than 0.93V and 0 if not.

The second method in this class is the "getDistanceClose" function which acts the same as the "getDistance" function but only returns one for the default case if the voltage is larger than 3.4V.

The final function in this class is the setup function which sets the pin modes for all the ports.

### Gyro Class
The Gyro class is used to interface with the L3G4200D gyro sensor through I2C communication. This class has four member variables which are the "angle", "prevTime", "drift", and "L3G4200D_Address".

The first method of the class is the "setup" function which writes to the gyro registers to set it up for use. This is done in the following code. `writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111); writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000); writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000); writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000); writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);`

The second method is the "writeRegister" function which utilizes the "Wire.h" library to write to a particular address.

The third method is the "readRegister" function which also utilizes the "Wire.h" library to read information from a particular address.

The fourth method is the "getGyroChange" function which gets the angular velocity of the gyro. This is done by using the "readRegister" function and shifting the bits to get the value as shown in the following code. ` return ((zMSB << 8) | zLSB);`

The fifth method in the Gyro class is the "getAngle" function which returns the robot's angle by returning the "angle" variable stored in the Gyro class.

The sixth method in the Gyro class is the calibrate function which takes 6000 sample inputs and finds the average. This is the drift of the gyro which we store in the drift variable for later use. We do the calibration with the following code. `for (int i = 0; i < 6000; i++){ drift +=getGyroChange(); } drift /= 6000; prevTime = micros();`

The final function in the class is the "updateGyro" function which updates the angle of the gyro when called. It does this by multiplying the value of the gyro by the time since the last call and then by a constant and adding that to the angle variable. This is done in the following code. `angle += (micros() - prevTime) / 1000000.0 * (getGyroChange() - drift) / -14.286 * 9/8; prevTime = micros();`

### Camera Class
The Camera class interfaces with the PixyCam 2.1 through SPI (Serial Peripheral Interface) and gets the detected blocks. The member variable in this function is the "pixy" variable, an instance of the Pixy2 class provided by the "Pixy2.h" library. 

The first method in the class is the "setup" function which initializes the PixyCam 2.1. This is done by using the "pixy.init()" function provided in the library.

The second method in the Camera class is the "getClosest" function which returns the closest block to the robot. The PixyCam 2.1 has the closest block as the first element in the array of blocks so we return that first block as shown in the following code `pixy.ccc.getBlocks(); return pixy.ccc.blocks[0];`

## Setup Code
In the setup of our program, we have a multitude of global functions and variables which we use in our main program. 

Arduino requires us to have a `void setup()` function which runs once when the Arduino is turned on and a `void loop` function which runs repeatedly until the Arduino turns off. In our code, we keep the setup of all the sensors and the waiting until the button is pressed in the `setup` function and we use the `loop` function to run our code. The functions which we use in our code are the `void delay_2` function which takes in an integer value and delays for that amount of milliseconds while updating the gyro sensor. We use this function instead of the default `delay` function since it allows our gyro sensor to constantly update. We then have two separate functions for each challenge. The `void open` function contains the code for the open challenge and the `void obstacle` contains the code for the obstacle challenge. We implement these functions in the `loop` function by commenting which function we are not running and uncommenting the function that we are running.

The global varables that we have in our code are the following
| Vairable Name | Type | Purpose |
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
| cornerScanDelay | int | Stores the time a corner was passed |
| cornerDetected | bool | Stores if a line has been detected |
| steer | float | Stores the steering value |
| prevObj | Block | Stores the previous block |
| closeBlock | Block | Stores the closest block |
| target | int | stores the target position for the blocks |
| err | float | Stores the rror for following the object |
| kP | float | Stores the kP value for following |
| objDelay | int | Stores the time when an object is passed |
| targetAngle | int | Stores the target angle for gyro following |

## Open Challenge Code

## Obstacle Challenge Code
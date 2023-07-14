from gpiozero import PhaseEnableMotor, Servo, AngularServo
import time

lMotor = PhaseEnableMotor(phase="GPIO17", enable="GPIO27")
rMotor = PhaseEnableMotor(phase="GPIO22", enable="GPIO24")

lMotor.forward(0.5)
rMotor.forward(0.5)
time.sleep(1)

steer = AngularServo(17, min_angle=Servo(17).min(), max_angle=Servo(17).max())
steer.angle = 0
time.sleep(1)
# Overview
Arduino Line Follower Car with Object Avoidance project at California State University Channel Islands (CSUCI) done by Romeo Touma. Line follower with obstacle avoidance is an intelligent robot that we can program it to start detecting a visual line embedded on the floor to follow it with. The path is like a black line on a white surface with a high contrasted color. In order to detect these lines and avoid objects, various sensors can be employed on the car and programed to act as necessary to get the car do the job.




# Dependencies:
## Software:

Arduino IDE 2.0

C/C++ language

## Hardware :
• Arduino Uno is the brain of the robot

• Motor driver l298n for controlling the motor

• 2 sets of IR Sensors for line tracking and object follow

• Ultrasonic sensor (hc-sr04) for detecting an object

• Battery (9V preferred)

• 2 sets of dc motors with the wheel

• Jumper wire



# Wiring and Setting Up the Car:
## Hardware part:

1-Connect left and right motor to the motor driver

2-Connect motor driver module to Arduino uno pins as follows:

ENA = D6 

IN1 = D7

IN2 = D8

IN3 = D9

IN4 = D10

ENB = D5

3-Attach wires to plus 5 volt and ground pins of motor driver of motor driver, then 
take plus volt and ground from motor driver and provide two ir sensors also provide 
plus 5 volt and ground to Arduino uno from the ports.

4-Now connect right sensor to d11 out pin and left sensor to d12 out pin of the 
Arduino 

5-Connect wires from switch to plus spell volt and ground pin of the motor driver then 

6-Connect the ultra-sonic sensor to the car as follows:

Port Gnd to Arduino Gnd

Echo to D2

Trig to D3

Vcc to 5 volt port


## Software:


Download Arduino IDE software on your pc/laptop
Plot your c/c++ language code on Arduino environment then press verify and import it to Arduino board after connecting it to your laptop/pc via usb port.



# Code Details:

the project contain one file (LineFollower_ObjectAvoidance.ino) which controls the car and sensors.
the code starts by the varialbles and the pins to use on the Arduino.

code example:



#define IR_SENSOR_RIGHT 11 

#define IR_SENSOR_LEFT 12

The Ultrasonic trigger pin is connect to Pin 3

The Ultrasonic echo receiver is connect to PWM Pin 2

Pin D2 Arduino to pin Echo of HC-SR04

#define echoPin 2 

Pin D3 Arduino to pin Trig of HC-SR04

#define trigPin 3




Define the Motor controller pins

Right motor

#define enableRightMotor 6

#define rightMotorPin1 7

#define rightMotorPin2 8



Left motor

#define enableLeftMotor 5

#define leftMotorPin1 9

#define leftMotorPin2 10




Define the Motor Speed static variable.

#define MOTOR_SPEED 180

# TODO for Future Developer
1- Add a functionality that let the car avoide the object by trunning around it to get back on track

2- Add a buzzer sensor and programe it to warn us when getting to close to an object

3- improve speed and stability so it runs faster without losing track

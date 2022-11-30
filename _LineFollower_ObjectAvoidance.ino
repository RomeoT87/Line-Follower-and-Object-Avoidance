// Romeo Touma 
// Comp 499 - Capstone Project
// Dr. Isaac
// 11/27/2022
//
// Description of Project:
// V2 Linefollower and enhanced ultrasonic features
//
// This software is downloaded to the Arduino microcontroller via
// the USB port.  The code will driver a two motor vehicle to follow
// a black line on the floor.  Also, when an object is detected 
// in front of the vehicle, it will go around the obsticle and 
// back to follow the black line.
//
// The vehicle will use two infrared trasmitters and receiver to
// measure the reflection or no reflection from the black surface.
// At the same time an Ultrasonic transreciever is used to detect
// the objects in front of the vehicle.
//
// The connections are as follow:
// Right IR sensor is connected to Pin 11
// Left IR sensor is connected to Pin 12
// The Ultrasonic trigger pin is connect to Pin 3
// The Ultrasonic echo receiver is connect to PWM Pin 2
//
//

// Define the varialbles and the pins to use on the Arduino.
#define IR_SENSOR_RIGHT 11 
#define IR_SENSOR_LEFT 12
// Pin D2 Arduino to pin Echo of HC-SR04
#define echoPin 2 
//Pin D3 Arduino to pin Trig of HC-SR04
#define trigPin 3

// Define the Motor controller pins

//Right motor
#define enableRightMotor 6
#define rightMotorPin1 7
#define rightMotorPin2 8

//Left motor
#define enableLeftMotor 5
#define leftMotorPin1 9
#define leftMotorPin2 10

// Define the Motor Speed static variable.
#define MOTOR_SPEED 180

// duration is the time it takes for the sound to travel.
long duration; 
// the distance the sound will travel.
int distance;

// This setup runs when Arduino is first started.
void setup()
{
  // Set the PWM Frequency for the motor controller.
  // This sets the frequency to about 7200hz.
  TCCR0B = TCCR0B & B11111000 | B00000010;

  // Setup the serial port to use for debugging. 
  // This setup the speed at which to send the data to 
  // the Serial Monitor.  The Serial Monitor will receive the
  // information from each Serial.println() command.
  Serial.begin(57600);

  // Setup the ultrasonic Transreceiver
  // Sets the trigPin as an OUTPUT
  pinMode(trigPin, OUTPUT);
  // Sets the echoPin as an INPUT 
  pinMode(echoPin, INPUT); 
  
  // Setup the Right Motor.  enableRight Motor is driven
  // by a PWM signal.  0 to 255. 
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  // Setup the Right Motor.  enableLeft Motor is driven
  // by a PWM signal.  0 to 255  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Setup the IR Sensor Pin for Input
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // Call the rotatMotor function to make sure the
  // motor stop at the start.
  rotateMotor(0,0);   
}

// Continous loop
void loop()
{

  // Reading the IR sensor values
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  // Display the sensors value and motor speed to serial monitor
  Serial.print("Right: ");
  Serial.println(rightIRSensorValue);
  Serial.print("Left: ");
  Serial.println(leftIRSensorValue);

  // Setup the trigger condition for the ulrasonic
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Here we calculate the distance, Speed of sound wave divided by 2.
  distance = duration * 0.034 / 2; 
  
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
 
  // Display Motor Speed
  Serial.print("MOTOR_SPEED: ");
  Serial.println(MOTOR_SPEED);

  // Check if there is an object in the way
  if (distance <= 12) 
  {
     //Turn left
     Serial.println("turn motor to the left!");
     rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
     //Delay 5 Seconds
     delayMicroseconds(5000000);
     //Go Straight
     Serial.println("go straight!");
     rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
     //Delay 1 Second
     delayMicroseconds(1000000);
     //Turn left until we see the black line
     Serial.println("turn motor to the right!");
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
     
  }  else  {
     //If none of the sensors detects black line, then go straight
     if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
     {
       Serial.println("I should be going Straight!");
       rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
     }
     //If right sensor detects black line, then turn right
     else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
     {
         Serial.println("I should be turning right!");
         rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
     }
     //If left sensor detects black line, then turn left  
     else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
     {
         Serial.println("I should be turning left!");
         rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
     } 
     //If both the sensors detect black line, then stop 
     else 
     {
       Serial.println("Both Sensors are black, I am stopping!");
       rotateMotor(0, 0);
     }
  } // end else
}

// This function will rotate the motors according to the input
// parameters. For example:
// rotateMotor(0,0) will stop the vehicle
// rotateMotor(-180,180) will turn right
// rotateMotor(180,-180) will turn left
// rotateMotor(-180, -180) will reverse the vehicle
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}

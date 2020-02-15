#include <Servo.h> 

Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myServo4;

const int flexPin1 = A0;
const int flexPin2 = A1;
const int flexPin3 = A2;
const int flexPin4 = A3;

const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 160000.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg

void setup() 
{ 
  myServo1.attach(12);
  myServo2.attach(11);
  myServo3.attach(9);
  myServo4.attach(7);
  Serial.begin(9600);
} 

void loop() 
{ 
  int flexValue1;
  int servoPosition1;
  int flexValue2;
  int servoPosition2;
  int flexValue3;
  int servoPosition3;
  int flexValue4;
  int servoPosition4;
  
  flexValue1 = analogRead(flexPin1);
  flexValue2 = analogRead(flexPin2);
  flexValue3 = analogRead(flexPin3);
  flexValue4 = analogRead(flexPin4);
  
  servoPosition1 = map(flexValue1, 0, 500, 0, 180);
  servoPosition1 = constrain(servoPosition1, 0, 180);
  servoPosition2 = map(flexValue2, 800, 900, 0, 180);
  servoPosition2 = constrain(servoPosition2, 0, 180);
  servoPosition3 = map(flexValue3, 800, 900, 0, 180);
  servoPosition3 = constrain(servoPosition3, 0, 180);
  servoPosition4 = map(flexValue4, 800, 900, 0, 180);
  servoPosition4 = constrain(servoPosition4, 0, 180);

  myServo1.write(servoPosition1);
  myServo2.write(servoPosition2);
  myServo3.write(servoPosition3);
  myServo4.write(servoPosition4);

  delay(20);

  int flexADC = analogRead(flexPin1);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();

  delay(100);
} 

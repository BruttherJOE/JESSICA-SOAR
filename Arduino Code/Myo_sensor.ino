#include <Servo.h>    // Using the Servo library
Servo elbowJoint;     // Naming our servo ‘elbowJoint’
int EMGsig;           // Store the EMG signal value
int servoPosition;    // The position (angle) value for the servo
int threshold = 300;  // Move the servo when EMG signal is above this threshold. Remember it ranges 0–1023.

void setup() {
 Serial.begin(9600); // Starting the communication with the computer
 elbowJoint.attach(9); // Tell the servo it is plugged into pin 9
}

void loop() {
 EMGsig = analogRead(A0); // Read the analog values of the rectified+integrated EMG signal (0–1023)
if (EMGsig < threshold){     // If EMG signal is below the threshold
    servoPosition = 20;       // Servo will remain at 20 degrees.
  } else{            // If the EMG signal is above the threshold,
   servoPosition=map(EMGsig,threshold,1023,20,160); 
  // The servo angle will be mapped with the EMG signal, 
  // changing the range of 300(our threshold)-1023 into the range of 20-160 degrees.
  // 20 and 160 can be switched depending on which direction of rotation you want.
}
 elbowJoint.write(servoPosition); 
  // Move the servo to the ‘servoPosition’ degree

 
  // Display the servo and EMG values.
 delay(1000); 
  // 1 second (1000ms) delay to not cause it to move as frantically. But this can be adjusted as you like.
}

# JESSICA-SOAR


Documentation for incoming team, taking over the JESSICA robot arm.

JESSICA is a robot arm that can automatically serve coffee upon activation. 
We plan to add a manual function, where users can control JESSICA with an EMG myosensor armband.


# Progress

However, due to its difficulty in controlling myosensor armband, we decided to resort to flex sensor.
18 - 23 Jan : Constructing a 3D-printed gripper.
# Reason : plan to swap the gripper since the new gripper is better in grabbing things
# Status : Completed

18 - 29 Jan : Shifting JESSICA robot from the old table to the new one and setting up the layout for JESSICA
# Reason : When moving JESSICA, the old table looks fragile so we wanted to get a more sturdy table for JESSICA
# Status : Completed

29 Jan -   : Moving the electronics from the bredboard to perfboard
# Reason : The layout of bredboard is messy. Therefore, we are trying to make the electronic arrangement neater.
# Status : Completed

30 Jan - 7 Feb : Constructing the flex-sensor-integrated glove
# Status : Postponed because JESSICA was not working after the shifting of JESSICA. 
# Problem 1 : When JESSICA starts moving, the RPi immediately reboots.

8 - 14 Feb : Debugging problem 1
What we did : Since the software is not touched, we suspect that the hardware or the wiring so we 









## Simple communication from Arduino Uno to Raspberry Pi:
### Code in Arduino
```
void setup(){
  Serial.begin(9600);
}

void loop(){
  Serial.println("Hello World!");   
  delay(2000);
}
```
### Python code in RPi:
```
ser = serial.Serial('/dev/ttyUSB0', 9600)
while 1: 
    if(ser.in_waiting >0):
        line = ser.readline()
        print(line)
```


### Useful guides:

Connecting RPi and Uno:
https://www.instructables.com/id/Connect-Your-Raspberry-Pi-and-Arduino-Uno/
https://classes.engineering.wustl.edu/ese205/core/index.php?title=Serial_Communication_between_Raspberry_Pi_%26_Arduino

Myosensor explained:
https://medium.com/@leex5202/an-unofficial-introductory-tutorial-to-myoware-muscle-sensor-development-kit-e2169948e63

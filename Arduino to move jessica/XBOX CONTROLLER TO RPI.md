 # xbox controller to move Jessica


got some initial problems : when connecting bluetooth module to rpi it kicks out ssh connection as well as ethernet connection at the same time.


We first install the driver so that it can be communicated with the controller:  
`sudo apt-get install xboxdrv`
  

`lsusb`  
lists all things connected via usb  

expected output within other useless outputs:  
`Bus 001 Device 005: ID 045e:0291 Microsoft Corp. Xbox 360 Wireless Receiver for Windows`  

ROS  
to check for name of joystick  
`ls /dev/input/`

you should get a list of all the connected input devices to the rpi.  
for our case js0 was the name of our connected xbox controller.  

> IMPT : FOR THE FOLLOWING CASES BELOW, X STANDS FOR THE NUMBER OF THE CONTROLLER. IN OUR CASE IT WAS JS0.  
  

to test whether the buttons give output log on terminal use   
`sudo jstest /dev/input/jsX`  

then we make joystick accessible for the ROS joy node. Start by listing the permissions of the joystick:  

`ls -l /dev/input/jsX`  

```
roscore
rosparam set joy_node/dev "/dev/input/jsX"
```

 
rosparam sets the controller profile to your connected controller   


-------------------------------------
we then start the joy node. this node needs to be running or else the arm cannot be moving.    
do  
`rosrun joy joy_node`  
  


### to view output logs from joynode

>open new terminal

do  
`rostopic echo joy`  


in niryo studio, enable joystick mode. the arm should be able to move when the buttons are pressed.

  
  
  
   

   

CREDITS
---------------
followed   
https://tutorials-raspberrypi.com/raspberry-pi-xbox-360-controller-wireless/ 

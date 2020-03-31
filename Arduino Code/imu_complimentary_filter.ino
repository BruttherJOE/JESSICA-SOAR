//compensate filter for mpu9255
//get the library at: https://github.com/Bill2462/MPU9255-Arduino-Library
//roll and pitch are acceptable, haven't figured out how to make yaw work

#include <MPU9255.h>//include MPU9255 library
 
MPU9255 mpu;

float Aroll, Apitch, Myaw, Mx, My;
float roll, pitch, yaw;

float dt = 0.02;
float HP = 0.98; //High pass weight
float LP = 0.02; //Low pass weight

float ax,ay,az, gx,gy,gz, mx,my,mz;

void setup() {
  Serial.begin(115200);//initialize Serial port
 
  if(mpu.init())
  {
  Serial.println("initialization failed");
  }
  else
  {
  Serial.println("initialization succesful!");
  }  

  //initiate
  mpu.read_acc();//get data from the accelerometer
  mpu.read_mag();//get data from the magnetometer

  ax = mpu.ax;
  ay = mpu.ay;
  az = mpu.az;

  mx = mpu.mx;
  my = mpu.my;
  mz = mpu.mz;
  
  roll = (float)(atan2(ay, sqrt(ax*ax + az*az)))*180/PI;
  pitch = (float)(atan2(-ax, sqrt(ay*ay + az*az)))*180/PI;

  Mx = mx*cos(pitch) + mz*sin(pitch);
  My = mx*sin(roll)*sin(pitch) + my*cos(roll) - mz*sin(roll)*cos(pitch);
  yaw = atan2(-My,Mx)*180/PI;
  
}

void loop() {
  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer

  ax = mpu.ax;
  ay = mpu.ay;
  az = mpu.az;

  gx = mpu.gx/131;
  gy = mpu.gy/131;
  gz = mpu.gz/131;

  mx = mpu.mx;
  my = mpu.my;
  mz = mpu.mz;
  
  Aroll = (float)(atan2(ay, sqrt(ax*ax + az*az)))*180/PI;
  Apitch = (float)(atan2(-ax, sqrt(ay*ay + az*az)))*180/PI;

  Mx = mx*cos(Apitch) + mz*sin(Apitch);
  My = mx*sin(Aroll)*sin(Apitch) + my*cos(Aroll) - mz*sin(Aroll)*cos(Apitch);
  Myaw = atan2(-My,Mx)*180/PI;

  roll = HP*(roll + gx*dt) + LP*Aroll;
  pitch = HP*(pitch + gy*dt) + LP*Apitch;
  yaw = HP*(yaw + gz*dt) + LP*Myaw;

  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print(" pitch: ");
  Serial.print(pitch);
  Serial.print(" yaw: ");
  Serial.println(yaw);

  delay(20);
}

//Kalman filter for mpu9255 to get orientation implemented by Kevinskwk
//get the library at: https://github.com/Bill2462/MPU9255-Arduino-Library
//reference: https://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?referer=https://www.google.com/&httpsredir=1&article=1422&context=eesp
//to do: need to set magnetometer offset value to get reasonable yaw values

#include <MPU9255.h>//include MPU9255 library
#include "Kalman.h"

MPU9255 mpu;

float dt = 0.02;

float ax,ay,az, gx,gy,gz, mx,my,mz;

float mag_off[3];
float angles[3];
Kalman KFilter[3];

unsigned long tStart, tStop, tElapse, eTime;

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

  mag_off[0] = 0;//to be updated
  mag_off[1] = 0;//to be updated
  mag_off[2] = 0;//to be updates

  delay(1000);

  //initiate
  mpu.read_acc();//get data from the accelerometer
  mpu.read_mag();//get data from the magnetometer

  ax = mpu.ax;
  ay = mpu.ay;
  az = mpu.az;
  
  KFilter[0].set_angle((float)(atan2(ay, sqrt(ax*ax + az*az)))*180/PI);
  KFilter[1].set_angle((float)(atan2(-ax, sqrt(ay*ay + az*az)))*180/PI);

  float norm = Norm(ax, ay, az);
  float pitchA = -asin(ax/norm);
  float rollA = asin(ay/cos(pitchA)/norm);

  float magX = mpu.mx - mag_off[0];
  float magY = mpu.my - mag_off[1];
  float magZ = mpu.mz - mag_off[2];

  norm = sqrt(magX*magX + magY*magY + magZ*magZ);

  mx = magX/ norm;
  my = -1 * magY/ norm;
  mz = magZ/ norm; 

  float Mx = mx * cos(pitchA) + mz * sin(pitchA);
  float My = mx * sin(rollA) * sin(pitchA) + my * cos(rollA) - mz * sin(rollA) * cos(pitchA);
  float yaw = atan2(-My,Mx)*180/PI;

  if (yaw > 360) {
    yaw -= 360;
  }else if (yaw < 0) {
    yaw += 360;
  }

  KFilter[2].set_angle(yaw);
  
  tStart = millis();
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
 
  float roll = (float)(atan2(ay, sqrt(ax*ax + az*az)))*180/PI;
  float pitch = (float)(atan2(-ax, sqrt(ay*ay + az*az)))*180/PI;

  float norm = Norm(ax, ay, az);
  float pitchA = -asin(ax/norm);
  float rollA = asin(ay/cos(pitchA)/norm);

  float magX = mpu.mx - mag_off[0];
  float magY = mpu.my - mag_off[1];
  float magZ = mpu.mz - mag_off[2];

  norm = sqrt(magX*magX + magY*magY + magZ*magZ);

  mx = magX/ norm;
  my = -1 * magY/ norm;
  mz = magZ/ norm; 

  float Mx = mx * cos(pitchA) + mz * sin(pitchA);
  float My = mx * sin(rollA) * sin(pitchA) + my * cos(rollA) - mz * sin(rollA) * cos(pitchA);
  float yaw = atan2(-My,Mx)*180/PI;

  if (yaw > 360) {
    yaw -= 360;
  }else if (yaw < 0) {
    yaw += 360;
  }
  
  tStop = millis();
  tElapse = tStop - tStart;
  tStart = millis();
  float temp = tElapse/1000.0;

  angles[0] = KFilter[0].get_angle(gx, roll, temp);
  angles[1] = KFilter[1].get_angle(gy, pitch, temp);
  angles[2] = KFilter[2].get_angle(gz, yaw, temp);

  Serial.print("roll: ");
  Serial.print(angles[0]);
  Serial.print(" pitch: ");
  Serial.print(angles[1]);
  Serial.print(" yaw: ");
  Serial.println(angles[2]);
}

float Norm(float a, float b, float c) {
  return sqrt(a*a + b*b + c*c);
}

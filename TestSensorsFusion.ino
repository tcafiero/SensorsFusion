#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);




// IMPORTANT
// this is not a working example, this is just to show how to set the filter
// if you need a working example please see: https://www.hackster.io/vincenzo-g/sensor-fusion-algorithms-made-simple-6c55f6


#include "SensorFusion.h" //SF
SF filter;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
   
   
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() {

  Serial.begin(115200); //serial to display data
  // your IMU begin code goes here
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  //Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
}

void loop() {
  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);
  /*
  gx=g.gyro.x * DEG_TO_RAD;
  gy=g.gyro.y * DEG_TO_RAD;
  gx=g.gyro.z * DEG_TO_RAD;
  ax=a.acceleration.x;
  ay=a.acceleration.y;
  az=a.acceleration.z;
  mx=m.magnetic.x;
  my=m.magnetic.y;
  mz=m.magnetic.z;
 */
  deltat = filter.deltatUpdate(); //this have to be done before calling the filter update
  //choose only one of these two:
  //filter.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  filter.MadgwickUpdate(g.gyro.x * DEG_TO_RAD, g.gyro.x * DEG_TO_RAD, g.gyro.x * DEG_TO_RAD, a.acceleration.x, a.acceleration.y, a.acceleration.z, m.magnetic.x, m.magnetic.y, m.magnetic.z, deltat);  //else use the magwick, it is slower but more accurate

  pitch = filter.getPitch();
  roll = filter.getRoll();    //you could also use getRollRadians() ecc
  yaw = filter.getYaw();

  Serial.print(pitch); Serial.print(":");
  Serial.print(roll); Serial.print(":");
  Serial.print(yaw); Serial.print(":");
  Serial.println();

  //delay(200);
}

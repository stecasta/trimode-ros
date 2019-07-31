#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

sensor_msgs::Imu IMU;

//imu::Quaternion offset;

float zeros[9]= {};

ros::NodeHandle nh;

ros::Publisher pub("imu/data", &IMU);

void setup()
{  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
//  delay(4000);
  
  //Initialize IMU data!!
  
  bno.setExtCrystalUse(true);
  
  nh.initNode();
  nh.advertise(pub);
}

void loop()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angulvel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE) / 360 * 6.28318530718;

  IMU.header.frame_id = "imu_link"; //check frame
  IMU.header.stamp = nh.now();
//  IMU.header.frame_id = "map"; //check frame
  IMU.orientation.x = quat.x();
  IMU.orientation.y = quat.y();
  IMU.orientation.z = quat.z();
  IMU.orientation.w = quat.w();
  
  IMU.angular_velocity.x = angulvel.x();
  IMU.angular_velocity.y = angulvel.y();
  IMU.angular_velocity.z = angulvel.z();

  IMU.linear_acceleration.x = linearaccel.x();
  IMU.linear_acceleration.y = linearaccel.y();
  IMU.linear_acceleration.z = linearaccel.z();

  for(unsigned row = 0; row < 3; ++ row) {
    for(unsigned col = 0; col < 3; ++ col) {
      IMU.orientation_covariance[row * 3 + col] = (row == col? 0.002: 0.);  // +-2.5deg
      IMU.angular_velocity_covariance[row * 3 + col] = (row == col? 0.003: 0.);  // +-3deg/s
      IMU.linear_acceleration_covariance[row * 3 + col] = (row == col? 0.60: 0.);  // +-80mg
    }
  }
  //add check if connection to IMU is lost
  pub.publish( &IMU );
  nh.spinOnce();
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

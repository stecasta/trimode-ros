#include <ros.h>
#include <trimode_control/FloatList.h>
#include <trimode_control/BoolList.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>
#include <Rotary.h>

#define switchPin0 9
#define switchPin1 10

//ros::NodeHandle nh;

ros::NodeHandle_<ArduinoHardware, 2, 2, 150, 150> nh;

trimode_control::FloatList wheel_pos;

volatile long encoder0Pos=0;
volatile long encoder1Pos=0;

//bool zeros[4] = {0};
//trimode_control::BoolList home_pos;
trimode_control::FloatList home_pos;

Rotary r0 = Rotary(2, 3);
Rotary r1 = Rotary(4, 5);

ros::Publisher pub("encoder_right_vel_transf", &wheel_pos);
ros::Publisher pub_home("home_right_position", &home_pos);

void setup()
{
  r0.begin();
  r1.begin();  
  PCICR |= (1 << PCIE2) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  sei();

  pinMode(switchPin0, INPUT);
  pinMode(switchPin1, INPUT);
  
  wheel_pos.data_length = 4;
  home_pos.data_length = 4;
  wheel_pos.data = (float*)malloc(sizeof(float) *2);
  home_pos.data = (float*)malloc(sizeof(float) *2);
    
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(pub_home);
}

void loop()
{
nh.spinOnce();
// Initialize every loop in case of lost connection.
wheel_pos.data[2] = 0;
wheel_pos.data[3] = 0;
  
long newposition0 = encoder0Pos;
long newposition1 = encoder1Pos;

wheel_pos.data[0] = 0;
wheel_pos.data[1] = 0;
wheel_pos.data[2] = newposition1;
wheel_pos.data[3] = newposition0;

// Add safety check if switch is not plugged!!

if (digitalRead(switchPin0)){
  home_pos.data[3] = digitalRead(switchPin0);
  encoder0Pos = 0;
}
else{
  home_pos.data[3] = 0;
}

if (digitalRead(switchPin1)){
  home_pos.data[2] = digitalRead(switchPin1);
  encoder1Pos = 0;
}
else{
  home_pos.data[2] = 0;
}

pub.publish( &wheel_pos );
pub_home.publish( &home_pos );
  
//delay(10);
}

ISR(PCINT2_vect) {
  unsigned char result0 = r0.process();
  unsigned char result1 = r1.process();
  if (result0 == DIR_NONE) {
    // do nothing
  }
  else if (result0 == DIR_CW) {
//    Serial.println("ClockWise");
    encoder0Pos++;
  }
  else if (result0 == DIR_CCW) {
//    Serial.println("CounterClockWise");
    encoder0Pos--;
  }
  if (result1 == DIR_NONE) {
    // do nothing
  }
  else if (result1 == DIR_CW) {
//    Serial.println("ClockWise");
    encoder1Pos++;
  }
  else if (result1 == DIR_CCW) {
//    Serial.println("CounterClockWise");
    encoder1Pos--;
  }
}

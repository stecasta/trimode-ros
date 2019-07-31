#include <ros.h>
#include <trimode_control/FloatList.h>
#include <trimode_control/BoolList.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>
#include <Rotary.h>

#define encoder0PinA  2
#define encoder0PinB  3
#define encoder1PinA  2
#define encoder1PinB  3
#define switchPin0 9
#define switchPin1 10

ros::NodeHandle nh;

trimode_control::FloatList wheel_pos;

volatile long encoder0Pos=0;
volatile long encoder1Pos=0;

//bool zeros[4] = {0};
//trimode_control::BoolList home_pos;
trimode_control::FloatList home_pos;

Rotary r0 = Rotary(2, 3);
Rotary r1 = Rotary(4, 5);

ros::Publisher pub("encoder_vel_transf", &wheel_pos);
ros::Publisher pub_home("home_right_position", &home_pos);


void messageCb( const trimode_control::FloatList & msg) {
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  wheel_pos = msg; 
  home_pos = msg;
}

ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );

void setup()
{
  r0.begin();
  r1.begin();  
  PCICR |= (1 << PCIE2) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  sei();

  pinMode(switchPin0, INPUT);
  pinMode(switchPin1, INPUT);
  
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(pub_home);
  nh.subscribe(sub);
}

void loop()
{
// Initialize every loop in case of lost connection.
wheel_pos.data[2] = 0;
wheel_pos.data[3] = 0;
//
//home_pos.data[0] = digitalRead(switchPin0);
//home_pos.data[1] = digitalRead(switchPin0);
//home_pos.data[2] = digitalRead(switchPin0);
//home_pos.data[3] = digitalRead(switchPin0);
  
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
//home_pos.data[0] = 2;
pub_home.publish( &home_pos );
nh.spinOnce();
  
delay(100);
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

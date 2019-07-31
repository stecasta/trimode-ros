#include <ros.h>
#include <trimode_control/FloatList.h>
#include <std_msgs/String.h>
#include <Arduino.h>
#include <Rotary.h>

Rotary r0 = Rotary(2, 3);
Rotary r1 = Rotary(4, 5);

ros::NodeHandle nh;

trimode_control::FloatList wheel_velocities;

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long oldposition0 = 0;
long oldposition1 = 0;  
unsigned long oldtime = 0;

ros::Publisher pub("encoder_front_vel", &wheel_velocities);

void messageCb( const trimode_control::FloatList & msg) {
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  wheel_velocities = msg; 
}

ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );

void setup()
{
  r0.begin();
  r1.begin();
  PCICR |= (1 << PCIE2) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  sei();

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop()
{
wheel_velocities.data[0] = 0;
wheel_velocities.data[1] = 0;
wheel_velocities.data[2] = 0;
wheel_velocities.data[3] = 0;
long newposition0 = encoder0Pos;
long newposition1 = encoder1Pos;
unsigned long newtime = micros();
float vel0 = ((float)(newposition0-oldposition0)/ 9500.0) * 3.14159265 * 2.0 / ((float)(newtime - oldtime) / (1000000.0));
float vel1 = ((float)(newposition1-oldposition1)/ 9500.0) * 3.14159265 * 2.0 / ((float)(newtime - oldtime) / (1000000.0));
oldposition0 = newposition0;
oldposition1 = newposition1;
oldtime = newtime;
wheel_velocities.data[0] = vel0;
wheel_velocities.data[1] = 0;
wheel_velocities.data[2] = vel1;
wheel_velocities.data[3] = 0;

pub.publish( &wheel_velocities );
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

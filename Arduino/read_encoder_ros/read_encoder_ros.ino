#include <ros.h>
#include <trimode_control/FloatList.h>
#include <std_msgs/String.h>
#include <Arduino.h>

#define encoder0PinA  2
#define encoder0PinB  3

ros::NodeHandle nh;

trimode_control::FloatList wheel_velocities;

volatile long encoder0Pos=0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
float vel;

ros::Publisher pub("encoder_vel", &wheel_velocities);

void messageCb( const trimode_control::FloatList & msg) {
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  wheel_velocities = msg; 
}

ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );

void setup()
{
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

}

void loop()
{
newposition = encoder0Pos;
newtime = micros();
//vel = ((newposition-oldposition) * 1000 * 3.14159265) / ((newtime-oldtime) * 9500);
float vel = ((float)(newposition-oldposition)/ 9500.0) * 3.14159265 * 2.0 / ((float)(newtime - oldtime) / (1000000.0));
oldposition = newposition;
oldtime = newtime;
wheel_velocities.data[0] = 0;
wheel_velocities.data[1] = vel;
wheel_velocities.data[2] = 0;
wheel_velocities.data[3] = 0;

pub.publish( &wheel_velocities );
nh.spinOnce();
  
delay(1);
}

void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

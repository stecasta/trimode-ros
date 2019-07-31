#include <ros.h>
#include <trimode_control/FloatList.h>
#include <std_msgs/String.h>
#include <Arduino.h>

#define encoder0PinA  2
#define encoder0PinB  3

ros::NodeHandle nh;

trimode_control::FloatList wheel_pos;

volatile long encoder0Pos=0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
float vel;
float pos;

ros::Publisher pub("encoder_vel_transf", &wheel_pos);

void messageCb( const trimode_control::FloatList & msg) {
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  wheel_pos = msg; 
}

ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );

void setup()
{
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2

  pos = 0;

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

}

void loop()
{
newposition = encoder0Pos;
float delta_pos = ((float)(newposition-oldposition)/ 5830.0) * 3.14159265 * 2.0; 
//float delta_pos = (float)(newposition-oldposition);
pos += delta_pos; 
oldposition = newposition;
oldtime = newtime;

wheel_pos.data[0] = 0;
wheel_pos.data[1] = pos;
wheel_pos.data[2] = 0;
wheel_pos.data[3] = 0;

pub.publish( &wheel_pos );
nh.spinOnce();
  
delay(10);
}

void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

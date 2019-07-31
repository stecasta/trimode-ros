/*
 *
rosrun rosserial_python serial_node.py /dev/ttyUSB0
rosrun rosserial_python serial_node.py /dev/ttyACM0

   0 left_front
   1 left_back
   2 right_front
   3 right_back
*/

#include <ros.h>
#include <trimode_control/FloatList.h>
#include <std_msgs/String.h>
#include <Arduino.h>

//motors for robot motion
#define enA 9
#define in1 7
#define in2 6
#define enB 7
#define in3 6
#define in4 5

ros::NodeHandle nh;

trimode_control::FloatList pwms;

int led = 13;
bool led_on = false;

int rotDirection = 0;

void messageCb( const trimode_control::FloatList & msg) {
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  pwms = msg;

  if (pwms.data[0] == 0) {
    analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  } else if (pwms.data[0] > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwms.data[0]); // Send PWM signal to L298N Enable pin
    //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, abs(pwms.data[0])); // Send PWM signal to L298N Enable pin
  }

  if (pwms.data[2] == 0) {
    analogWrite(enB, 0); // Send PWM signal to L298N Enable pin
  } else if (pwms.data[3] > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, pwms.data[2]); // Send PWM signal to L298N Enable pin
  } else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, abs(pwms.data[2])); // Send PWM signal to L298N Enable pin
  }
}

ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );

//std_msgs::String str_msg;
ros::Publisher pub("arduino_debug", &pwms);

void setup()
{
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop()
{       
  //str_msg.data = hello;
  pub.publish( &pwms );
  nh.spinOnce();
  delay(20);
}

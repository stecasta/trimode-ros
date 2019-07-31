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
#define enAf 7
#define in1f 40
#define in2f 41
#define enBf 9
#define in3f 24
#define in4f 25

#define enAb 11
#define in1b 36
#define in2b 37
#define enBb 5
#define in3b 30
#define in4b 31

ros::NodeHandle nh;

trimode_control::FloatList pwms;
//ros::Time last_received_cmd;
unsigned long last_received_cmd;

int led = 13;
bool led_on = false;

void messageCb( const trimode_control::FloatList & msg) {
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//  last_received_cmd = ros::Time::now();
  last_received_cmd = millis();
  pwms = msg;

  if (pwms.data[0] == 0) {
    analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin
  } else if (pwms.data[0] > 0) {
    digitalWrite(in1f, HIGH);
    digitalWrite(in2f, LOW);
    analogWrite(enAf, pwms.data[0]); // Send PWM signal to L298N Enable pin
    //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  } else {
    digitalWrite(in1f, LOW);
    digitalWrite(in2f, HIGH);
    analogWrite(enAf, abs(pwms.data[0])); // Send PWM signal to L298N Enable pin
  }

  if (pwms.data[2] == 0) {
    analogWrite(enBf, 0); // Send PWM signal to L298N Enable pin
  } else if (pwms.data[2] > 0) {
    digitalWrite(in3f, LOW);
    digitalWrite(in4f, HIGH);
    analogWrite(enBf, pwms.data[2]); // Send PWM signal to L298N Enable pin
  } else {
    digitalWrite(in3f, HIGH);
    digitalWrite(in4f, LOW);
    analogWrite(enBf, abs(pwms.data[2])); // Send PWM signal to L298N Enable pin
  }
  
  if (pwms.data[1] == 0) {
    analogWrite(enAb, 0); // Send PWM signal to L298N Enable pin
  } else if (pwms.data[1] > 0) {
    digitalWrite(in1b, HIGH);
    digitalWrite(in2b, LOW);
    analogWrite(enAb, pwms.data[1]); // Send PWM signal to L298N Enable pin
    //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  } else {
    digitalWrite(in1b, LOW);
    digitalWrite(in2b, HIGH);
    analogWrite(enAb, abs(pwms.data[1])); // Send PWM signal to L298N Enable pin
  }

  if (pwms.data[3] == 0) {
    analogWrite(enBb, 0); // Send PWM signal to L298N Enable pin
  } else if (pwms.data[3] > 0) {
    digitalWrite(in3b, LOW);
    digitalWrite(in4b, HIGH);
    analogWrite(enBb, pwms.data[3]); // Send PWM signal to L298N Enable pin
  } else {
    digitalWrite(in3b, HIGH);
    digitalWrite(in4b, LOW);
    analogWrite(enBb, abs(pwms.data[3])); // Send PWM signal to L298N Enable pin
  }  
  
  
}

ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );

//std_msgs::String str_msg;
ros::Publisher pub("arduino_debug", &pwms);

void setup()
{
  pinMode(enAf, OUTPUT);
  pinMode(in1f, OUTPUT);
  pinMode(in2f, OUTPUT);
  pinMode(enBf, OUTPUT);
  pinMode(in3f, OUTPUT);
  pinMode(in4f, OUTPUT);

  pinMode(enAb, OUTPUT);
  pinMode(in1b, OUTPUT);
  pinMode(in2b, OUTPUT);
  pinMode(enBb, OUTPUT);
  pinMode(in3b, OUTPUT);
  pinMode(in4b, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1f, HIGH);
  digitalWrite(in2f, LOW);
  digitalWrite(in3f, LOW);
  digitalWrite(in4f, HIGH);
  
  digitalWrite(in1b, HIGH);
  digitalWrite(in2b, LOW);
  digitalWrite(in3b, LOW);
  digitalWrite(in4b, HIGH);

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void stop_motors(){
  analogWrite(enAb, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enBb, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enBf, 0); // Send PWM signal to L298N Enable pin
}

void loop()
{       
  //str_msg.data = hello;
  if (millis() - last_received_cmd > 200){
    stop_motors();
  }
  pub.publish( &pwms );
  nh.spinOnce();
  
  delay(10);
}

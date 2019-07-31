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
#define enAb 11
#define in1b 38
#define in2b 39
#define enBb 7
#define in3b 52
#define in4b 53

//motors for front wheels transform
#define enAf 5
#define in1f 44
#define in2f 45
#define enBf 10
#define in3f 51
#define in4f 50

trimode_control::FloatList pwms;
//ros::Time last_received_cmd;
unsigned long last_received_cmd;

int led = 13;
bool led_on = false;
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

}

void stop_motors(){
  analogWrite(enAb, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enBb, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enBf, 0); // Send PWM signal to L298N Enable pin
}

void loop()
{       
  analogWrite(enAb, 60); // Send PWM signal to L298N Enable pin

  
  delay(10);
}

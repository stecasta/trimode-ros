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

//motors for back wheels transform
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

ros::NodeHandle nh;

trimode_control::FloatList pwms_wheel;

int led = 13;
bool led_on = false;

int rotDirection = 0;

void messageCb_wheel( const trimode_control::FloatList & msg_wheel) {
  pwms_wheel = msg_wheel;

  // front left wheel
  if (pwms_wheel.data[0] == 1 && pwms_wheel.data[1] != 0) {
    //analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin

    if (pwms_wheel.data[1] > 0) {
      digitalWrite(in1f, LOW);
      digitalWrite(in2f, HIGH);
      analogWrite(enAf, pwms_wheel.data[1]); // Send PWM signal to L298N Enable pin
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    }   else {
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
      digitalWrite(in1f, HIGH);
      digitalWrite(in2f, LOW);
      analogWrite(enAf, abs(pwms_wheel.data[1])); // Send PWM signal to L298N Enable pin
    }
  }
  else {
    analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin
  }

  //front right wheel
  if (pwms_wheel.data[0] == 0 && pwms_wheel.data[1] != 0) {
    //analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin

    if (pwms_wheel.data[1] > 0) {
      digitalWrite(in3f, LOW);
      digitalWrite(in4f, HIGH);
      analogWrite(enBf, pwms_wheel.data[1]); // Send PWM signal to L298N Enable pin
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    }   else {
      digitalWrite(in3f, HIGH);
      digitalWrite(in4f, LOW);
      analogWrite(enBf, abs(pwms_wheel.data[1])); // Send PWM signal to L298N Enable pin
    }
  }
  else {
    analogWrite(enBf, 0); // Send PWM signal to L298N Enable pin
  }

  //left back wheel
  if (pwms_wheel.data[0] == 2 && pwms_wheel.data[1] != 0) {
    //analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin

    if (pwms_wheel.data[1] > 0) {
      digitalWrite(in1b, LOW);
      digitalWrite(in2b, HIGH);
      analogWrite(enAb, pwms_wheel.data[1]); // Send PWM signal to L298N Enable pin
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    }   else {
      digitalWrite(in1b, HIGH);
      digitalWrite(in2b, LOW);
      analogWrite(enAb, abs(pwms_wheel.data[1])); // Send PWM signal to L298N Enable pin
    }
  }
  else {
    analogWrite(enAb, 0); // Send PWM signal to L298N Enable pin
  }

  //right back wheel
  if (pwms_wheel.data[0] == 3 && pwms_wheel.data[1] != 0) {
    //analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin
    if (pwms_wheel.data[1] > 0) {
      digitalWrite(in3b, LOW);
      digitalWrite(in4b, HIGH);
      analogWrite(enBb, pwms_wheel.data[1]); // Send PWM signal to L298N Enable pin
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    }   else {
      digitalWrite(in3b, HIGH);
      digitalWrite(in4b, LOW);
      analogWrite(enBb, abs(pwms_wheel.data[1])); // Send PWM signal to L298N Enable pin
    }
  }
  else {
    analogWrite(enBb, 0); // Send PWM signal to L298N Enable pin
  }
}

ros::Subscriber<trimode_control::FloatList> sub_wheel("pwms_wheel", messageCb_wheel );

//std_msgs::String str_msg;
ros::Publisher pub("arduino_wheel_debug", &pwms_wheel);

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
  digitalWrite(in1f, LOW);
  digitalWrite(in2f, HIGH);
  digitalWrite(in3f, HIGH);
  digitalWrite(in4f, LOW);

  digitalWrite(in1b, LOW);
  digitalWrite(in2b, HIGH);
  digitalWrite(in3b, HIGH);
  digitalWrite(in4b, LOW);

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub_wheel);
}

void loop()
{       
  //str_msg.data = hello;
  pub.publish( &pwms_wheel );
  nh.spinOnce();
  delay(20);
}

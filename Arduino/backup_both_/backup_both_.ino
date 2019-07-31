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
#define enA 10
#define in1 9
#define in2 8
#define enB 7
#define in3 6
#define in4 5

//motors for front wheels transform
#define enAf 2
#define in1f 48
#define in2f 49
#define enBf 3
#define in3f 50
#define in4f 51

//motors for back wheels transform
#define enAb 12
#define in1b 26
#define in2b 27
#define enBb 13
#define in3b 28
#define in4b 29

ros::NodeHandle nh;

trimode_control::FloatList pwms;
trimode_control::FloatList pwms_wheel;

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
ros::Subscriber<trimode_control::FloatList> sub("pwms", messageCb );
ros::Subscriber<trimode_control::FloatList> sub_wheel("pwms_wheel", messageCb_wheel );

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
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

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
  nh.subscribe(sub);
  nh.subscribe(sub_wheel);
}

void loop()
{       
  //str_msg.data = hello;
  pub.publish( &pwms );
  nh.spinOnce();
  delay(20);
}

/*

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
#include <IRremote.h>

//motors for robot motion
#define enA 10
#define in1 9
#define in2 8
#define enB 7
#define in3 6
#define in4 5

ros::NodeHandle nh;

trimode_control::FloatList pwms;
trimode_control::FloatList debug;

//电机部分
int pwm[2] = {10, 7};
int outputA1[2] = {9, 8};
int outputA2[2] = {6, 5};

//红外部分
int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;
long int command; //用于记录红外下达的指令
long int lastcommand; //用于记录上次红外下达的指令

int speed = 50; //pwm的速度
int speed_max = 180; //pwm上限



//函数定义部分，operate为红外遥控指令处理函数
void operate() {
  //其中有一部分用于变形控制：
  // 16716015 左 圆->爪
  // 16734885 右 爪->圆 16753245 逆时针 1  16736925 顺时针 2
  // 16726215 ok 暂停
  // 我们将使用其他进行前进电机的控制
  //


  if (command == 16718055) {
    //上 pwm加速
    if (speed < speed_max - 5)  speed += 5;
    else speed = speed_max;
    Serial.print("up current speed=");
    Serial.println(speed);
  }

  if (command == 16730805) {
    //下 pwm减速
    if (speed > 5)  speed -= 5;
    else speed = 0;
    Serial.print("down current speed=");
    Serial.println(speed);
  }

  if (command == 16738455) {
    //逆时针
    for (int i = 0; i < 2; i++) digitalWrite(outputA1[i], 1);
    for (int i = 0; i < 2; i++) digitalWrite(outputA2[i], 0);
    Serial.print("current speed=");
    Serial.println(speed);
    for (int i = 0; i < 2; i++) analogWrite(pwm[i], speed);
    digitalWrite(13, HIGH);   // blink the led
  }

  if (command == 16756815) {
    //顺时针
    for (int i = 0; i < 2; i++) digitalWrite(outputA1[i], 0);
    for (int i = 0; i < 2; i++) digitalWrite(outputA2[i], 1);
    Serial.print("current speed=");
    Serial.println(speed);
    for (int i = 0; i < 2; i++) analogWrite(pwm[i], speed);
    digitalWrite(13, HIGH);   // blink the led
  }

  if (command == 16726215) {
    //暂停
    for (int i = 0; i < 2; i++) digitalWrite(outputA1[i], 0);
    for (int i = 0; i < 2; i++) digitalWrite(outputA2[i], 0);
    for (int i = 0; i < 2; i++) analogWrite(pwm[i], 0);
    Serial.print("Stop!");
  }
}

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
ros::Publisher pub("arduino_debug", &debug);

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // 开启红外遥控

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
  debug.data[0] = results.value;
  debug.data[0] = 0;
  pub.publish( &debug );
  nh.spinOnce();


  if (irrecv.decode(&results)) {
    irrecv.resume(); // Receive the next value
  }
  //去除ffffffff，并作为command后调用operate函数。
  if (results.value != 4294967295 and results.value != 0) {
    command = results.value;
    Serial.println(command);
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  }
  if (command != lastcommand) operate();
  Serial.print("pwm[0]=");
  Serial.println(analogRead(pwm[0]));
  lastcommand = command;
  
  delay(20);
}

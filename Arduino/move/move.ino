/*
    Rotary Encoder - Polling Example
    
    The circuit:
    * encoder pin A to Arduino pin 2
    * encoder pin B to Arduino pin 3
    * encoder ground pin to ground (GND)
    
*/

#include <Rotary.h>

#define enAf 9
#define in1f 7
#define in2f 6
#define enBf 10
#define in3f 11
#define in4f 12

volatile long encoder0Pos=5800;
volatile long encoder1Pos=5800;   
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
float vel;
float pos;

void setup() {

  pinMode(enAf, OUTPUT);
  pinMode(in1f, OUTPUT);
  pinMode(in2f, OUTPUT);
  pinMode(enBf, OUTPUT);
  pinMode(in3f, OUTPUT);
  pinMode(in4f, OUTPUT);
  
  digitalWrite(in2f, HIGH);
  digitalWrite(in1f, LOW);
  digitalWrite(in3f, HIGH);
  digitalWrite(in4f, LOW);
}

void loop() {

    digitalWrite(in2f, HIGH);
    digitalWrite(in1f, LOW);
    analogWrite(enAf, 200); // Send PWM signal to L298N Enable pin
    analogWrite(enBf, 200); // Send PWM signal to L298N Enable pin  
}

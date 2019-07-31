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

Rotary r = Rotary(2, 3);
Rotary r1 = Rotary(4, 5);

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

  Serial.begin(9600);
  r.begin();
  PCICR |= (1 << PCIE2) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  sei();
}

void loop() {
  long newpos0 = encoder0Pos;
  long newpos1 = encoder1Pos;

//  float delta_pos = ((float)(newposition-oldposition);
  if (newpos0 > 0){
    digitalWrite(in2f, HIGH);
    digitalWrite(in1f, LOW);
    analogWrite(enAf, 200); // Send PWM signal to L298N Enable pin
    Serial.println("enc0: ");   
    Serial.println(newpos0);   
}
  else
  {
    analogWrite(enAf, 0); // Send PWM signal to L298N Enable pin
    Serial.println("enc0: ");   
    Serial.println(newpos0); 
  }

  if (newpos1 > 0){
    digitalWrite(in3f, HIGH);
    digitalWrite(in4f, LOW);
    analogWrite(enBf, 200); // Send PWM signal to L298N Enable pin
    Serial.println("enc1: ");       
    Serial.println(newpos1);   
}
  else
  {
    analogWrite(enBf, 0); // Send PWM signal to L298N Enable pin
    Serial.println("enc1: ");       
    Serial.println(encoder1Pos);      
  }
  
//  while (encoder0Pos > 0){
//    digitalWrite(in2f, LOW);
//    digitalWrite(in1f, HIGH);
//    analogWrite(enAf, 100); // Send PWM signal to L298N Enable pin    
//    Serial.println(encoder0Pos);
//  }
  
//    unsigned char result = r.process();
//  if (result) {
//    Serial.println(result == DIR_CW ? "Right" : "Left");
//  }
}

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  unsigned char result1 = r1.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) {
    Serial.println("ClockWise");
    encoder0Pos++;
  }
  else if (result == DIR_CCW) {
    Serial.println("CounterClockWise");
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

#include <Arduino.h>

// 1 ultrasonic sensor
int usTrigger = 1;
int usEcho = 2;

// 4 IR sensors to detect the outer line
int ir1 = 3;
int ir2 = 4;
int ir3 = 5;
int ir4 = 6;

// 2 motors
int ena1 = 9;   // left speed
int ena2 = 10;  // right speed
int in1 = 7;
int in2 = 8;
int in3 = 11;
int in4 = 12;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CyberTank");
  // ultrasonic sensor
  pinMode(usTrigger, OUTPUT);
  pinMode(usEcho, INPUT);
  // IR sensors
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  // motors
  pinMode(ena1, OUTPUT);
  pinMode(ena2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}

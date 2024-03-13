#include <Arduino.h>

// 1 ultrasonic sensor
#define usTrigger 3
#define usEcho 4

// 4 IR sensors to detect the outer line
#define ir1 2
#define ir2 4
#define ir3 5
#define ir4 6

// Left motor
#define ena1 6   // left speed, Digital PWM
#define in1  7
#define in2  8

// Right motor
#define ena2 9  // right speed, Digital PWM
#define in3  10
#define in4  11

// Constants
int pwmFrequency = 1000;
int threshold = 500;

void setupIR() {
  // IR sensors
  Serial.println("Setting up IR sensors");
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
}

void setupMotors() {
  // Motors
  Serial.println("Setting up motors");
  pinMode(ena1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void setupUltrasonic() {
  // Ultrasonic sensor
  Serial.println("Setting up ultrasonic sensor");
  pinMode(usTrigger, OUTPUT);
  pinMode(usEcho, INPUT);
}

void readIR() {
  int sensorValue = digitalRead(ir1); // seeing black = 1, seeing white = 0
  if (sensorValue == 1) {
    Serial.println("IR1 sees black");
  } else {
    Serial.println("IR1 sees white");
  }
}

void usScan() {
  digitalWrite(usTrigger, LOW);
  delayMicroseconds(5);
  digitalWrite(usTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrigger, LOW);
  long duration = pulseIn(usEcho, HIGH);
  int distance = duration * 0.034 / 2;
  Serial.print(distance);
  Serial.println(" cm");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CyberTank is setting up...");
  setupUltrasonic();
  Serial.println("CyberTank is ready!");
}

void loop() {

  // Black reflects less IR light, so sensor value will be lower
  usScan();

  delay(10); // Adjust delay based on your desired response speed

}

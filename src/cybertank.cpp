#include <Arduino.h>

// 1 ultrasonic sensor
#define usTrigger 3 // Digital PWM
#define usEcho 4

// 4 IR sensors to detect the outer line, analog IN
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3

// Left motor
#define ena1 6 // left speed, Digital PWM
#define in1 7
#define in2 8

// Right motor
#define ena2 9 // right speed, Digital PWM
#define in3 10
#define in4 11

// Constants
int pwmFrequency = 1000;
int threshold = 500;

void setupIR()
{
  // IR sensors
  Serial.println("Setting up IR sensors");
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
}

/**
 * Setup the motors
 */
void setupMotors()
{
  // Motors
  Serial.println("Setting up motors");
  pinMode(ena1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

/**
 * Setup the ultrasonic sensor
 */
void setupUltrasonic()
{
  // Ultrasonic sensor
  Serial.println("Setting up ultrasonic sensor");
  pinMode(usTrigger, OUTPUT);
  pinMode(usEcho, INPUT);
}

/**
 * Ping the IR sensors
 */
void pingIR()
{
  int sensorValue = digitalRead(ir1); // seeing black = 1, seeing white = 0
  if (sensorValue == 1)
  {
    Serial.println("IR1 sees black");
  }
  else
  {
    Serial.println("IR1 sees white");
  }
}

/**
 * Ping the ultrasonic sensor
 */
void pingUS()
{
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

/**
 * Stop the motors
 */
void stopMotors()
{
  Serial.println("Stopping motors");
  // Left motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Right motor
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void leftMotorForward(int speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena1, speed);
}

void rightMotorForward(int speed)
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ena2, speed);
}

void leftMotorBackward(int speed)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena1, speed);
}

void rightMotorBackward(int speed)
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena2, speed);
}

/**
 * Run the motors at the given speed
 * @param leftSpeed: 0x00 to 0xFF
 * @param rightSpeed: 0x00 to 0xFF
 */
void motorsForward(int leftSpeed, int rightSpeed)
{
  // Left motor
  Serial.print("Running forward: Left: ");
  Serial.print(leftSpeed);
  leftMotorForward(leftSpeed);

  // Right motor
  Serial.print(", Right: ");
  Serial.println(rightSpeed);
  rightMotorForward(rightSpeed);
}

void turnRight(int speed)
{
  Serial.println("Turning right");
  // Left motor continues forward
  leftMotorForward(speed);

  // Right motor going backward
  rightMotorBackward(speed);
}

void turnLeft(int speed)
{
  Serial.println("Turning left");
  // Right motor continues forward
  rightMotorForward(speed);

  // Left motor going backward
  leftMotorBackward(speed);
}

void testMotorSpeed()
{
  Serial.println("First loop");
  for (uint16_t i = 0x20; i <= 0xFF; i += 0x10)
  {
    motorsForward(i, i);
    delay(1000);
  }

  Serial.println("Second loop");
  for (uint16_t j = 0xFF; j >= 0x20; j -= 0x10)
  {
    motorsForward(j, j);
    delay(1000);
  }
  stopMotors();
  delay(5000);
}

void testLeftMotions(int speed)
{
  Serial.println("Test left motions");
  stopMotors();
  delay(100);
  leftMotorForward(speed);
  delay(2000);
  stopMotors();
  delay(100);
  leftMotorBackward(speed);
  delay(2000);
  stopMotors();
  delay(100);
}

void logicTest()
{
  Serial.println("Logic test");
  Serial.println("0 0 state");
  stopMotors();
  delay(1000);
  Serial.println("0 1 state");
  leftMotorForward(0x40);
  rightMotorForward(0x40);
  delay(2000);
  stopMotors();
  Serial.println("1 0 state");
  leftMotorBackward(0x40);
  rightMotorBackward(0x40);
  delay(2000);
  stopMotors();
  Serial.println("1 1 state");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  delay(2000);
  stopMotors();

  Serial.println("Turning right");
  rightMotorBackward(0x40);
  leftMotorForward(0x40);
  delay(2000);
}

void testTurningLeft(int speed)
{
  Serial.println("Test turning left");
  stopMotors();
  delay(100);
  motorsForward(speed, speed);
  delay(2000);
  stopMotors();
  delay(100);
  turnLeft(speed);
  delay(3000);
  stopMotors();
  delay(100);
  turnRight(speed);
  delay(1000);
}

/**
 * Main setup function
 */
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CyberTank is setting up...");
  // setupUltrasonic();
  setupMotors();
  Serial.println("CyberTank is ready!");
}

/**
 * Main loop function
 */
void loop()
{
  // testTurningLeft(40);
  logicTest();
  // testLeftMotions(40);
}

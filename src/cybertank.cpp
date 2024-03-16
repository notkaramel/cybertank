#include <Arduino.h>

// All controllers/sensors uses 5V
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

// Black = inside the zone, White = zone border
int minInZone = 300;

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
 * Read all the IR sensor
 * @return the value
*/
int readAllIR()
{
  int ir1Value = analogRead(ir1);
  int ir2Value = analogRead(ir2);
  int ir3Value = analogRead(ir3);
  int ir4Value = analogRead(ir4);
  Serial.print("IR1: ");
  Serial.print(ir1Value > minInZone ? "Black" : "White");
  Serial.print("\tIR2: ");
  Serial.print(ir2Value > minInZone ? "Black" : "White");
  Serial.print("\tIR3: ");
  Serial.print(ir3Value > minInZone ? "Black" : "White");
  Serial.print("\tIR4: ");
  Serial.println(ir4Value > minInZone ? "Black" : "White");
  Serial.println("----");
  return ir1Value;

}

/**
 * Ping the ultrasonic sensor
 * @return the distance in cm
 */
int getUSDistance()
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
  return distance;
}

/**
 * Stop the motors.
 */
void stopMotors()
{
  Serial.println("--- Stopping motors ---");
  // Left motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Right motor
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/**
 * Run the left motor forward
 * Don't call this function directly in main
 */
void leftMotorForward(uint8_t speed)
{
  Serial.print("Left motor forward at speed ");
  Serial.println(speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena1, speed);
}

/**
 * Run the left motor backward
 * Don't call this function directly in main
 */
void rightMotorForward(uint8_t speed)
{
  Serial.print("Right motor forward at speed ");
  Serial.println(speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ena2, speed);
}

/**
 * Run the left motor backward
 * Don't call this function directly in main
 */
void leftMotorBackward(uint8_t speed)
{
  Serial.print("Left motor backward at speed ");
  Serial.println(speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena1, speed);
}

/**
 * Run the right motor backward
 * Don't call this function directly in main
 */
void rightMotorBackward(uint8_t speed)
{
  Serial.print("Right motor backward at ");
  Serial.println(speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena2, speed);
}

/**
 * Run the motors at the given speed
 * @param leftSpeed: 0x00 to 0xFF
 * @param rightSpeed: 0x00 to 0xFF
 */
void goForward(uint8_t leftSpeed, uint8_t rightSpeed)
{
  // Left motor
  Serial.println("--- Running forward ---");
  leftMotorForward(leftSpeed);
  rightMotorForward(rightSpeed);
}

/**
 * Run the motors at the given speed
 * @param leftSpeed: 0x00 to 0xFF
 * @param rightSpeed: 0x00 to 0xFF
 */
void goBackward(uint8_t leftSpeed, uint8_t rightSpeed)
{
  // Left motor
  Serial.println("--- Running backward ---");
  leftMotorBackward(leftSpeed);
  rightMotorBackward(rightSpeed);
}

/**
 * Turn right
 * @param speed: 0x00 to 0xFF
 * @param turnDuration: in milliseconds
 */
void turnRight(uint8_t speed, int turnDuration)
{
  Serial.println("--- TURN RIGHT ---");
  stopMotors();
  delay(10);

  // only left motor
  goForward(speed, 0);
  delay(turnDuration / 2);

  stopMotors();
  delay(10);

  goBackward(0, speed);
  delay(turnDuration / 2);
  stopMotors();
}

void turnLeft(int speed, int turnDuration)
{
  Serial.println("--- TURN LEFT ---");
  stopMotors();
  delay(10);

  // only right motor
  goForward(0, speed);
  delay(turnDuration / 2);

  stopMotors();
  delay(10);

  goBackward(speed, 0);
  delay(turnDuration / 2);
  stopMotors();
}

/**
 * Main setup function
 */
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CyberTank is setting up...");
  setupUltrasonic();
  setupMotors();
  setupIR();
  Serial.println("CyberTank is ready!");
}

/**
 * Main loop function
 */
void loop()
{
  // uint8_t testSpeed = 0x32;
  // if(getUSDistance() < 10)
  // {
  //   stopMotors();
  //   delay(100);
  //   return;
  // }
  readAllIR();
  // goForward(testSpeed, testSpeed);
  delay(1000);
}

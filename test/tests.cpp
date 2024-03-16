#include "cybertank.cpp"

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
  goForward(speed, speed);
  delay(2000);
  stopMotors();
  delay(100);
  turnLeft(speed);
  delay(3000);
  stopMotors();
  delay(100);
//   turnRight(speed);
  delay(1000);
}


void testMotorSpeed()
{
  Serial.println("First loop");
  for (uint16_t i = 0x20; i <= 0xFF; i += 0x10)
  {
    goForward(i, i);
    delay(1000);
  }

  Serial.println("Second loop");
  for (uint16_t j = 0xFF; j >= 0x20; j -= 0x10)
  {
    goForward(j, j);
    delay(1000);
  }
  stopMotors();
  delay(5000);
}

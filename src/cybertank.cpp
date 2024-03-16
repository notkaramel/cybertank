#include <Arduino.h>

// All controllers/sensors uses 5V
// 1 ultrasonic sensor
#define usTrigger 3 // Digital PWM
#define usEcho 4

// 4 IR sensors to detect the outer line, analog IN
#define ir1 A0 // front
#define ir2 A1 // right (going clockwise)
#define ir3 A2 // back
#define ir4 A3 // left

// Control by the remote control of the organizer
#define remoteStart 13

// Left motor
#define ena1 6 // left speed, Digital PWM
#define in1 7
#define in2 8

// Right motor
#define ena2 9 // right speed, Digital PWM
#define in3 10
#define in4 11

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

void setupRemoteControl()
{
  // Remote control
  Serial.println("Setting up remote control");
  pinMode(remoteStart, INPUT);
}

/**
 * Read the remote control
 * @return 1 if "Start" button is pressed, 0 otherwise
 */
int isStartGame()
{
  return digitalRead(remoteStart);
}


/**
 * Read all the IR sensor
 * @return the value
*/
void readIRInZone()
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
}

bool isIRInZone()
{
  int ir1Value = analogRead(ir1);
  int ir2Value = analogRead(ir2);
  int ir3Value = analogRead(ir3);
  int ir4Value = analogRead(ir4);

  bool condition = ir1Value > minInZone || ir2Value > minInZone || ir3Value > minInZone || ir4Value > minInZone;
  Serial.println(condition ? "In zone" : "Out of zone");
  return condition;
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

void waitForGameStart()
{
  while(isStartGame() == 0)
  {
    Serial.print("Waiting for remote control to start the game...\r");
  }
  Serial.println("\nGame on!");
  // countdown
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("GO!");
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
  waitForGameStart();
}

/**
 * Main loop function
 */
void loop()
{
  if(isStartGame() == 0)
  {
    Serial.println("Game over!");
    stopMotors();
    delay(1000);
    endGame:
    goto endGame;
    // wait for the next game to start
  }
  // readIRInZone();
  // goForward(testSpeed, testSpeed);
  // isIRInZone();
  if(getUSDistance() < 30)
  {
    stopMotors();
    delay(1000);
    // turnRight(255, 1000);
  }
  else
  {
    goForward(50, 50);
  }
}

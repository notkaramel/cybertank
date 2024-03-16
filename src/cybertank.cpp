#include <Arduino.h>

// All controllers/sensors uses 5V
// 1 ultrasonic sensor
#define usTrigger 3 // Digital PWM
#define usEcho 4

// Left motor
#define ena1 6 // left speed, Digital PWM
#define in1 7
#define in2 8

// Right motor
#define ena2 9 // right speed, Digital PWM
#define in3 10
#define in4 11

// Control by the remote control of the organizer
#define remoteStart 13

// 4 IR sensors to detect the outer line, analog IN
#define ir1 A0 // front
#define ir2 A1 // right (going clockwise)
#define ir3 A2 // back
#define ir4 A3 // left

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
int readRemoteControl()
{
  return digitalRead(remoteStart);
}

// Black = inside the zone (digital 1), White = zone border (digital 0)
int minInZone = 450; // min : 0, max : 1023
/**
 * Read all the IR sensor
 * @return the value
 */
uint16_t readSideSensor()
{
  int ir1Value = analogRead(ir1);
  int ir2Value = analogRead(ir2);
  int ir3Value = analogRead(ir3);
  int ir4Value = analogRead(ir4);
  Serial.print("IR1: ");
  Serial.print(ir1Value);
  Serial.print("\tIR2: ");
  Serial.print(ir2Value);
  Serial.print("\tIR3: ");
  Serial.print(ir3Value);
  Serial.print("\tIR4: ");
  Serial.println(ir4Value);

  uint16_t sideSensor = 0B1111;
  if (ir1Value < minInZone)
  {
    sideSensor &= 0B0111;
  }
  if (ir2Value < minInZone)
  {
    sideSensor &= 0B1011;
  }
  if (ir3Value < minInZone)
  {
    sideSensor &= 0B1101;
  }
  if (ir4Value < minInZone)
  {
    sideSensor &= 0B1110;
  }
  Serial.print("Side sensor: ");
  Serial.println((char)sideSensor, BIN);
  return sideSensor;
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
void turnRightForward(uint8_t speed, int turnDuration)
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

void turnLeftForward(int speed, int turnDuration)
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

void turnRightBackward(int speed, int turnDuration)
{
  Serial.println("--- TURN RIGHT BACKWARD ---");
  stopMotors();
  delay(10);

  // only left motor
  goBackward(speed, 0);
  delay(turnDuration / 2);

  stopMotors();
  delay(10);

  goForward(0, speed);
  delay(turnDuration / 2);
  stopMotors();
}

void turnLeftBackward(int speed, int turnDuration)
{
  Serial.println("--- TURN LEFT BACKWARD ---");
  stopMotors();
  delay(10);

  // only right motor
  goBackward(0, speed);
  delay(turnDuration / 2);

  stopMotors();
  delay(10);

  goForward(speed, 0);
  delay(turnDuration / 2);
  stopMotors();
}

/* ------------------ ACTION AND LOGIC STUFF ------------------- */
// Some global variables
bool isSetup = false;
bool foundOpponent = false;
uint8_t minSpeed = 40;
uint8_t lowSpeed = 80;
uint8_t mediumSpeed = 124;
uint8_t highSpeed = 160;
uint8_t maxSpeed = 200; // 0xFF

/**
 * spagetti code :)
 * Handle cases when tank is at the edge of the zone.
 * 8 major cases: top, top-right, right, bottom-right, bottom, bottom-left, left, top-left
 * Corresponding binary value: 0B1000, 0B1100, 0B0100, 0B0110, 0B0010, 0B0011, 0B0001, 0B1001
 * @param sideSensor: the value of the side sensor in binary to handle the edge case
 */
void handleEdges(uint16_t sideSensor)
{
  int goDuration = 500;
  int turnDuration = 1000;
  uint8_t handlingSpeed = 80; // resolve the edge case at a lower speed, but must be swiftly
  Serial.print("Handling edge case: ");
  Serial.println(sideSensor, BIN);
  // each cases shouldn't take more than 1.5 second
  if (sideSensor == 0B1000 || sideSensor == 0B1100) // case 1 and 2
  {
    // top sensor sees the edge || top-right sensor sees the edge
    // back a little bit and turn right backward (left on backward, right off)
    goBackward(handlingSpeed, handlingSpeed);
    delay(goDuration); // might change

    goBackward(handlingSpeed, 0);
    delay(turnDuration);
  }
  else if (sideSensor == 0B0100) // case 3
  {
    // right sensor sees the edge, probably being pushed by the opponent
    // Give up?
    turnLeftForward(handlingSpeed, turnDuration);
    turnLeftForward(handlingSpeed, turnDuration);
    goForward(handlingSpeed, handlingSpeed);
    delay(goDuration - turnDuration);
  }
  else if (sideSensor == 0B0110) // case 4
  {
    // bottom-right
    goForward(0, handlingSpeed);
    delay(goDuration * 2);
  }
  else if (sideSensor == 0B0010)
  {
    // bottom
    goForward(handlingSpeed, handlingSpeed);
    delay(goDuration); // might change
  }
  else if (sideSensor == 0B0011)
  {
    // bottom-left
    goForward(handlingSpeed, 0);
    delay(goDuration * 2);
  }
  else if (sideSensor == 0B0001)
  {
    // left
    // Give up?
    turnRightForward(handlingSpeed, turnDuration);
    turnRightForward(handlingSpeed, turnDuration);
    goForward(handlingSpeed, handlingSpeed);
    delay(goDuration);
  }
  else if (sideSensor == 0B1001)
  {
    // top-left
    goBackward(handlingSpeed, handlingSpeed);
    delay(goDuration); // might change
    turnLeftBackward(handlingSpeed, turnDuration);
  }
  else
  {
    // in the zone
    Serial.println("In the zone! Nothing to handle");
    return; // return to the caller
  }
  Serial.println("Edge case handled!");
}

/**
 * Wait for the game to start (via remote control)
 */
void waitForGameStart()
{
  while (readRemoteControl() == 0)
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

void scanForOpponent(uint8_t scanSpeed, int scanTimeGap)
{
  // scan for opponent
  // if found, set foundOpponent to true

  while (getUSDistance() > 75)
  {
    // scan for opponent
    turnRightForward(scanSpeed, scanTimeGap);
    if (getUSDistance() <= 75)
    {
      // if found, set foundOpponent to true
      foundOpponent = true;
      break;
    }
  }
}

/**
 * Main setup function
 */
void setup()
{
  if (isSetup == false)
  {
    Serial.begin(9600);
    Serial.println("CyberTank is setting up...");
    setupRemoteControl();
    setupUltrasonic();
    setupMotors();
    setupIR();
    isSetup = true;
    Serial.println("CyberTank is ready!");
  }
  // waitForGameStart();
}

/**
 * Main loop function
 */
void loop()
{
  Serial.println(readRemoteControl());
  // if (readRemoteControl() == 0)
  // {
  //   Serial.println("Game over! Please reset the board.");
  //   stopMotors();
  //   delay(1000);
  //   setup();
  //   // wait for the next game to start
  // }

  // Making sure that the tank is in the zone. First priority!
  uint16_t sideSensor = readSideSensor();
  if (sideSensor != 0B1111)
  {
    handleEdges(sideSensor);
  }
  stopMotors();
  delay(500);

  int timeDelay = 5000;

  int TEST_MOTOR = 0;
  if (TEST_MOTOR == 1)
  {
    goForward(minSpeed, minSpeed);
    delay(timeDelay);
    stopMotors();
    delay(2000);
    goForward(lowSpeed, lowSpeed);
    delay(timeDelay);
    stopMotors();
    delay(2000);

    goForward(160, 160);
    delay(timeDelay);
    stopMotors();
    delay(2000);

    goForward(maxSpeed, maxSpeed);
    delay(timeDelay);
    stopMotors();
    delay(2000);
  }

  // IR zone sensor has higher priority
  // Step 1: scan for opponent
  if (foundOpponent == false)
  {

    // if opponent is not found, keep scanning
    // if found, set foundOpponent to true
  }
  else
  {
    // if opponent is found, attack
  }

  // Step 2: attack
  // if opponent is in range, attack
  // if not, move closer
}

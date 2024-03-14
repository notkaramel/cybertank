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

void setupUltrasonic()
{
  // Ultrasonic sensor
  Serial.println("Setting up ultrasonic sensor");
  pinMode(usTrigger, OUTPUT);
  pinMode(usEcho, INPUT);
}

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
 * Run the motors at the given speed
 * @param leftSpeed: 0x00 to 0xFF
 * @param rightSpeed: 0x00 to 0xFF
*/
void runMotors(uint8_t leftSpeed, uint8_t rightSpeed)
{
  // Left motor
  Serial.print("Running: Left: ");
  Serial.print(leftSpeed);
  analogWrite(ena1, leftSpeed);
  digitalWrite(in1, 0x1); // Forward
  digitalWrite(in2, 0x0);

  // Right motor
  Serial.print(", Right: ");
  Serial.println(rightSpeed);
  analogWrite(ena2, rightSpeed);
  digitalWrite(in3, 0x1);
  digitalWrite(in4, 0x0);
}

void stopMotors()
{
  Serial.println("Stopping motors");
  // Left motor
  analogWrite(ena1, 0x0);
  digitalWrite(in1, 0x0);
  digitalWrite(in2, 0x0);

  // Right motor
  analogWrite(ena2, 0x0);
  digitalWrite(in3, 0x0);
  digitalWrite(in4, 0x0);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CyberTank is setting up...");
  // setupUltrasonic();
  setupMotors();
  Serial.println("CyberTank is ready!");
}

void loop()
{
  // Black reflects less IR light, so sensor value will be lower
  Serial.println("First loop");
  for (uint16_t i = 0x20; i <= 0xFF; i += 0x10)
  {
    runMotors(i, i);
    delay(1000);
  }

  Serial.println("Second loop");
  for (uint16_t j = 0xFF; j >= 0x20; j -= 0x10)
  {
    runMotors(j, j);
    delay(1000);
  }
  stopMotors();
  delay(5000);
}

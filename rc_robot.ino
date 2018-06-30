#include <math.h>
#include <Wire.h>
#include <SoftwareSerial.h>
// Adafruit Motor Shield V2 Library - Version: 1.0.5
#include <Adafruit_MotorShield.h>

const uint16_t loopWaitTime = 1;

// Motors
const uint8_t maxSpeed = 255; // Any faster and it causes brownout :(
const uint8_t minSpeed = 0; // Min speed other than full stop
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Bluetooth
SoftwareSerial bluetooth(4, 5); // RX, TX
const uint8_t controllerInputBufferSize = 2;
byte controllerInputBuffer[controllerInputBufferSize];
#define AXIS_X (int8_t)controllerInputBuffer[0]
#define AXIS_Y (int8_t)controllerInputBuffer[1]
uint8_t controllerInputBufferCurrentPosition = 0;
byte tempData = 0;
bool receiving = false;
const byte beginMessage = -127;
//const byte endMessage = 127;
// -128 reserved for future use

// Joystick
const int8_t deadzoneRadius = 24;
const int8_t maxMagnitude = 126; // -128, -127, 127 reserved for serial protocol
const double spinHalfWidth = 0.4; // Offset in radians from 0 and pi in which robot is spinning
                                   // 0 < x << pi/2
const double m1 = 1 / spinHalfWidth;
const double m2 = 1 / (PI / 2 - spinHalfWidth);
const double intercept2 = 1 - PI * m2 / 2;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial ON...");
  bluetooth.begin(9600);
  bluetooth.println("Bluetooth ON...");

  AFMS.begin();  // create with the default frequency 1.6KHz

  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop() {
  if (bluetooth.available()) {
    tempData = bluetooth.read(); // Truncate to byte because we just called available() so we know we won't get -1
    if (receiving == false && tempData == beginMessage) {
      receiving = true;
      controllerInputBufferCurrentPosition = 0;
    }
    else if (receiving == true) {
      controllerInputBuffer[controllerInputBufferCurrentPosition] = tempData;
      controllerInputBufferCurrentPosition++;
      if (controllerInputBufferCurrentPosition >= controllerInputBufferSize) {
        receiving = false;
        controllerInputBufferCurrentPosition = 0;
        setMotorSpeedsFromJoystick(AXIS_X, AXIS_Y);
      }
    }
  }

//  delay(loopWaitTime);
}

void setMotorSpeedsFromJoystick(int8_t x, int8_t y) {
  const double theta = atan2(y, x); // [-PI, PI]
  const double r = constrain(sqrt(pow(x, 2) + pow(y, 2)), deadzoneRadius, maxMagnitude);
  const double rMappedToMotorSpeed = fmap(r, deadzoneRadius, maxMagnitude, minSpeed, maxSpeed);
  double leftMultiplier = 0; // [-1, 1]
  double rightMultiplier = 0; // [-1, 1]

//  double leftVelocity = cos(theta + PI / 4);
//  double rightVelocity = -sin(theta + PI / 4);
//  leftMotor->setSpeed(fabs(leftVelocity * rMappedToMotorSpeed));
//  leftMotor->run((leftVelocity >= 0) ? FORWARD : BACKWARD);
//  rightMotor->setSpeed(fabs(rightVelocity * rMappedToMotorSpeed));
//  rightMotor->run((rightVelocity >= 0) ? FORWARD : BACKWARD);

  if (theta <= 0) {
    // FORWARD
    leftMultiplier = getRamp(theta + PI);
    rightMultiplier = getRamp(-theta);
  }
  else {
    // BACKWARD
    leftMultiplier = -getRamp(theta);
    rightMultiplier = -getRamp(PI - theta);
  }

  leftMotor->setSpeed(fabs(leftMultiplier * rMappedToMotorSpeed));
  leftMotor->run((leftMultiplier >= 0) ? FORWARD : BACKWARD);
  rightMotor->setSpeed(fabs(rightMultiplier * rMappedToMotorSpeed));
  rightMotor->run((rightMultiplier >= 0) ? FORWARD : BACKWARD);

//  Serial.print("(");
//  Serial.print(x);
//  Serial.print(", ");
//  Serial.print(y);
//  Serial.print(")\t{");
//  Serial.print((int)round(r));
//  Serial.print(", ");
//  Serial.print((int)round(theta * 180 / PI));
//  Serial.print(" deg}\t[");
//  Serial.print(leftMultiplier);
//  Serial.print(", ");
//  Serial.print(rightMultiplier);
//  Serial.println("]");
}

// Input: [0, PI]
// Output: [-1, 1], follows straight line segments between (theta, r):
// (0, -1), (spinHalfWidth, 0), (PI / 2, 1), (PI, 1)
double getRamp(double theta) {
  if (theta < 0) {
    Serial.println("ERROR: theta should not be < 0");
    return 0;
  }
  else if (theta <= spinHalfWidth) {
    return m1 * theta - 1;
  }
  else if (theta <= PI / 2) {
    return m2 * theta + intercept2;
  }
  else if (theta <= PI) {
    return 1;
  }
  else {
    Serial.println("ERROR: theta should not be > PI");
    return 0;
  }
}

double fmap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#include<Servo.h>
#include <PID_v1.h>

const int thrPinRC = A0, thrPinOut = 3;
const int yawPinRC = A1, yawPinOut = 5;
const int pitchPinRC = A2, pitchPinOut = 6;
const int rollPinRC = A3, rollPinOut = 10;
const int armPinRC = A4, armPinOut = 11;
const int echoPin = A5, trigPin = 8;

double dist, setDist = 100, thrVal;

Servo thrServo, yawServo, rollServo, pitchServo, armServo;
PID thrPID(&dist, &setDist, &thrVal, 2, 5, 1, REVERSE);

void setup(){
//  Serial.begin(9600);
  thrServo.attach(thrPinOut);
  yawServo.attach(yawPinOut);
  rollServo.attach(rollPinOut);
  pitchServo.attach(pitchPinOut);
  armServo.attach(armPinOut);
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//  dist = readSensor();
//  thrPID.SetOutputLimits(1000, 2000);
//  thrPID.SetMode(AUTOMATIC);
//  thrServo.writeMicroseconds(1070);
//  armServo.writeMicroseconds(2000);
}

void loop(){
  thrServo.writeMicroseconds(pulseIn(thrPinRC, HIGH, 25000));
  armServo.writeMicroseconds(pulseIn(armPinRC, HIGH, 25000));
  yawServo.writeMicroseconds(pulseIn(yawPinRC, HIGH, 25000));
  rollServo.writeMicroseconds(pulseIn(rollPinRC, HIGH, 25000));
  pitchServo.writeMicroseconds(pulseIn(pitchPinRC, HIGH, 25000));
//  dist = readSensor();
//  thrPID.Compute();
//  thrServo.writeMicroseconds(thrVal);
//  Serial.print(dist);
//  Serial.print("\t");
//  Serial.println(thrVal);
//  delay(100);
}

long msToCm(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long readSensor(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  return msToCm(pulseIn(echoPin, HIGH, 25000));
}
  

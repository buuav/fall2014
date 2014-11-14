#include <PID_v1.h>

#include <Servo.h>

const int rcPin1 = A0;
const int rcPin2 = A1;
const int rcPin3 = A2;
const int rcPin4 = A3;
const int rcPin5 = A4;

const int xBeePin = 0;

const int multiWiiPin1 = 5;
const int multiWiiPin2 = 6;
const int multiWiiPin3 = 9;
const int multiWiiPin4 = 10;
const int multiWiiPin5 = 11;

Servo multiWiiThrottle;
Servo multiWiiYaw;
Servo multiWiiPitch;
Servo multiWiiRoll;
Servo multiWiiAux;

const int GROUNDED = 0;
const int MANUALMODE = 1;
const int AUTO = 2;

const int LAST = 0;
const int CURRENT = 1;

const long AUX_THRESHOLD = 1700;

int _mode;
int _oldMode;
boolean _stateChanged;

const int echoPin = A5, trigPin = 8;
double dist = 0, setDist = 60, thrVal = 0;
const int sampleTime = 500;  // Sample time in ms

PID thrPID(&dist, &thrVal, &setDist, 0.349, 0.0003062, 1, DIRECT);

struct Inputs{
  long rcPin1;
  long rcPin2;
  long rcPin3;
  long rcPin4;
  long rcPin5;
  char xBee;
  boolean hasXBee;
};

Inputs _inputs;

struct State{
  float Z;
  float Yaw;
  float Pitch;
  float Roll;
};

State _state;

void setup()
{
  Serial.begin(9600);
  rcSetup();
  xBeeSetup();
  sensorSetup();
  multiWiiSetup();
  
  _mode = GROUNDED;
  _oldMode = GROUNDED;
  _stateChanged = true;
}

void rcSetup()
{
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);
  pinMode(rcPin4, INPUT);
  pinMode(rcPin5, INPUT);
}

void xBeeSetup(){
  pinMode(xBeePin, INPUT);
}

void multiWiiSetup(){
  multiWiiThrottle.attach(multiWiiPin1);
  multiWiiThrottle.writeMicroseconds(1000);
  
  multiWiiYaw.attach(multiWiiPin4);
  multiWiiYaw.writeMicroseconds(1500);
  
  multiWiiPitch.attach(multiWiiPin3);
  multiWiiPitch.writeMicroseconds(1500);
  
  multiWiiRoll.attach(multiWiiPin2);
  multiWiiRoll.writeMicroseconds(1500);
  
  multiWiiAux.attach(multiWiiPin5);
  multiWiiAux.writeMicroseconds(0);
}



void sensorSetup(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  readSensor(&dist);
  thrPID.SetOutputLimits(1250, 1750);
  thrPID.SetSampleTime(sampleTime);
}

void loop()
{
  updateInputs();
  updateMode();
  updateState();
  switch (getMode()){
    case GROUNDED:
      groundedAct();
      break;
    case MANUALMODE:
      ManualAct();
      break;
    default: //AUTO
      autoAct();
  }
}

void updateMode()
{
  _oldMode = _mode;
  
  struct Inputs inputs = getInputs();
  
  if (_oldMode == GROUNDED){
     if (AUX_THRESHOLD <= inputs.rcPin5){
       _mode = MANUALMODE; 
     } else {
       _mode = GROUNDED;
     }
  }
  else if (_oldMode == MANUALMODE){
    if (inputs.rcPin5 < AUX_THRESHOLD && inputs.rcPin2 <= 1100){
      _mode = GROUNDED;
    } else if (inputs.hasXBee && inputs.xBee == '1') {
      _mode = AUTO;
    } 
  } else { //AUTO
    if (inputs.hasXBee && inputs.xBee == '0'){
      _mode = MANUALMODE;
    }
  }
  
  if(_oldMode != _mode){
    _stateChanged = true;
    switch(_mode){
      case GROUNDED:
        sendDisarm();
        break;
      case MANUALMODE:
        sendArm();
        break;
      default: //AUTO
        sendAuto();
    }
  }
}

int getMode(){
  return _mode;
}

void updateState(){
  
}

struct State getState(){
  return _state;
}

void groundedAct(){
  if(_stateChanged){
    Serial.println("grounded...");
    _stateChanged = false;
  }
}

void ManualAct(){
  if(_stateChanged){
    Serial.println("MANUALMODE flying...");
    _stateChanged = false;
  }
  
  struct Inputs inputs = getInputs();
  
  multiWiiAux.writeMicroseconds(inputs.rcPin5);
  multiWiiThrottle.writeMicroseconds(inputs.rcPin2);
  multiWiiYaw.writeMicroseconds(inputs.rcPin1);
  multiWiiPitch.writeMicroseconds(inputs.rcPin3);
  multiWiiRoll.writeMicroseconds(inputs.rcPin4);
}

void autoAct(){
  if(_stateChanged){
    Serial.println("auto flying...");
    _stateChanged = false;
  }
  struct Inputs inputs = getInputs();
  
  readSensor(&dist);
  thrPID.Compute();
  Serial.print("dist:");Serial.print(dist);Serial.print("\t");
  Serial.print("setDist:");Serial.print(setDist);Serial.print("\t");
  Serial.print("thrVal:");Serial.println(thrVal); 
  delay(sampleTime);
  multiWiiThrottle.writeMicroseconds(thrVal);
  multiWiiYaw.writeMicroseconds(inputs.rcPin1);
  multiWiiPitch.writeMicroseconds(inputs.rcPin3);
  multiWiiRoll.writeMicroseconds(inputs.rcPin4);
  
}

void updateInputs(){
  _inputs.rcPin1 = pulseIn(rcPin1, HIGH);
  _inputs.rcPin2 = pulseIn(rcPin2, HIGH);
  _inputs.rcPin3 = pulseIn(rcPin3, HIGH);
  _inputs.rcPin4 = pulseIn(rcPin4, HIGH);
  _inputs.rcPin5 = pulseIn(rcPin5, HIGH);
  
  if (Serial.available() > 0){
    _inputs.hasXBee = true;
    _inputs.xBee = Serial.read();
  }
  else {
    _inputs.hasXBee = false;
  }  
}

struct Inputs getInputs(){
  return _inputs;
}

void sendDisarm(){
  Serial.println("disarming");
  
  struct Inputs inputs = getInputs();
  multiWiiAux.writeMicroseconds(inputs.rcPin5);
  multiWiiThrottle.writeMicroseconds(inputs.rcPin2);
  thrPID.SetMode(MANUAL);
}

void sendArm(){
  Serial.println("arming");

  struct Inputs inputs = getInputs();
  multiWiiAux.writeMicroseconds(inputs.rcPin5);
  multiWiiThrottle.writeMicroseconds(inputs.rcPin2);
  multiWiiYaw.writeMicroseconds(inputs.rcPin1);
  multiWiiPitch.writeMicroseconds(inputs.rcPin3);
  multiWiiRoll.writeMicroseconds(inputs.rcPin4);
  
  dist = setDist;
  thrPID.SetMode(MANUAL);
}

void sendAuto(){
  Serial.println("auto");
  thrPID.SetMode(AUTOMATIC); 
}

long readSensor(double *dist){
  double numReadings = 2.0;
  long distSum = 0;
  for(int i=0; i<numReadings; i++){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    *dist = pulseIn(echoPin, HIGH, 25000)/29/2*10/9;
    *dist = smooth(pulseIn(echoPin, HIGH, 25000)/29/2, 0.6, *dist);
    distSum += pulseIn(echoPin, HIGH, 25000) / 29 / 2;
  }
  Serial.print("dist:");Serial.println((double)*dist);
  //*dist = constrain(distSum / numReadings, 0, 150);
}


int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}




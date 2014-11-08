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
const int MANUAL = 1;
const int AUTO = 2;

const int LAST = 0;
const int CURRENT = 1;

const long AUX_THRESHOLD = 1700;

int _mode;
int _oldMode;
boolean _stateChanged;

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
    case MANUAL:
      manualAct();
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
       _mode = MANUAL; 
     } else {
       _mode = GROUNDED;
     }
  }
  else if (_oldMode == MANUAL){
    if (inputs.rcPin5 < AUX_THRESHOLD && inputs.rcPin2 <= 1100){
      _mode = GROUNDED;
    } else if (inputs.hasXBee && inputs.xBee == '1') {
      _mode = AUTO;
    } 
  } else { //AUTO
    if (!inputs.hasXBee || inputs.xBee != '1'){
      _mode = MANUAL;
    }
  }
  
  if(_oldMode != _mode){
    _stateChanged = true;
    switch(_mode){
      case GROUNDED:
        sendDisarm();
        break;
      case MANUAL:
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

void manualAct(){
  if(_stateChanged){
    Serial.println("manual flying...");
    _stateChanged = false;
  }
  
  multiWiiAux.writeMicroseconds(_inputs.rcPin5);
  multiWiiThrottle.writeMicroseconds(_inputs.rcPin2);
  multiWiiYaw.writeMicroseconds(inputs.rcPin1);
  multiWiiPitch.writeMicroseconds(inputs.rcPin3);
  multiWiiRoll.writeMicroseconds(inputs.rcPin4);
}

void autoAct(){
  if(_stateChanged){
    Serial.println("auto flying...");
    _stateChanged = false;
  }
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
}

void sendArm(){
  Serial.println("arming");

  struct Inputs inputs = getInputs();
  multiWiiAux.writeMicroseconds(inputs.rcPin5);
  multiWiiThrottle.writeMicroseconds(inputs.rcPin2);
  multiWiiYaw.writeMicroseconds(inputs.rcPin1);
  multiWiiPitch.writeMicroseconds(inputs.rcPin3);
  multiWiiRoll.writeMicroseconds(inputs.rcPin4);
  
}

void sendAuto(){
  Serial.println("auto"); 
}



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

const int GROUNDED = 0;
const int MANUAL = 1;
const int AUTO = 2;

const int LAST = 0;
const int CURRENT = 1;

const long AUX_THRESHOLD = 2000;//?

int _mode;
int _oldMode;

struct Inputs{
  long rcPin1,
  long rcPin2,
  long rcPin3,
  long rcPin4,
  long rcPin5
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
  rcSetup();
  xBeeSetup();
  sensorSetup();
  
  _mode = GROUNDED;
  _oldMode = GROUNDED;
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

void sensorSetup(){
  
}

void loop()
{
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
  
  long throttle = pulseIn(rcPin2, HIGH);
  long aux = pulseIn(rcPin5, HIGH);
    
  long isAuto = digitalRead(xBeePin);
  
  if (_oldMode == GROUNDED){
     if (AUX_THRESHOLD <= aux){
       _mode = MANUAL; 
     } else {
       _mode = GROUNDED;
     }
  }
  else if (_oldMode == MANUAL){
    if (aux < AUX_THRESHOLD && throttle == 0){
      _mode = GROUNDED;
    } else if (isAuto == HIGH) {
      _mode = AUTO;
    } 
  } else { //AUTO
    if (isAuto == LOW){
      _mode = MANUAL;
    }
  }
}

int getMode(){
  return _mode;
}

void updateState(){
  
}

State getState(){
  return _state;
}

void groundedAct(){
  
}

void manualAct(){
  
}

void autoAct(){
  
}

void updateInputs()

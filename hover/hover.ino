const int rcPin1 = 0;
const int rcPin2 = 0;
const int rcPin3 = 0;
const int rcPin4 = 0;
const int rcPin5 = 0;

const int xBeePin = 0;

const int GROUNDED = 0;
const int MANUAL = 1;
const int AUTO = 2;

const int LAST = 0;
const int CURRENT = 1;

const long AUX_THRESHOLD = 2000;
const int A

int _mode;
int _oldMode;

void setup()
{
  rcSetup();
  xBeeSetup();
  
  mode = GROUNDED;
  oldMode = GROUNDED;
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
  
  if (oldMode == GROUNDED){
     if (auxThreshold <= aux){
       _mode = MANUAL; 
     } else {
       _mode = GROUNDED;
     }
  }
  else if (oldMode == MANUAL){
    if (aux < AUX_THRESHOLD && throttle = 0){
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

void groundedAct(){
  
}

void manualAct(){
  
}

void autoAct(){
  
}



#include <PID_v1.h>
#include <Servo.h>

const int rcPin[5] = {A0, A1, A2, A3, A4};
const int echoPin = A5, trigPin = 8;
const int wiiPin[5] = {5, 6, 9, 10, 11};

Servo wiiThrottle, wiiYaw, wiiPitch, wiiRoll, wiiAux;

bool isAuto = false; 
const int AUX_LOW = 1600, AUX_HIGH = 1800;
double dist, setDist, thrVal;
const int sampleTime = 20, serialPrintDelay = 200;    // Sample time in ms
unsigned long lastPrintTime = 0;

PID thrPID(&dist, &thrVal, &setDist, 3.5, 1, 0.8, DIRECT);

void setup(){
    Serial.begin(9600);
    pinsSetup();
    PIDSetup();
    wiiSetup();
}

void pinsSetup(){
    for(int i=0; i<5; i++)  pinMode(rcPin[i], INPUT);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
}

void wiiSetup(){
    wiiThrottle.attach(wiiPin[0]);  wiiThrottle.writeMicroseconds(1000);
    wiiYaw.attach(wiiPin[3]);       wiiYaw.writeMicroseconds(1500);  
    wiiPitch.attach(wiiPin[2]);     wiiPitch.writeMicroseconds(1500);
    wiiRoll.attach(wiiPin[1]);      wiiRoll.writeMicroseconds(1500);
    wiiAux.attach(wiiPin[4]);       wiiAux.writeMicroseconds(1850);
}

void PIDSetup(){
    thrPID.SetOutputLimits(1050,1850);
    thrPID.SetSampleTime(sampleTime);
}

void loop(){
    int aux = pulseIn(rcPin[4], HIGH);
    if(aux>AUX_HIGH){
      if(!isAuto){
        isAuto = true;
        thrVal = pulseIn(rcPin[1], HIGH);
        dist = readSensor(true, dist);
        setDist = dist;
        thrPID.SetMode(AUTOMATIC);
      }
    } else if(aux<AUX_LOW) {
      if(isAuto){
        isAuto = false;
        thrPID.SetMode(MANUAL);
      }
    }
    wiiAux.writeMicroseconds(1850);
    wiiYaw.writeMicroseconds(pulseIn(rcPin[0], HIGH));
    wiiPitch.writeMicroseconds(pulseIn(rcPin[2], HIGH));
    wiiRoll.writeMicroseconds(pulseIn(rcPin[3], HIGH));
    unsigned long now = millis();
    dist = readSensor(true, dist);
    
    if(isAuto){
        if(thrPID.Compute()){
            wiiThrottle.writeMicroseconds(thrVal);
        }
        if((now - lastPrintTime) > serialPrintDelay){
            Serial.print((int)dist);Serial.print("\t");
            Serial.print((int)setDist);Serial.print("\t");
            Serial.println((int)thrVal); 
            lastPrintTime = now;
        }
    }
    else{
      wiiThrottle.writeMicroseconds(pulseIn(rcPin[1], HIGH));
      if((now - lastPrintTime) > serialPrintDelay){
          Serial.println((int)dist);
          lastPrintTime = now;
      }
    }
}
    
void serialEvent(){
    char inChar[2];
    Serial.readBytes(inChar, Serial.available());
    setDist = atoi(inChar);
}
    
long readSensor(boolean filter, double dist){
    digitalWrite(trigPin, LOW);     delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    if(filter)      return smooth(pulseIn(echoPin, HIGH, 25000)/29/2*10/9, 0.6, dist);
    else            return pulseIn(echoPin, HIGH, 25000)/29/2*10/9;
}

int smooth(int data, float filterVal, float smoothedVal){
  // Higher filter values cause more filtering, slower response
    filterVal = constrain(filterVal, 0, 1.0);
    smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

    return (int)smoothedVal;
}

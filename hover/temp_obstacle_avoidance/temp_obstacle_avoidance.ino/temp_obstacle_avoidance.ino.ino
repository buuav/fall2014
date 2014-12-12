#include <PID_v1.h>
#include <Servo.h>

const int AUX_THRESHOLD[2] = {
  1600, 1800};          // Low and high aux values with deadzone in between
const double filterValue = 0.6;                     // Amount of smoothing in the ultrasonic sensor low pass filter
const bool hoverOnly = false;
const double e = 2.7182;

const int rcPin[4] = {
  A0, A1, A2, A3};  // {Throttle-Yellow, Pitch-Black, Roll-Green, Aux-Orange} {Yaw-Orange is direct from RC to Multiwii}
const int echoPin[5] = {
  2, 3, 4, 5, 6}
, trigPin[2] = {7, 8}; // {Down-Green, front, right, rear, left} with common trigger-yellow
const int wiiPin[4] = {
  10, 11, 12, 13};    // {Throttle-Yellow, Pitch-Black, Roll-Green, Aux-Orange}

Servo wiiThrottle, wiiPitch, wiiRoll, wiiAux;

double usReading[5];
int rcReading[4];
int kP = 3.5, kI = 0.8, kD = 2;

unsigned long now;
const int samplingDelay[4] = {
  30, 30, 100, 200};   // Time delays in the order {Control, ultrasound, RC, print}
unsigned long timeStamp[4];                         // Time since last occurance in same order as above.

bool isAuto = false; 
double dist, setDist, thrVal;                       // PID input, setpoints and output variables

PID thrPID(&dist, &thrVal, &setDist, 3.1, 1, 0.6, DIRECT);

void setup(){
  Serial.begin(9600);
  pinsSetup();
  PIDSetup();   
  wiiSetup();
}
	
void pinsSetup(){
  for(int i=0; i<4; i++)  pinMode(rcPin[i], INPUT);
  for(int i=0; i<5; i++)  pinMode(echoPin[i], INPUT);
  for(int i=0; i<2; i++)	pinMode(trigPin[i], OUTPUT);
  pinMode(A5, OUTPUT);     
  digitalWrite(A5, HIGH);				// This pin provides power to the RC receiver
  pinMode(9, OUTPUT);
	digitalWrite(9, HIGH);				// This pin provides power to the downfacing uS sensor
}

void wiiSetup(){
  wiiThrottle.attach(wiiPin[0]);
  wiiPitch.attach(wiiPin[1]);
  wiiRoll.attach(wiiPin[2]);
  wiiAux.attach(wiiPin[3]);   
  wiiAux.writeMicroseconds(2000);     // Since this is permanently armed
}

void PIDSetup(){
  thrPID.SetOutputLimits(1050,1950);
  thrPID.SetSampleTime(samplingDelay[0]);
}

void loop(){
  now = millis();

  // Update all ultrasound readings
  if((now - timeStamp[1]) > samplingDelay[1]){
    unsigned int temp = readSensor(trigPin[1], echoPin[0]);
    if(abs(temp - usReading[0]) < 5)  usReading[0] = smooth(temp, filterValue, usReading[0]);
    else if(temp>usReading[0])  usReading[0] += 5;
    else usReading[0] -= 5;
    if(isAuto) dist = usReading[0];
    if(!hoverOnly){
      digitalWrite(trigPin[0], LOW);		delayMicroseconds(2);
			digitalWrite(trigPin[0], HIGH);		delayMicroseconds(10);
			digitalWrite(trigPin[0], LOW);
      uint32_t endTime[4];
			uint8_t lastState = (PIND>>1)&B00001111;
			uint32_t timeOut = micros() + 5800;
			for(uint8_t i=0; i<4; i++)	endTime[i] = timeOut;
			uint32_t startTime = micros();
			while(micros()<timeOut){
				for(uint8_t i=0; i<4; i++)
					if((((PIND>>1)&B00001111)^lastState)&(1<<i) & lastState)
						endTime[i] = micros();
				lastState = (PIND>>1)&B00001111;
			}
			for(int i=0; i<4; i++)
					usReading[i+1] = (endTime[i]-startTime) / 58;
		}
    timeStamp[1] = now;
  }

  // Update all RC inputs
  if((now - timeStamp[2]) > samplingDelay[2]){
    for(int i=0; i<4; i++)
      rcReading[i] = pulseIn(rcPin[i], HIGH);

    // Check if Aux has been flipped and change mode between manual and auto
    if(rcReading[3]>AUX_THRESHOLD[1] && !isAuto){
      isAuto = true;
      thrVal = rcReading[0]; 
      dist = usReading[0];
      setDist = dist;
      // To eliminate jerk, match input and output of PID to the current values before switching it on
      thrPID.SetMode(AUTOMATIC);
    } 
    else if(rcReading[3]<AUX_THRESHOLD[0] && isAuto) {
      isAuto = false;
      thrPID.SetMode(MANUAL);
    }
    timeStamp[2] = now;
  }

  // Output commands to the MultiWii
  if((now - timeStamp[0]) > samplingDelay[0]){
    if(isAuto){
      thrPID.Compute();
      wiiThrottle.writeMicroseconds(thrVal);
      if(hoverOnly){
        wiiPitch.writeMicroseconds(rcReading[1]);
        wiiRoll.writeMicroseconds(rcReading[2]);
      } 
      else {
        wiiPitch.writeMicroseconds(rcReading[1] + 100*(pow(e, -0.07*usReading[3]) - pow(e, -0.07*usReading[1])));
        wiiRoll.writeMicroseconds(rcReading[2] + 100*(pow(e, -0.07*usReading[4]) - pow(e, -0.07*usReading[2])));
      }
    } 
    else {
      wiiThrottle.writeMicroseconds(rcReading[0]);
      wiiRoll.writeMicroseconds(rcReading[2]);
      wiiPitch.writeMicroseconds(rcReading[1]);
    }
    timeStamp[0] = now;
  }

  // Print output on serial line
  if((now - timeStamp[3]) > samplingDelay[3]){
//    if(isAuto){
//      Serial.print((int)usReading[0]);        
//      Serial.print("\t");
//      Serial.print((int)setDist);             
//      Serial.print("\t");
//      Serial.println((int)thrVal); 
//    } 
//    else   Serial.println((int)usReading[0]);
    for(int i=0; i<5; i++){
      Serial.print((int)usReading[i]);
      Serial.print("\t");
    }
    Serial.println(""); 
    timeStamp[3] = now;
  }
}

void serialEvent(){
  char inChar[4];
  Serial.readBytes(inChar, Serial.available());
  char valChar[3];
  for(int i=0; i<3; i++)  valChar[i] = inChar[i+1];
  if(inChar[0] == 'p')  kP = atof(valChar);
  else if(inChar[0] == 'i')  kI = atof(valChar);
  else if(inChar[0] == 'd')  kD = atof(valChar);
  else if(inChar[0] == 'a')  setDist = atof(valChar);
  thrPID.SetTunings(kP, kI, kD);
  Serial.print("Tunings:");  
  Serial.print(kP);
  Serial.print("\t");  
  Serial.print(kI);
  Serial.print("\t");  
  Serial.print(kD);
  Serial.print("\t");  
  Serial.println(setDist);
}

double readSensor(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);     
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);    
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, 25000)/29/2*10/9;
}

double smooth(double data, double filterVal, double smoothedVal){
  // Higher filter values cause more filtering, slower response
  filterVal = constrain(filterVal, 0, 1.0);
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}


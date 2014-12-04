#include <PID_v1.h>
#include <Servo.h>
#include <NewPing.h>

const int AUX_THRESHOLD[2] = {1600, 1800};          // Low and high aux values with deadzone in between
const double filterValue = 0.6;                     // Amount of smoothing in the ultrasonic sensor low pass filter
const double e = 2.7182;

const int rcPin[4] = {A1, A2, A3, A4};  // {Throttle, Pitch, Roll, Aux}
const int wiiPin[4] = {5, 6, 9, 11};    // {Throttle, Roll, Pitch, Aux}

Servo wiiThrottle, wiiPitch, wiiRoll, wiiAux;
NewPing sonar[2] = {
  NewPing(8, A5, 200),
  NewPing(10, 3, 200)
};
uint32_t pingTimer[2];
uint8_t cm[2];
uint8_t currentSensor = 0;

int rcReading[4];
int avoidPitch = 0;
int kP = 3.5, kI = 1, kD = 0.8;

unsigned long now;
const int samplingDelay[4] = {30, 30, 100, 200};   // Time delays in the order {Control, ultrasound, RC, print}
unsigned long timeStamp[4];                         // Time since last occurance in same order as above.

bool isAuto = false; 
double dist, setDist, thrVal;                       // PID input, setpoints and output variables

PID thrPID(&dist, &thrVal, &setDist, 3.5, 1, 0.8, DIRECT);

void setup(){
    Serial.begin(9600);
    pinsSetup();
    PIDSetup();
    wiiSetup();
    pingTimer[1] = millis()+100;
    pingTimer[2] = pingTimer[1] + 30;
}

void pinsSetup(){
    for(int i=0; i<4; i++)  pinMode(rcPin[i], INPUT);
    pinMode(4, OUTPUT);  digitalWrite(4, HIGH);
}

void wiiSetup(){
    wiiThrottle.attach(wiiPin[0]);
    wiiPitch.attach(wiiPin[2]);
    wiiRoll.attach(wiiPin[1]);
    wiiAux.attach(wiiPin[3]);   wiiAux.writeMicroseconds(2000);     // Since this is permanently armed
}

void PIDSetup(){
    thrPID.SetOutputLimits(1050,1950);
    thrPID.SetSampleTime(samplingDelay[0]);
}

void loop(){
    now = millis();
    
    // Update all ultrasound readings
    for(int i=0; i<2; i++){
      if(now >= pingTimer[i]){
        pingTimer[i] += 60;
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;         // Sensor being accessed.
        if(currentSensor==1)  cm[currentSensor] = 200;
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }
        
    
    // Update all RC inputs
    if((now - timeStamp[2]) > samplingDelay[2]){
        for(int i=0; i<4; i++)
            rcReading[i] = pulseIn(rcPin[i], HIGH);
        
        // Check if Aux has been flipped and change mode between manual and auto
        if(rcReading[3]>AUX_THRESHOLD[1] && !isAuto){
            isAuto = true;
            thrVal = rcReading[0]; 
            dist = cm[0];
            setDist = dist;
            // To eliminate jerk, match input and output of PID to the current values before switching it on
            thrPID.SetMode(AUTOMATIC);
        } else if(rcReading[3]<AUX_THRESHOLD[0] && isAuto) {
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
            wiiRoll.writeMicroseconds(rcReading[2]);
            avoidPitch =100*(pow(e, -0.025*cm[1]));
            wiiPitch.writeMicroseconds(rcReading[1] + avoidPitch);
        } else {
            wiiThrottle.writeMicroseconds(rcReading[0]);
            wiiRoll.writeMicroseconds(rcReading[2]);
            wiiPitch.writeMicroseconds(rcReading[1]);
        }
        timeStamp[0] = now;
    }
    
    // Print output on serial line
    if((now - timeStamp[3]) > samplingDelay[3]){
        if(isAuto){
            Serial.print((int)cm[0]);          Serial.print("\t");
            Serial.print((int)setDist);               Serial.print("\t");
            Serial.print((int)thrVal);                Serial.print("\t\t"); 
            Serial.print((int)cm[1]);          Serial.print("\t");
            Serial.println((int)avoidPitch);
        } else {
            Serial.print((int)cm[0]);          Serial.print("\t");
            Serial.println((int)cm[1]);
        }
    timeStamp[3] = now;
    }
}
    
//void serialEvent(){
//    char inChar[4];
//    Serial.readBytes(inChar, Serial.available());
//    char valChar[3];
//    for(int i=0; i<3; i++)  valChar[i] = inChar[i+1];
//    if(inChar[0] == 'p')  kP = atof(valChar);
//    else if(inChar[0] == 'i')  kI = atof(valChar);
//    else if(inChar[0] == 'd')  kD = atof(valChar);
//    else if(inChar[0] == 'a')  setDist = atof(valChar);
//    thrPID.SetTunings(kP, kI, kD);
//    Serial.print("Tunings:");  Serial.print(kP);
//    Serial.print("\t");  Serial.print(kI);
//    Serial.print("\t");  Serial.print(kD);
//    Serial.print("\t");  Serial.println(setDist);
//}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()){
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    if(currentSensor == 0)  dist = cm[currentSensor];
  }
}
                
double readSensor(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);     delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH, 25000)/29/2*10/9;
}

double smooth(double data, double filterVal, double smoothedVal){
    // Higher filter values cause more filtering, slower response
    filterVal = constrain(filterVal, 0, 1.0);
    smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

    return smoothedVal;
}

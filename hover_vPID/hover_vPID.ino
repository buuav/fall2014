#include <PID_v1.h>
#include <Servo.h>

const double e = 2.7182;
const int rcPin[5] = {A1, A2, A3, A4};  // {Yaw, Throttle, Pitch, Roll, Aux}
const int echoPin[5] = {A5, 3, 4, 7, 8}, trigPin = 8; // {Down, front, right, rear, left} with common trigger
const int wiiPin[5] = {5, 6, 9, 11};    // {Throttle, Roll, Pitch, Yaw, Aux}

Servo wiiThrottle, wiiPitch, wiiRoll, wiiAux;

double usReading[5];
int rcReading[5];

unsigned long now;
const int samplingDelay[4] = {40, 40, 100, 200};   // Time delays in the order {Control, ultrasound, RC, print}
unsigned long timeStamp[4];                         // Time since last occurance in same order as above.

bool isAuto = false; 
const int AUX_THRESHOLD[2] = {1600, 1800};          // Low and high aux values with deadzone in between
const double filterValue = 0.6;                     // Amount of smoothing in the ultrasonic sensor low pass filter
double dist, setDist, thrVal;                       // PID input, setpoint and output variables

PID thrPID(&dist, &thrVal, &setDist, 3.5, 1, 0.8, DIRECT);

void setup(){
    Serial.begin(9600);
    pinsSetup();
    PIDSetup();
    wiiSetup();
}

void pinsSetup(){
    for(int i=0; i<5; i++)  pinMode(rcPin[i], INPUT);
    for(int i=0; i<5; i++)  pinMode(echoPin[i], INPUT);
    pinMode(trigPin, OUTPUT);
}

void wiiSetup(){
    wiiThrottle.attach(wiiPin[0]);
    wiiPitch.attach(wiiPin[2]);
    wiiRoll.attach(wiiPin[1]);
    wiiAux.attach(wiiPin[4]);   wiiAux.writeMicroseconds(2000);     // Since this is permanently armed
}

void PIDSetup(){
    thrPID.SetOutputLimits(1050,1950);
    thrPID.SetSampleTime(samplingDelay[0]);
}

void loop(){
    now = millis();
    
    // Update all ultrasound readings
    if((now - timeStamp[1]) > samplingDelay[1]){
        for(int i=0; i<5; i++)
            usReading[i] = smooth(readSensor(trigPin, echoPin[i]), filterValue, usReading[i]);
        if(isAuto)  dist = usReading[0];
        timeStamp[1] = now;
    }
    
    // Update all RC inputs
    if((now - timeStamp[2]) > samplingDelay[2]){
        for(int i=0; i<5; i++)
            rcReading[i] = pulseIn(rcPin[i], HIGH);
        
        // Check if Aux has been flipped and change mode between manual and auto
        if(rcReading[4]>AUX_THRESHOLD[1] && !isAuto){
            isAuto = true;
            thrVal = rcReading[1]; 
            dist = usReading[0];
            setDist = dist;
            // To eliminate jerk, match input and output of PID to the current values before switching it on
            thrPID.SetMode(AUTOMATIC);
        } else if(rcReading[4]<AUX_THRESHOLD[0] && isAuto) {
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
            wiiPitch.writeMicroseconds(rcReading[2] + 100*(pow(e, -0.07*usReading[3]) - pow(e, -0.07*usReading[1])));
            wiiRoll.writeMicroseconds(rcReading[3] + 100*(pow(e, -0.07*usReading[4]) - pow(e, -0.07*usReading[2])));
        } else {
            wiiThrottle.writeMicroseconds(rcReading[1]);
            wiiRoll.writeMicroseconds(rcReading[3]);
            wiiPitch.writeMicroseconds(rcReading[2]);
        }
        timeStamp[0] = now;
    }
    
    // Print output on serial line
    if((now - timeStamp[2]) > samplingDelay[2]){
        if(isAuto){
            Serial.print((int)dist);        Serial.print("\t");
            Serial.print((int)setDist);     Serial.print("\t");
            Serial.println((int)thrVal); 
        } else   Serial.println((int)dist);
    timeStamp[2] = now;
    }
}
    
void serialEvent(){
    char inChar[2];
    Serial.readBytes(inChar, Serial.available());
    setDist = atoi(inChar);
}
                
double readSensor(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);     delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH, 25000)/(29/2)*10/9;
}

double smooth(double data, double filterVal, double smoothedVal){
    // Higher filter values cause more filtering, slower response
    filterVal = constrain(filterVal, 0, 1.0);
    smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

    return smoothedVal;
}
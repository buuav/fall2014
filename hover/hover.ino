#include <PID_v1.h>
#include <Servo.h>

const int auxThreshold = 1750;                     // Threshold for auto hold switch
const double filterValue = 0.6;                     // Amount of smoothing in the ultrasonic sensor low pass filter
int kP = 1, kI = 0.08, kD = 1.8;

const int rcPin[5] = {A0, A1, A2, A3, A4};          // {THR, PIT, ROL, YAW, AUTO}
const int wiiPin[4] = {9, 10, 11, 12};              // {THR, PIT, ROL, YAW}
const int echoPin = 2, trigPin = 8;

Servo wiiServo[4];                                  // {THR, PIT, ROL, YAW}
int rcReading[4];                                   // {THR, PIT, ROL, YAW}

unsigned long now;
const int samplingDelay[4] = {30, 30, 100, 200};    // Time delays for {uS, Fast RC, Slow RC, Serial}
unsigned long timeStamp[4];                         // Time stamps for {uS, Fast RC, Slow RC, Serial}

bool isAuto = false; 
double dist, setDist, thrVal;                       // PID input, setpoints and output variables

PID thrPID(&dist, &thrVal, &setDist, kP, kI, kD, DIRECT);

void setup(){
    Serial.begin(115200);
    
    for(int i=0; i<5; i++)  pinMode(rcPin[i], INPUT);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);

    for(int i=0; i<4; i++)  wiiServo[i].attach(wiiPin[i]);

    thrPID.SetOutputLimits(1150,1850);
    thrPID.SetSampleTime(samplingDelay[0]);
}

void loop(){
    now = millis();

    // Update all ultrasound readings
    if((now - timeStamp[0]) > samplingDelay[0]){
        timeStamp[0] = now;
        dist += constrain((smooth(readSensor(trigPin, echoPin), filterValue, dist) - dist), -5, 5);
        if (isAuto){
            thrPID.Compute();
            wiiServo[0].writeMicroseconds(thrVal);
        }
    }
    
    now = millis();
    
    if((now - timeStamp[1]) > samplingDelay[1]){
        timeStamp[1] = now;
        for(int i=0; i<3; i++){
            rcReading[i] = pulseIn(rcPin[i], HIGH, 20000);
            if (!isAuto || i>0) wiiServo[i].writeMicroseconds(rcReading[i]);
        }
    }
    
    now = millis();
    
    if((now - timeStamp[2]) > samplingDelay[2]){
        timeStamp[2] = now;
        rcReading[3] = pulseIn(rcPin[3], HIGH, 20000);
        wiiServo[3].writeMicroseconds(rcReading[3]);
        boolean switchHigh = pulseIn(rcPin[4], HIGH, 20000) > auxThreshold;
        // Check if Aux has been flipped and change mode between manual and auto
        if(!isAuto && switchHigh){
            isAuto = true;
            thrVal = rcReading[0];
            setDist = dist;
            // To eliminate jerk, we match input and output of PID to the current values before switching it on
            thrPID.SetMode(AUTOMATIC);
        }
        else if(isAuto && !switchHigh) {
            isAuto = false;
            thrPID.SetMode(MANUAL);
        }
    }
    
    now = millis();

    // Print output on serial line
    if((now - timeStamp[3]) > samplingDelay[3]){
        timeStamp[3] = now;
        Serial.print((int)dist);           Serial.print("\t");
        Serial.print((int)setDist);        Serial.print("\t");
        Serial.print((int)thrVal);         Serial.print("\t");
        if(isAuto)  Serial.println("AUTO");
        else        Serial.println("MANUAL");
    }
}
    
double readSensor(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);     delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH, 25000)/29/2;
}

double smooth(double data, double filterVal, double smoothedVal){
    // Higher filter values cause more filtering, slower response
    filterVal = constrain(filterVal, 0, 1.0);
    return (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
}


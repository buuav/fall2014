#include <PID_v1.h>
#include <Servo.h>

const int AUX_THRESHOLD[2] = {1600, 1800};          // Low and high aux values with deadzone in between
const double filterValue = 0.6;                     // Amount of smoothing in the ultrasonic sensor low pass filter
int kP = 3.1, kI = 1, kD = 0.6;

const int rcPin[5] = {A0, A1, A2, A3, A4};          // {THR, PIT, ROL, YAW, AUX1}
const int wiiPin[5] = {9, 10, 11, 12, 13};             // {THR, PIT, ROL, YAW, AUX1}
const int echoPin = 2, trigPin = 8;

Servo wiiServo[5];                                  // {THR, PIT, ROL, YAW, AUX1}
int rcReading[5];                                   // {THR, PIT, ROL, YAW, AUX1}

unsigned long now;
const int samplingDelay[3] = {30, 30, 200};    // Time delays for {Control, uS, RC, Serial}
unsigned long timeStamp[3];                         // Time stamps for {Control, uS, RC, Serial}

bool isAuto = false; 
double dist, setDist, thrVal;                       // PID input, setpoints and output variables

PID thrPID(&dist, &thrVal, &setDist, kP, kI, kD, DIRECT);

void setup(){
    Serial.begin(9600);
    
    for(int i=0; i<5; i++)  pinMode(rcPin[i], INPUT);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);

    for(int i=0; i<5; i++)  wiiServo[i].attach(wiiPin[i]);
    wiiServo[4].writeMicroseconds(2000);            // Since this is permanently armed

    thrPID.SetOutputLimits(1050,1950);
    thrPID.SetSampleTime(samplingDelay[0]);
}

void loop(){
    now = millis();

    // Update all ultrasound readings
    if((now - timeStamp[1]) > samplingDelay[1]){
        timeStamp[1] = now;
        dist += constrain(smoooth(readSensor(trigPin, echoPin), filterValue, dist), -5, 5);
    }
    
    for(int i=0; i<5; i++){
        temp = pulseIn(rcPin[i], HIGH, 2000);
        if (temp!=0){
            rcReading[i] = temp;
            if (!isAuto || i>0) wiiServo[i].writeMicroseconds(temp);
        }
    }
    // Check if Aux has been flipped and change mode between manual and auto
    if(!isAuto && rcReading[4]>AUX_THRESHOLD[1]){
        isAuto = true;
        thrVal = rcReading[0];
        setDist = dist;
        // To eliminate jerk, we match input and output of PID to the current values before switching it on
        thrPID.SetMode(AUTOMATIC);
    }
    else if(isAuto && rcReading[4]<AUX_THRESHOLD[0]) {
        isAuto = false;
        thrPID.SetMode(MANUAL);
    }
    

    // Output commands to the MultiWii
    if(isAuto && (now - timeStamp[0]) > samplingDelay[0]){
        timeStamp[0] = now;
        thrPID.Compute();
        wiiServo[0].writeMicroseconds(thrVal);
    }

    // Print output on serial line
    if((now - timeStamp[2]) > samplingDelay[2]){
        Serial.print((int)dist);
        Serial.print("\t");
        Serial.print((int)setDist);
        Serial.print("\t");
        Serial.print((int)thrVal);
        Serial.print("\t");
        if(isAuto)  Serial.println("AUTO");
        else        Serial.println("MANUAL");
    timeStamp[2] = now;
    }
}
    
double readSensor(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);     delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH, 25000)/29/2*10/9;
}

double smooth(double data, double filterVal, double smoothedVal){
    // Higher filter values cause more filtering, slower response
    filterVal = constrain(filterVal, 0, 1.0);
    return (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
}


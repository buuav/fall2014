//#include <PID_v1.h>

const int echoPin = A5, trigPin = 8;
double dist = 0, setDist = 100, thrVal = 0;
const int sampleTime = 500;  // Sample time in ms

//PID thrPID(&dist, &thrVal, &setDist, 2, 5, 1, DIRECT);

void setup(){
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  readSensor(&dist);
//  thrPID.SetOutputLimits(1000, 2000);
//  thrPID.SetMode(AUTOMATIC);
//  thrPID.SetSampleTime(sampleTime);
  
}

void loop(){
  readSensor(&dist);
//  thrPID.Compute();
  Serial.print("dist:");Serial.print(dist);Serial.print("\t");
  Serial.print("setDist:");Serial.print(setDist);Serial.print("\t");
  Serial.print("thrVal:");Serial.println(thrVal); 
  delay(sampleTime);
}

long readSensor(double *dist){
//  int numReadings = 2;
//  long distSum = 0;
//  for(int i=0; i<numReadings; i++){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  *dist = pulseIn(echoPin, HIGH, 25000)/29/2*10/9;
//  *dist = smooth(pulseIn(echoPin, HIGH, 25000)/29/2, 0.6, *dist);
//  distSum += pulseIn(echoPin, HIGH, 25000) / 29 / 2;
//  }
//  *dist = constrain(distSum / numReadings, 0, 150);
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


//long msToCm(long microseconds)
//{
//  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
//  // The ping travels out and back, so to find the distance of the
//  // object we take half of the distance travelled.
//  return microseconds / 29 / 2;
//}

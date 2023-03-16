#include <B31DGMonitor.h>
#include <Ticker.h>

B31DGCyclicExecutiveMonitor monitor;
Ticker periodicTicker;

#define TASK1PIN 23
#define TASK2PIN 15
#define TASK3PIN 19
#define TASK4INPUTPIN 34
#define TASK4OUTPUTPIN 27

//variables for tasks
int counter, frameCounter;  //counter variables
float arr[4] = {0, 0, 0, 0};  //hold past 4 readings for task 4
int frequency2Int, frequency3Int; //hold frequencies

//constants
const int pulse1 = 200, pulse2 = 20, pulseDelay = 50; //task1 pulse constants
const float maxRange = 4095.00; //Hold max range of potentiometer
const int lowerThresh = 0, higherThresh = 99; //thresholds for remapping task2/task3 readings
const int lowerFreq2 = 333, higherFreq2 = 1000, lowerFreq3 = 500, higherFreq3 = 1000;

//test variables for calculating time
int time1, time2, timeDiff;


void setup() {
  Serial.begin(9600);
  monitor.startMonitoring();

  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, INPUT);
  pinMode(TASK3PIN, INPUT);
  pinMode(TASK4INPUTPIN, INPUT);
  pinMode(TASK4OUTPUTPIN, OUTPUT);

  counter = 0;
  frameCounter = 0;

  schedule();
  periodicTicker.attach_ms(4,schedule);
}

void schedule(){
  int frame = frameCounter % 50;
  switch(frame){
    case 0:case 10:case 20:case 30:case 40:
      task1();  task3();  task4();  break;
    case 5:case 15:case 25:case 35:case 45:
      task1();  task2();  task4();  break;
    case 1:case 11:case 21:case 31:case 41:
      task1();  task2();  break;
    case 6:case 32:
      task1();  task3();  task5();  break;
    default:
      if(frame%2 == 0){
        task1();  task3();
      } else {
        task1();
      }
      break;
  }
  frameCounter++;
}

void loop() {
}

void task1(){
  monitor.jobStarted(1);
  //ceate a pulse of 200us followed by a pulse of 20us with a 50us delay between pulses
  digitalWrite(TASK1PIN, HIGH);
  delayMicroseconds(pulse1);
  digitalWrite(TASK1PIN, LOW);
  delayMicroseconds(pulseDelay);
  digitalWrite(TASK1PIN, HIGH);
  delayMicroseconds(pulse2);
  digitalWrite(TASK1PIN, LOW);
  
  monitor.jobEnded(1);
}

float measureFrequency(uint8_t pin){
  float halfTime, startTime;
  if (digitalRead(pin) == HIGH){
    while(digitalRead(pin) == HIGH){
    }
    startTime = micros();
    while(digitalRead(pin) == LOW){
    }
    halfTime = micros();
  }
  else{
    while(digitalRead(pin) == LOW){
    }
    startTime = micros();
    while(digitalRead(pin) == HIGH){
    }
    halfTime = micros();
  }
  float frequency = (float(1)/((halfTime-startTime)*float(2)))*float(1000000);
  return frequency;
}

void task2(){
  monitor.jobStarted(2);
  float frequency2 = measureFrequency(TASK2PIN);
  frequency2Int = map(frequency2, lowerFreq2, higherFreq2, lowerThresh, higherThresh);
  frequency2Int = constrain(frequency2Int, lowerThresh, higherThresh);
  monitor.jobEnded(2);
}

void task3(){
  monitor.jobStarted(3);
  float frequency3 = measureFrequency(TASK3PIN);
  frequency3Int = map(frequency3, lowerFreq3, higherFreq3, lowerThresh, higherThresh);
  frequency3Int = constrain(frequency3Int, lowerThresh, higherThresh);
  monitor.jobEnded(3);
}

void task4(){
  monitor.jobStarted(4);
  float task4Value = analogRead(TASK4INPUTPIN);  //Read input
  //save input
  int arrPos = counter % 4;
  arr[arrPos] = task4Value;
  //average last 4 values
  float average = 0;
  for (int i = 0; i < 4; i++){
    average += arr[i];
  }
  average = average/4;
  if(average > (maxRange/2)){
    digitalWrite(TASK4OUTPUTPIN, HIGH);
  } else {
    digitalWrite(TASK4OUTPUTPIN, LOW);
  }
  
  //update counter
  counter++;
  monitor.jobEnded(4);
}

void task5(){
  monitor.jobStarted(5);
  //...
  //char string = ("%d,%d", frequency2Int, frequency3Int);
  Serial.print(frequency2Int);
  Serial.print(",");
  Serial.println(frequency3Int);
  //Serial.flush();
  monitor.jobEnded(5);
}

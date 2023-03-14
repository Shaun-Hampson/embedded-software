#include <B31DGMonitor.h>
#include <Ticker.h>

B31DGCyclicExecutiveMonitor monitor;
Ticker periodicTicker;

#define TASK1PIN 23
#define TASK2PIN 15
#define TASK3PIN 19
#define TASK4INPUTPIN 34
#define TASK4OUTPUTPIN 27

int counter;
float arr[4] = {0, 0, 0, 0};
float average;
float maxRange = 4095.00;
int time1, time2, timeDiff;
float startTime, halfTime, frequency2, frequency3;
int frequency2Int, frequency3Int;

void setup() {
  Serial.begin(9600);
  monitor.startMonitoring();

  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, INPUT);
  pinMode(TASK3PIN, INPUT);
  pinMode(TASK4INPUTPIN, INPUT);
  pinMode(TASK4OUTPUTPIN, OUTPUT);

  counter = 0;
}

void loop() {
  task2();
  task3();
  time1 = micros();
  task5();
  time2 = micros();
  timeDiff = time2-time1;
  Serial.println(timeDiff);
  Serial.println();
}

void task1(){
  monitor.jobStarted(1);
  //...
  digitalWrite(TASK1PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(TASK1PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(TASK1PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TASK1PIN, LOW);
  
  monitor.jobEnded(1);
}

void task2(){
  monitor.jobStarted(2);
  //...
  if (digitalRead(TASK2PIN) == HIGH){
    while(digitalRead(TASK2PIN) == HIGH){
      //Serial.print("while 1");
    }
    startTime = micros();
    while(digitalRead(TASK2PIN) == LOW){
      //Serial.print("while 12");
    }
    halfTime = micros();
  }
  else{
    while(digitalRead(TASK2PIN) == LOW){
      //Serial.print("while 2");
    }
    startTime = micros();
    while(digitalRead(TASK2PIN) == HIGH){
      //Serial.print("while 22");
    }
    halfTime = micros();
  }
  frequency2 = (float(1)/((halfTime-startTime)*float(2)))*1000000;
  //Serial.println(halfTime-startTime);
  //Serial.println(frequency2);
  frequency2Int = map(frequency2, 333, 1000, 0, 99);
  frequency2Int = constrain(frequency2Int, 0, 99);
  Serial.println(frequency2Int);
  monitor.jobEnded(2);
}

void task3(){
  monitor.jobStarted(3);
  //..
  if (digitalRead(TASK3PIN) == HIGH){
    while(digitalRead(TASK3PIN) == HIGH){
      //Serial.print("while 1");
    }
    startTime = micros();
    while(digitalRead(TASK3PIN) == LOW){
      //Serial.print("while 12");
    }
    halfTime = micros();
  }
  else{
    while(digitalRead(TASK3PIN) == LOW){
      //Serial.print("while 2");
    }
    startTime = micros();
    while(digitalRead(TASK3PIN) == HIGH){
      //Serial.print("while 22");
    }
    halfTime = micros();
  }
  frequency3 = (float(1)/((halfTime-startTime)*float(2)))*1000000;
  //Serial.println(halfTime-startTime);
  //Serial.println(frequency2);
  frequency3Int = map(frequency3, 500, 1000, 0, 99);
  frequency3Int = constrain(frequency3Int, 0, 99);
  Serial.println(frequency3Int);
  monitor.jobEnded(3);
}

void task4(){
  monitor.jobStarted(4);
  //...
  float task4Value = analogRead(TASK4INPUTPIN);  //Read input
  //save input
  int arrPos = counter % 4;
  arr[arrPos] = task4Value;
  //average last 4 values
  average = 0;
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
  Serial.println(frequency2Int +","+ frequency3Int);
  monitor.jobEnded(5);
}

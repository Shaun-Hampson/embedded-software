#include <B31DGMonitor.h>

B31DGCyclicExecutiveMonitor monitor;

#define TASK1PIN 23
#define TASK2PIN 15
#define TASK3PIN 19
#define TASK4INPUTPIN 34
#define TASK4OUTPUTPIN 27

float task2;
float task3;
int counter;
float arr[4] = {0, 0, 0, 0};
float average;
int maxRange = 0;
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
}

void loop() {
  time1 = micros();
  task2Proceedure();
  time2 = micros();
  timeDiff = time2-time1;
  Serial.println(timeDiff);
}

void task1Proceedure(){
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

void task2Proceedure(){
  monitor.jobStarted(2);
  //...
  task2 = pulseIn(TASK2PIN, HIGH);
  monitor.jobEnded(2);
}

void task3Proceedure(){
  monitor.jobStarted(3);
  //..
  task3 = digitalRead(TASK3PIN);
  monitor.jobEnded(3);
}

void task4Proceedure(){
  monitor.jobStarted(4);
  //...
  float task4 = analogRead(TASK4INPUTPIN);  //Read input

  //save input
  int arrPos = counter % 4;
  arr[arrPos] = task4;

  //average last 4 values
  average = 0;
  for (int i = 0; i < sizeof(arr); i++){
    average += arr[i];
  }
  average = average/sizeof(arr);

  if(average > maxRange/2){
    digitalWrite(TASK4OUTPUTPIN, HIGH);
  } else {
    digitalWrite(TASK4OUTPUTPIN, LOW);
  }
  
  //update counter
  counter++;
  monitor.jobEnded(4);
}

void task5Proceedure(){
  monitor.jobStarted(5);
  //...
  int task2Int = map(task2, 333, 1000, 0, 99);
  int task3Int = map(task3, 500, 1000, 0, 99);
  char string = ("Frequency of signal in task 2: %d, Frequency of signal in task 3 %d", task2Int, task3Int);
  Serial.println(string);
  monitor.jobEnded(5);
}

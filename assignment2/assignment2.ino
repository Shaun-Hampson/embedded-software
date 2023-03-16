#include <B31DGMonitor.h>
#include <Ticker.h>

B31DGCyclicExecutiveMonitor monitor;
Ticker periodicTicker;

#define TASK1PIN 23
#define TASK2PIN 15
#define TASK3PIN 19
#define TASK4INPUTPIN 34
#define TASK4OUTPUTPIN 27

int counter, frameCounter;
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
  //time1 = micros();
  //task3();
  //time2 = micros();
  //timeDiff = time2-time1;
  //Serial.println(timeDiff);
  //Serial.println();
}

void task1(){
  monitor.jobStarted(1);
  //...
  //ceate a pulse of 200us followed by a pulse of 20us with a 50us delay between pulses
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
  frequency2 = (float(1)/((halfTime-startTime)*float(2)))*float(1000000);
  //Serial.println(frequency2);
  frequency2Int = map(frequency2, 333, 1000, 0, 99);
  frequency2Int = constrain(frequency2Int, 0, 99);
  //Serial.println(frequency2Int);
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
  //Serial.println(frequency3Int);
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
  Serial.print(frequency2Int);
  Serial.print(",");
  Serial.println(frequency3Int);
  //Serial.flush();
  monitor.jobEnded(5);
}

#include <B31DGMonitor.h>
#include <Ticker.h>

B31DGCyclicExecutiveMonitor monitor;  //implement monitor
Ticker periodicTicker;  //implement ticker

//create defines
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

void setup() {
  Serial.begin(9600); //start serial monitor
  monitor.startMonitoring();  //start monitor

  //setup pins
  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, INPUT);
  pinMode(TASK3PIN, INPUT);
  pinMode(TASK4INPUTPIN, INPUT);
  pinMode(TASK4OUTPUTPIN, OUTPUT);

  //initiate counters
  counter = 0;
  frameCounter = 0;

  schedule(); //run first frame
  periodicTicker.attach_ms(4,schedule); //run schedule every 4ms
}

void schedule(){
  int frame = frameCounter % 50;  //calculate current frame
  switch(frame){
    case 0:case 10:case 20:case 30:case 40: //on frames 0, 10, 20, 30, 40
      task1();  task3();  task4();  break;    //run tasks 1, 3, 4
    case 5:case 15:case 25:case 35:case 45: //on frames 5, 15, 25, 35, 45
      task1();  task2();  task4();  break;    //run tasks 1, 2, 4
    case 1:case 11:case 21:case 31:case 41: //on frames 1, 11, 21, 31, 41
      task1();  task2();  break;              //run tasks 1 and 2
    case 6:case 32:                         //on frames 6 and 32
      task1();  task3();  task5();  break;    //run tasks 1, 3, 5
    default:                                //if not any stated frames
      if(frame%2 == 0){                       //if even frame
        task1();  task3();                      //run tasks 1 and 3
      } else {                                //else if odd frame
        task1();                                //run task 1
      }
      break;
  }
  frameCounter++; //increment frame counter
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
  //check if signal is HIGH
  //find the time for half the period
  if (digitalRead(pin) == HIGH){
    while(digitalRead(pin) == HIGH){  //while HIGH, keep polling
    }
    startTime = micros(); //once signal is LOW, record current time
    while(digitalRead(pin) == LOW){ //while LOW, keep polling
    }
    halfTime = micros();  //once signal is HIGH, record current time
  }
  //following lines occur if signal is LOW instead of HIGH. Work the same as above
  else{
    while(digitalRead(pin) == LOW){
    }
    startTime = micros();
    while(digitalRead(pin) == HIGH){
    }
    halfTime = micros();
  }
  float frequency = (float(1)/((halfTime-startTime)*float(2)))*float(1000000);  //find the frequency, 1/period 
  return frequency;
}

void task2(){
  monitor.jobStarted(2);
  float frequency2 = measureFrequency(TASK2PIN);  //read frequency
  frequency2Int = map(frequency2, lowerFreq2, higherFreq2, lowerThresh, higherThresh);  //remap frequency
  frequency2Int = constrain(frequency2Int, lowerThresh, higherThresh);  //constrain values
  monitor.jobEnded(2);
}

void task3(){
  monitor.jobStarted(3);
  float frequency3 = measureFrequency(TASK3PIN);  //read frequency
  frequency3Int = map(frequency3, lowerFreq3, higherFreq3, lowerThresh, higherThresh);  //remap frequency between lowerThresh and higherThsh values
  frequency3Int = constrain(frequency3Int, lowerThresh, higherThresh);  //constrain values between threshold values
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
  
  //check average is greater than half max range
  if(average > (maxRange/2)){
    digitalWrite(TASK4OUTPUTPIN, HIGH); //if greater turn LED on
  } else {
    digitalWrite(TASK4OUTPUTPIN, LOW);  //else turn LED off
  }
  
  //update counter
  counter++;
  monitor.jobEnded(4);
}

void task5(){
  monitor.jobStarted(5);
  Serial.print(frequency2Int);  //print task 2 frequency
  Serial.print(",");  //print comma
  Serial.println(frequency3Int);  //print task 3 frequency
  monitor.jobEnded(5);
}

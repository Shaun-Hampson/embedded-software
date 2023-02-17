#include <stdio.h>

#define SIGNALA 16
#define SIGNALB 21
#define MODEBUTTON 22
#define ENABLEBUTTON 23
#define MODE0 14
#define MODE1 17

  double a;  //width of pulses (uS)
  double b;  //width between pulses (uS)
  int c;   //number of pulses in block
  double d; //width between pulse blocks

void setup() {
  //variables for pulses
  a = 800;  //width of pulses (mS) 800uS = 0.8mS
  b = 100;  //width between pulses (mS) 100uS = 0.1ms
  c = MODE0;   //number of pulses in block
  d = 5500; //width between pulse blocks (ms) 5500uS = 5.5ms
  Serial.begin(115200);
  pinMode(SIGNALA,OUTPUT);
  pinMode(SIGNALB, OUTPUT);
  pinMode(MODEBUTTON, INPUT);
  pinMode(ENABLEBUTTON, INPUT);
}

void loop() {
  if(digitalRead(ENABLEBUTTON) == LOW){ //if enable utton is not pressed, run following block
    if(digitalRead(MODEBUTTON) == HIGH){  //if mode button is presed...
      c = MODE1;  //set c to 17
    } else {
      c = MODE0;  //if mode button is not pressed, set c to 14
    }
    
    //pulse signal b high for 50uS
    digitalWrite(SIGNALB, HIGH);
    delayMicroseconds(50);
    digitalWrite(SIGNALB, LOW);

    //pulse signal a c times
    for(int i=0; i<c; i++){
      digitalWrite(SIGNALA, HIGH);
      double delayTime = (a+(50*i));  //set the length of time the pulse is high
      delayMicroseconds(delayTime);
      digitalWrite(SIGNALA, LOW);
      delayMicroseconds(b); //set length of time signal a is low
    }
    delayMicroseconds(d); //set daley time between blocks of pulses
  }
}

#include <stdio.h>

#define signalA 16
#define signalB 21
#define modeButton 22
#define enableButton 23
#define mode0 14
#define mode1 17

  double a;  //width of pulses (uS)
  double b;  //width between pulses (uS)
  int c;   //number of pulses in block
  double d; //width between pulse blocks

void setup() {
  //variables for pulses
  a = 800;  //width of pulses (mS) 800uS = 0.8mS
  b = 100;  //width between pulses (mS) 100uS = 0.1ms
  c = mode0;   //number of pulses in block
  d = 5500; //width between pulse blocks (ms) 5500uS = 5.5ms
  Serial.begin(115200);
  pinMode(signalA,OUTPUT);
  pinMode(signalB, OUTPUT);
  pinMode(modeButton, INPUT);
  pinMode(enableButton, INPUT);
}

void loop() {
  if(digitalRead(enableButton) == LOW){
    if(digitalRead(modeButton) == HIGH){
      c = mode1;
    } else {
      c = mode0;
    }
    digitalWrite(signalB, HIGH);
    delayMicroseconds(50);
    digitalWrite(signalB, LOW);
    for(int i=0; i<c; i++){
      digitalWrite(signalA, HIGH);
      double delayTime = (a+(50*i));
      delayMicroseconds(delayTime);
      digitalWrite(signalA, LOW);
      delayMicroseconds(b);
    }
    delayMicroseconds(d);
  }
}

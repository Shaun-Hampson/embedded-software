#include <stdio.h>

#define signalA 15
#define signalB 21
#define modeButton 23
#define enableButton 22
#define mode0 14
#define mode1 17

void setup() {
  //variables for pulses
  int a = 800;  //width of pulses (uS)
  int b = 100;  //width between pulses (uS)
  int c = mode0;   //number of pulses in block
  int d = 5500; //width between pulse blocks

  pinMode(signalA,OUTPUT);
  pinMode(signalB, OUTPUT);
  pinMode(modeButton, INPUT);
  pinMode(enableButton, INPUT);
}

void loop() {

  int a = 800;  //width of pulses (uS)
  int b = 100;  //width between pulses (uS)
  int c = mode0;   //number of pulses in block
  int d = 5500; //width between pulse blocks
  
  if(enableButton == LOW){
    if(modeButton == HIGH){
      c = mode1;
    } else {
      c = mode0;
    }
    digitalWrite(signalB, HIGH);
    delay(0.05);
    digitalWrite(signalB, LOW);
    for(int i=0; i<c; i++){
      digitalWrite(signalA, HIGH);
      delay((a+(0.05*i)));
      digitalWrite(signalA, LOW);
      delay(b);
    }
    delay(d);
  }

}

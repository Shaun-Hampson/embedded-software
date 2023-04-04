#include <Arduino_FreeRTOS.h>

//create defines
#define TASK1PIN 23
#define TASK2PIN 16
#define TASK3PIN 19
#define TASK4INPUTPIN 34
#define TASK4OUTPUTPIN 27
#define BUTTONPIN 15
#define LEDPIN 13

void setup() {
  // put your setup code here, to run once:
  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, INPUT);
  pinMode(TASK3PIN, INPUT);
  pinMode(TASK4INPUTPIN, INPUT);
  pinMode(TASK4OUTPUTPIN, OUTPUT);
  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}

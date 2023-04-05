//assigns which core the program should be sent to
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//create defines
#define TASK1PIN 23
#define TASK2PIN 16
#define TASK3PIN 19
#define TASK4INPUTPIN 34
#define TASK4OUTPUTPIN 27
#define BUTTONPIN 15
#define LEDPIN 13

//constants
#define DEBOUNCE_DELAY 50 //

//create queues
static QueueHandle_t button_queue;

void setup() {
  // put your setup code here, to run once:
  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, INPUT);
  pinMode(TASK3PIN, INPUT);
  pinMode(TASK4INPUTPIN, INPUT);
  pinMode(TASK4OUTPUTPIN, OUTPUT);
  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);

  //start serial monitor
  Serial.begin(9600);

  //set up queue
  button_queue = xQueueCreate(40,sizeof(bool));
  
  xTaskCreate(button_Pressed,
    "Debounce Button",
    2048,
    NULL,
    1,
    NULL);

  xTaskCreate(LED_Toggle,
    "LED Toggle",
    2048,
    NULL,
    1,
    NULL);
}

void button_Pressed(void *argp){
  int lastState = LOW, buttonState, state;
  bool ledState = false;
  float timer;

  for(;;){
    state = digitalRead(BUTTONPIN);
    if (state != lastState){
      timer = millis();
    }
    if(millis()-timer > DEBOUNCE_DELAY){
      if(state != buttonState){
        buttonState = state;
        if(buttonState == HIGH){
          ledState = !ledState;
          if(xQueueSendToBack(button_queue,(void *)ledState, 1) == pdPASS){
            
          }
        }
      }
    }
    taskYIELD();
  }
}

void LED_Toggle(void *argp){
  BaseType_t queue;
  bool state = false, ledState;

  for(;;){
    queue = xQueueReceive(button_queue,(void *) ledState, portMAX_DELAY);
    assert(queue);
    if(ledState){
      state ^= true;
      digitalWrite(LEDPIN, state);
    }
  }
}

void loop() {}

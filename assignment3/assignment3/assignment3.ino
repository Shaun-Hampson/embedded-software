//assigns which core the program should be sent to
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//create defines
#define TASK1PIN 23//4
#define TASK2PIN 16
#define TASK3PIN 18
#define TASK4INPUTPIN 34//35
#define TASK4OUTPUTPIN 27//21
#define BUTTONPIN 15//27
#define LEDPIN 13

//constants
#define DEBOUNCE_DELAY 50

//delays
//FreeRTOS tick rate is 1ms
//all values in ms
#define D_T1 4
#define D_T2 20
#define D_T3 8
#define D_T4 20
#define D_T5 100
#define D_TBUTTON 10 
#define D_TLED 10

//create queues
static QueueHandle_t button_queue;

//create semaphores
static SemaphoreHandle_t frequency_semaphore;
static SemaphoreHandle_t timings_semaphore;


//create struct
struct Data{
  int frequency2; //hold value for task2 frequency
  int frequency3; //hold value for task3 frequency
} frequency;

void setup() {
  //start serial monitor
  Serial.begin(9600);
  
  //set up queue
  button_queue = xQueueCreate(3,sizeof(bool));
  
  //set up semaphores
  frequency_semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(frequency_semaphore);  //start semaphore in a "given" state so tasks 2,3,5 can run
  timings_semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(timings_semaphore);  //start semaphore in a "given" state so task 1 and measureFrequency can run
  
  // put your setup code here, to run once:
  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, INPUT);
  pinMode(TASK3PIN, INPUT);
  pinMode(TASK4INPUTPIN, INPUT);
  pinMode(TASK4OUTPUTPIN, OUTPUT);
  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);

  delay(1000);
  
  //create tasks
  /*
   * task
   * task name
   * stack size
   * parameters
   * priority
   * pxCreatedTask
   */
  xTaskCreate(task1,
    "Task 1",
    1024,
    NULL,
    3,
    NULL);

  xTaskCreate(task2,
    "Task 2",
    1024,
    NULL,
    2,
    NULL);

  xTaskCreate(task3,
    "Task 3",
    1024,
    NULL,
    2,
    NULL);

  xTaskCreate(task4,
    "Task 4",
    1024,
    NULL,
    1,
    NULL);

  xTaskCreate(task5,
    "Task 5",
    2048,
    NULL,
    2,
    NULL);
  
  xTaskCreate(button_Pressed,
    "Debounce Button",
    1024,
    NULL,
    1,
    NULL);

  xTaskCreate(LED_Toggle,
    "LED Toggle",
    1024,
    NULL,
    1,
    NULL);
}

//task1 - create pulses of 200ms and 20ms with a 50ms gap
void task1(void *pvParameter){
  const int pulse1 = 200, pulse2 = 20, pulseDelay = 50;
  
  for(;;){
    //Serial.println("task1");
    if(xSemaphoreTake(timings_semaphore, portMAX_DELAY) == pdPASS){
      digitalWrite(TASK1PIN, HIGH);
      delayMicroseconds(pulse1);
      digitalWrite(TASK1PIN, LOW);
      delayMicroseconds(pulseDelay);
      digitalWrite(TASK1PIN, HIGH);
      delayMicroseconds(pulse2);
      digitalWrite(TASK1PIN, LOW);
      xSemaphoreGive(timings_semaphore);
    }
    vTaskDelay(D_T1);
  }
}

//task2 - measure frequency between 333hz and 1000hz and rebind value from 0 to 99
void task2(void *pvParameter){
  const int lowerFrequency = 333, higherFrequency = 1000;
  const int lowerThresh = 0, higherThresh = 99;
  float frequency2;
  int frequency2Int;

  for(;;){
    frequency2 = measureFrequency(TASK2PIN);  //read frequency
    frequency2Int = map(frequency2, lowerFrequency, higherFrequency, lowerThresh, higherThresh);  //remap frequency
    frequency2Int = constrain(frequency2Int, lowerThresh, higherThresh);  //constrain values
    if(xSemaphoreTake(frequency_semaphore, portMAX_DELAY) == pdPASS){
      frequency.frequency2 = frequency2Int;
      xSemaphoreGive(frequency_semaphore);
    }
    vTaskDelay(D_T2);
  }
}

//task3 - measure frequency between 500hz and 1000hz and rebind value from 0 to 99
void task3(void *pvParameter){
  const int lowerFrequency = 500, higherFrequency = 1000;
  const int lowerThresh = 0, higherThresh = 99;
  float frequency3;
  int frequency3Int;

  for(;;){
    frequency3 = measureFrequency(TASK3PIN);
    frequency3Int = map(frequency3, lowerFrequency, higherFrequency, lowerThresh, higherThresh);  //remap frequency
    frequency3Int = constrain(frequency3Int, lowerThresh, higherThresh);  //constrain values
    if(xSemaphoreTake(frequency_semaphore, portMAX_DELAY) == pdPASS){
      frequency.frequency3 = frequency3Int;
      xSemaphoreGive(frequency_semaphore);
    }
    vTaskDelay(D_T3);
  }
}


//measureFrequency - read in frequency from desired pin
float measureFrequency(uint8_t pin){
  float halfTime, startTime;
  //check if signal is HIGH
  //find the time for half the period
  if(xSemaphoreTake(timings_semaphore, portMAX_DELAY) == pdPASS){
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
    xSemaphoreGive(timings_semaphore);
  }
  float frequency = (float(1)/((halfTime-startTime)*float(2)))*float(1000000);  //find the frequency, 1/period 
  return frequency;
}

//task4 - average the past for anaglog readings and turn on an led if over half of maxRange
void task4(void *pvParameter){
  const int maxRange = 4095.00;
  float task4Value, average;
  int counter = 0, arrPos;
  float arr[4] = {0,0,0,0};
  
  for(;;){
    //Serial.println("task4");
    task4Value = analogRead(TASK4INPUTPIN);

    arrPos = counter%4;
    arr[arrPos] = task4Value;

    average = 0;
    for(int i = 0; i < 4; i++){
      average += arr[i];
    }
    average = average/4;

    if(average > (maxRange/2)){
      digitalWrite(TASK4OUTPUTPIN, HIGH);
    } else {
      digitalWrite(TASK4OUTPUTPIN, LOW);
    }

    counter++;
    vTaskDelay(D_T4);
  }
}

//task5 - print the lastest readings from tasks 2 and 3
void task5(void *pvParameter){
  for(;;){
    if(xSemaphoreTake(frequency_semaphore, portMAX_DELAY) == pdPASS){
      Serial.print(frequency.frequency2);
      Serial.print(",");
      Serial.println(frequency.frequency3);
      xSemaphoreGive(frequency_semaphore);
    }
    vTaskDelay(D_T5);
  }
}

//task6 - button debounce, add state of led to a queue
void button_Pressed(void *pvParameters){
  int lastState = LOW, buttonState, state;
  bool ledState = false;
  float timer = 0;

  for(;;){
    state = digitalRead(BUTTONPIN);
    
    if (state != lastState){
      timer = millis();
    }
    
    if((millis()-timer) > DEBOUNCE_DELAY){
      if(state != buttonState){
        buttonState = state;
        if(buttonState == HIGH){
          ledState = !ledState;
          if(xQueueSend(button_queue,&ledState, 1) == pdPASS){
            
          }
        }
      }
    }
    vTaskDelay(D_TBUTTON);
    lastState = state;
  }
}

//task7 - LED toggle, read new state from queue and update LED
void LED_Toggle(void *pvParameters){
  BaseType_t queue;
  bool ledState;

  for(;;){
    queue = xQueueReceive(button_queue,&ledState, portMAX_DELAY);
    assert(queue);
    digitalWrite(LEDPIN, ledState);
    vTaskDelay(D_TLED);
  }
}

void loop() {}

/*
Third assignement embedded software

Name : EMONOT--DE CAROLIS Evrard
ID : H00385163

*/

//The FreeRTOS Library and semaphore is already on the ESP32

//we define our pin on the ESP32 board

#define INPUT_1 35 //pin for the input button
#define LED 14 //pin for the LED output
#define TASK1 25 //pin to send the SIGNAL B's first assignement
#define INTERRUPT_PIN 34 //pin to read the frequency of a square wave
#define POTENTIOMETER_PIN 32 //pin to read the potentiometer


//we declare our variable and constant (t* correspond in which task we use the variable)

unsigned long actualTime = micros();//we use micros to check the current time in microseconds
unsigned long passedTime = micros();

unsigned short input_potentiometer_t4 = 0; //value of the potentiometer

unsigned short all_potentiometer_t5[4] = {0,0,0,0}; //array that will received the last 4 values of our potentiometer

const short max_potentiometer_t7 = 4095; //max value we can get from our potentiometer
bool error_code_t7 = 0; //error_code depending on the value we read on the potentiometer


// Declare a mutex Semaphore Handle which we will use to access our structure in a safe way.
SemaphoreHandle_t xSerialSemaphore;// It will be used to ensure only only one Task is accessing to the structure at a time.
QueueHandle_t potentiometer_queue;// The queue will transmit the value of the potentiometer between task4 and 5


//we create the structure that contains the information that we will send in the Serial port
struct Task10 {
   bool input_Btn_t2; //state of the button we read
   float frequency_measured_t3; //frequency of the square signal
   unsigned short average_potentiometer_t5; //value that will contain the average of the last 4 reads of the potentiometer
}; 

struct Task10 struct_task10;
//we declare our functions, each task is a function + 2 other functions (one for the square interrupt and the other for executing a task at a certain frequence)


//===== INTERRUPT =====
//we check the period of the square wave, the interrupt is called for the rising edge of the square wave.
void interrupt(){
  
  passedTime = actualTime;
  actualTime = micros();
}

//===== TASK1 =====
//we send the signal B from the first assignement
//the format of the function correspond to the one to use RTOS
void task1(void *pvParameters){
  (void) pvParameters;
  for(;;){  
    digitalWrite(TASK1, HIGH);    
    vTaskDelay( 0.05 / portTICK_PERIOD_MS ); 
    digitalWrite(TASK1, LOW);
    vTaskDelay( 42.1 / portTICK_PERIOD_MS );//vTaskDelay is used to calculate how long the task will wait before being called again
    //so it correspond the frequency of the task
  } 
}

//===== TASK2 =====
//we read the input value of the button
void task2(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )//we use our semaphore to access the structure
    {
 
     struct_task10.input_Btn_t2 = digitalRead(INPUT_1); //read the button value

    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the access for others.
    }
    vTaskDelay( 200 ); //this task occurs every 200ms
  }
}

//===== TASK3 =====
//we calculate the frequency of the square signal using the values we get from the interrupt
void task3(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )//we use our semaphore to access the structure
    {
      
      struct_task10.frequency_measured_t3 = (1/((actualTime - passedTime)*0.000001));//calculate the frequenc and send it to the structure
      
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay( 1000 / portTICK_PERIOD_MS );//this task occurs every seconds
  }  
}

//===== TASK4 =====
//we read the value of the potentiometer
void task4(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    input_potentiometer_t4 = analogRead(POTENTIOMETER_PIN); //read the potentiometer
    xQueueSend(potentiometer_queue, &input_potentiometer_t4, portMAX_DELAY);//send this value through a queue
    vTaskDelay( 41 / portTICK_PERIOD_MS ); //the task occurs every 41ms
  }
}

//===== TASK5 =====
//we calculate the average of the last 4 reads of the potentiometer
void task5(void *pvParameters){
  (void) pvParameters;
  for(;;){
    //FIFO list to gather all last potentiometer values
    all_potentiometer_t5[3] = all_potentiometer_t5[2];
    all_potentiometer_t5[2] = all_potentiometer_t5[1];
    all_potentiometer_t5[1] = all_potentiometer_t5[0]; 
    xQueueReceive(potentiometer_queue, all_potentiometer_t5, portMAX_DELAY);//we received our value from the queue of task4 outside the semaphore
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )//we use our semaphore to access the structure
    {
      
      struct_task10.average_potentiometer_t5 = 0;
      unsigned short i=0;
      for (i=0;i<=3;i++) struct_task10.average_potentiometer_t5 += all_potentiometer_t5[i];
      struct_task10.average_potentiometer_t5 /= 4;
  
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay( 41 / portTICK_PERIOD_MS ); //the task occurs every 41ms
  }
}

//===== TASK6 =====
//execute 1000 times an empty instruction
void task6(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    unsigned short j=0;
    for(j=0;j<=999;j++) __asm__ __volatile__ ("nop");
    vTaskDelay( 100 / portTICK_PERIOD_MS );//the task occurs every 100ms
  }
}

//===== TASK7 =====
//if the average of the potentiometer is higher than half of the maximum of the potentiometer, we put an error code to one, otherwise it is 0
void task7(void *pvParameters){
  (void) pvParameters;
  for(;;){
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )//we use our semaphore to access the structure
    {

    //assign the error_code
    if (struct_task10.average_potentiometer_t5 > max_potentiometer_t7/2 ) error_code_t7 = 1;
    else error_code_t7 = 0;
    
    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    } 
    vTaskDelay( 333 / portTICK_PERIOD_MS ); //this task occurs every 333ms
  }
  
}

//===== TASK8 =====
//we light the LED knowing the error code
void task8(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if (error_code_t7 == 1) digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
    vTaskDelay( 333 / portTICK_PERIOD_MS ); //this task occurs every 333ms
  }
}

//===== TASK9 =====
//We display in a csv file format the state of the input button, the frequency of the square wave and the average of the last 4 read of the potentiometer
void task9(){
   
  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )//we use our semaphore to access the structure
  {
    //print every information we gather in our strcuture in the serial port
    Serial.print(struct_task10.input_Btn_t2);
    Serial.print(",");
    Serial.print(struct_task10.frequency_measured_t3);
    Serial.print(",");
    Serial.println(struct_task10.average_potentiometer_t5);

    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
  }

}

//===== TASK10 =====
//launch task 9 if the button is pressed
//the task is called every 5 seconds
void task10(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )//we use our semaphore to access the structure
    {
      if (struct_task10.input_Btn_t2 == 1) task9();
       
    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}

//in the setup function, we initialise our pin, and create our task in an RTOS method
void setup(){

  //intialise our serial port
  Serial.begin(115200);
  Serial.print("");
  potentiometer_queue = xQueueCreate(1,sizeof(short));//creating our queue of size one
  
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Structure
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make available for use, by "Giving" the Semaphore.
  }
  
  //initialisation of the serial comunication to print the values in CSV format
  Serial.println("input_state,frequency_value,filtered_analog_input");

  //we initialise our pin as output or input, but also the interruption
  pinMode(INPUT_1, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(TASK1, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);

  //Create all RTOS task, we set a priority where the shortest task are executed first
  //We fixed the memory being used to 4096 to avoid overflow
  xTaskCreate(task1,"task1",4096,NULL,3,NULL);
  xTaskCreate(task2,"task2",4096,NULL,4,NULL);  
  xTaskCreate(task3,"task3",4096,NULL,7,NULL);
  xTaskCreate(task4,"task4",4096,NULL,1,NULL);
  xTaskCreate(task5,"task5",4096,NULL,2,NULL);
  xTaskCreate(task6,"task6",4096,NULL,5,NULL);
  xTaskCreate(task7,"task7",4096,NULL,6,NULL);
  xTaskCreate(task8,"task8",4096,NULL,8,NULL);
  xTaskCreate(task10,"task10",4096,NULL,9,NULL);
  
}

void loop(){
  // the loop function stays empty as an we are using RTOS
}
//END

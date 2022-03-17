/*
Third assignement embedded software

Name : EMONOT--DE CAROLIS Evrard
ID : H00385163

*/

//we import the FreeRTOS Library and semaphore
//#include <Arduino_FreeRTOS.h>
//#include <semphr.h> 

//we define our pin on the ESP32 board

#define INPUT_1 35 //pin for the input button
#define LED 14 //pin for the LED output
#define TASK1 25 //pin to send the SIGNAL B's first assignement
#define INTERRUPT_PIN 34 //pin to read the frequency of a square wave
#define POTENTIOMETER_PIN 32 //pin to read the potentiometer

//we declare our variable and constant (t* correspond to the task where the variable has been used)


bool input_Btn_t2 = 0; //state of the button we read

float frequency_measured_t3;
unsigned long actualTime = micros();//we use micros to check the current time in microseconds
unsigned long passedTime = micros();

unsigned short input_potentiometer_t4 = 0; //value of the potentiometer

unsigned short all_potentiometer_t5[4] = {0,0,0,0}; //array that will received the last 4 values of our potentiometer
unsigned short average_potentiometer_t5 = 0; //value that will contains the average of the last 4 read of the potentiometer

const short max_potentiometer_t7 = 4095; //max value we can get from our potentiometer
bool error_code_t7 = 0; //error_code depending on the value we read on the potentiometer

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

int counter = 0;


//we declare our functions, each task is a function + 2 other functions (one for the square interrupt and the other for executing a task at a certain frequence)

//we check the period of the square wave, the interrupt is called for the rising edge of the square wave.
void interrupt(){
  
  passedTime = actualTime;
  actualTime = micros();
}

//we send the signal B from the first assignement
void task1(void *pvParameters){
  (void) pvParameters;
  for(;;){  
    
    digitalWrite(TASK1, HIGH);    
    vTaskDelay( 0.05 / portTICK_PERIOD_MS );
    digitalWrite(TASK1, LOW);
    vTaskDelay( 42.1 / portTICK_PERIOD_MS );
  } 
}

//we read the input value of the button
void task2(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    
    input_Btn_t2 = digitalRead(INPUT_1);
    vTaskDelay( 200 / portTICK_PERIOD_MS );
  }
}

//we calculate the frequency of the square signal using the values we get from the interrupt
void task3(void *pvParameters){

  (void) pvParameters;
  for(;;){ 
    frequency_measured_t3 = (1/((actualTime - passedTime)*0.000001));
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); 
    
  }  
}

//we read the value of the potentiometer
void task4(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    input_potentiometer_t4 = analogRead(POTENTIOMETER_PIN);
    vTaskDelay( 41 / portTICK_PERIOD_MS );
  }
}

//we calculate the average of the last 4 reads of the potentiometer
void task5(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    //FIFO list to gather all last potentiometer values
    all_potentiometer_t5[3] = all_potentiometer_t5[2];
    all_potentiometer_t5[2] = all_potentiometer_t5[1];
    all_potentiometer_t5[1] = all_potentiometer_t5[0];
    all_potentiometer_t5[0] = input_potentiometer_t4;
  
    average_potentiometer_t5 = 0;
    unsigned short i=0;
    for (i=0;i<=3;i++) average_potentiometer_t5 += all_potentiometer_t5[i];
    average_potentiometer_t5 /= 4;
    vTaskDelay( 41 / portTICK_PERIOD_MS );
  }
}

//execute 1000 times an empty instruction
void task6(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    unsigned short j=0;
    for(j=0;j<=999;j++) __asm__ __volatile__ ("nop");
    vTaskDelay( 100 / portTICK_PERIOD_MS );
  }
}

//if the average of the potentiometer is higher than half of the maximum of the potentiometer, we put an error code to one, otherwise it is 0
void task7(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if (average_potentiometer_t5 > max_potentiometer_t7/2 ) error_code_t7 = 1;
    else error_code_t7 = 0;
    vTaskDelay( 333 / portTICK_PERIOD_MS );
  }
}

//we light the LED knowing the error code
void task8(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if (error_code_t7 == 1) digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
    vTaskDelay( 333 / portTICK_PERIOD_MS );
  }
}

//we display in a csv file format the state of the input button, the frequency of the square wave and the average of the last 4 read of the potentiometer
/*
void task9(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      Serial.print(input_Btn_t2);
      Serial.print(",");
      Serial.print(frequency_measured_t3);
      Serial.print(",");
      Serial.println(average_potentiometer_t5);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
}

void task10(void *pvParameters){
  (void) pvParameters;
  for(;;){ 
    if (input_Btn_t2 == 1) task9();
    vTaskDelay( 5000 / portTICK_PERIOD_MS ); 
  }
}
*/
//we check the value of our counter and we execute each task at a certain frequency
//we use the modulo operation to execute the task at each new period (for exemple, if the number is 1000, we execute the task every second)
void do_all_task(){ 

  /*
  if (counter % 42 == 0) task1(); //period of signal B in first assignement is 42.15ms
  if (counter % 200 == 0) task2();  
  if (counter % 1000 == 0) task3();  
  if (counter % 41 == 0) task4();  
  if (counter % 41 == 0) task5();  
  if (counter % 100 == 0) task6();   
  if (counter % 333 == 0) task7();    
  if (counter % 333 == 0)task8();    
  if (counter % 5000 == 0) task9();
*/
}


//in the setup function, we initialise our pin, and add our ticker
void setup(){

  Serial.begin(115200);
  
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  
  //initialisation of the serial comunication to print the values in CSV format
  
  Serial.println("input_state,frequency_value,filtered_analog_input");

  //we initialise our pin as output or input, but also the interruption
  pinMode(INPUT_1, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(TASK1, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);
  
  //periodicTicker.attach_ms(1,do_all_task); //the ticker will called the do_all_task function every ms

  xTaskCreate(task1,"task1",4096,NULL,1,NULL);
  xTaskCreate(task2,"task2",4096,NULL,2,NULL);  
  xTaskCreate(task3,"task3",4096,NULL,3,NULL);
  xTaskCreate(task4,"task4",4096,NULL,4,NULL);
  xTaskCreate(task5,"task5",4096,NULL,5,NULL);
  xTaskCreate(task6,"task6",4096,NULL,6,NULL);
  xTaskCreate(task7,"task7",4096,NULL,7,NULL);
  xTaskCreate(task8,"task8",4096,NULL,8,NULL);
  //xTaskCreate(task9,(const portCHAR *)"task9",128,NULL,9,NULL);
  //xTaskCreate(task10,(const portCHAR *)"task10",128,NULL,10,NULL);
  
  
}

struct Books {
   char  title[50];
   char  author[50];
   char  subject[100];
   int   book_id;
} book; 

void loop(){
  // the loop function stays empty as an interrupt will called do_all_task every millisecond
}
//END

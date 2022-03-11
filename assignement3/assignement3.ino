/*
Third assignement embedded software

Name : EMONOT--DE CAROLIS Evrard
ID : H00385163

*/

#include <Ticker.h> //we import the Ticker librairie for using cycling executive method

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

Ticker periodicTicker; //our ticker object
int counter = 0;


//we declare our functions, each task is a function + 2 other functions (one for the square interrupt and the other for executing a task at a certain frequence)

//we check the period of the square wave, the interrupt is called for the rising edge of the square wave.
void interrupt(){
  
  passedTime = actualTime;
  actualTime = micros();
}

//we send the signal B from the first assignement
void task1(){
    
  digitalWrite(TASK1, HIGH);    
  delayMicroseconds(50);
  digitalWrite(TASK1, LOW); 
}

//we read the input value of the button
void task2(){
  
  input_Btn_t2 = digitalRead(INPUT_1);
}

//we calculate the frequency of the square signal using the values we get from the interrupt
void task3(){
  
  frequency_measured_t3 = (1/((actualTime - passedTime)*0.000001)); 
}

//we read the value of the potentiometer
void task4(){
  
  input_potentiometer_t4 = analogRead(POTENTIOMETER_PIN);
}

//we calculate the average of the last 4 reads of the potentiometer
void task5(){

  //FIFO list to gather all last potentiometer values
  all_potentiometer_t5[3] = all_potentiometer_t5[2];
  all_potentiometer_t5[2] = all_potentiometer_t5[1];
  all_potentiometer_t5[1] = all_potentiometer_t5[0];
  all_potentiometer_t5[0] = input_potentiometer_t4;

  average_potentiometer_t5 = 0;
  unsigned short i=0;
  for (i=0;i<=3;i++) average_potentiometer_t5 += all_potentiometer_t5[i];
  average_potentiometer_t5 /= 4;
}

//execute 1000 times an empty instruction
void task6(){
  
  unsigned short j=0;
  for(j=0;j<=999;j++) __asm__ __volatile__ ("nop");
}

//if the average of the potentiometer is higher than half of the maximum of the potentiometer, we put an error code to one, otherwise it is 0
void task7(){
  
  if (average_potentiometer_t5 > max_potentiometer_t7/2 ) error_code_t7 = 1;
  else error_code_t7 = 0;
}

//we light the LED knowing the error code
void task8(){
  
  if (error_code_t7 == 1) digitalWrite(LED, HIGH);
  else digitalWrite(LED, LOW);
}

//we display in a csv file format the state of the input button, the frequency of the square wave and the average of the last 4 read of the potentiometer
void task9(){
  
  Serial.print(input_Btn_t2);
  Serial.print(",");
  Serial.print(frequency_measured_t3);
  Serial.print(",");
  Serial.println(average_potentiometer_t5); 
}

void task10(){
  if (input_Btn_t2 == 1) task9();
 
}

//we check the value of our counter and we execute each task at a certain frequency
//we use the modulo operation to execute the task at each new period (for exemple, if the number is 1000, we execute the task every second)
void do_all_task(){ 
  
  if (counter % 42 == 0) task1(); //period of signal B in first assignement is 42.15ms
  if (counter % 200 == 0) task2();  
  if (counter % 1000 == 0) task3();  
  if (counter % 41 == 0) task4();  
  if (counter % 41 == 0) task5();  
  if (counter % 100 == 0) task6();   
  if (counter % 333 == 0) task7();    
  if (counter % 333 == 0)task8();    
  if (counter % 5000 == 0) task9();
  
  counter += 1; //we increment our counter of 1ms each time the ticker is called
  if (counter >= 500000) counter = 0; //to avoid an overflow of the counter we arbitrarily
}


//in the setup function, we initialise our pin, and add our ticker
void setup(){

  //initialisation of the serial comunication to print the values in CSV format
  Serial.begin(115200);
  Serial.println("input_state,frequency_value,filtered_analog_input");

  //we initialise our pin as output or input, but also the interruption
  pinMode(INPUT_1, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(TASK1, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);
  
  periodicTicker.attach_ms(1,do_all_task); //the ticker will called the do_all_task function every ms
}

void loop(){
  // the loop function stays empty as an interrupt will called do_all_task every millisecond
}
//END

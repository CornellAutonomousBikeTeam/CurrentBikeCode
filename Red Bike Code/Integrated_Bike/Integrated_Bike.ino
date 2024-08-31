
#include <FreeRTOS_SAMD21.h>
#include "IMU_Header.h"
#include "SPI.h" 
#include "ControlEquation.h"
#define SERIAL          SerialUSB

volatile float velocity;
volatile float roll_yaw_pitch[3]; 
volatile float motionEq;
IMU imu;
ControlEquation sc;

SemaphoreHandle_t imuSemaphore = NULL;
SemaphoreHandle_t wheelSpeedSemaphore = NULL;

void IMU_Thread( void *pvParameters ) 
{
  SERIAL.println("IMU Thread: Started"); 
  while(1){
    
    if (imuSemaphore == NULL){
      imuSemaphore = xSemaphoreCreateCounting(1,0);
    }
    float* temp = imu.IMUClassloop();
    for (int i = 0; i < 3; i++) {
        roll_yaw_pitch[i] = temp[i]; // Copy each element.
    }
    xSemaphoreGive(imuSemaphore);
    SERIAL.println("IMU Semaphore Given!");
    vTaskDelay(100);
  }
  
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  
  SERIAL.println("IMU Thread: Deleting");
  vTaskDelete( NULL );
}

void Stability_Thread( void *pvParameters ) 
{
  
  SERIAL.println("Stability: Started");
  while(1){
      if(xSemaphoreTake(imuSemaphore, 1000) == true || xSemaphoreTake(wheelSpeedSemaphore, 1000) == true){
        SERIAL.println("Semaphores Taken by Stability!");
        motionEq = sc.rollAngleAcceleration(roll_yaw_pitch[0], 0, velocity);
      }
      else{
        SERIAL.println("Semaphores NOT Taken by Stability!");
        delay(100);
        vTaskDelay(1000);
      }
  }
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Stability Thread: Deleting");
  vTaskDelete( NULL );
}

void Wheel_Speed_Thread( void *pvParameters ) 
{
  
  SERIAL.println("Wheel Speed Thread: Started");
  while(1){
    
    if (wheelSpeedSemaphore == NULL){
      wheelSpeedSemaphore = xSemaphoreCreateCounting(1,0);
    }
    velocity = 1;
    xSemaphoreGive(wheelSpeedSemaphore);
    SERIAL.println("Wheel Speed Semaphore Given!");
    vTaskDelay(100);
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Wheel Speed Thread: Deleting");
  vTaskDelete( NULL );
}


void setup()
{
  Serial.begin(9600);
  while(!Serial);
  
  IMU imu;
  imu.IMUClasssetup();


  xTaskCreate(
    
    IMU_Thread
    ,  "IMU" 
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

    xTaskCreate(
    Wheel_Speed_Thread
    ,  "Wheel Speed" 
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

      xTaskCreate(
    Stability_Thread
    ,  "Stability Calc" 
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

    vTaskStartScheduler();
}


void loop() // Constantly prints out the change in values of each axis.
{
 //Serial.println("finish");
}

#include <FreeRTOS_SAMD21.h>
#include "IMU_Header.h"
#include "SPI.h" 
#include "ControlEquation.h"

volatile float velocity;
volatile float roll_yaw_pitch[3]; 
volatile float motionEq;
IMU imu;
ControlEquation sc;

void IMU_Thread( void *pvParameters ) 
{
  
  Serial.println("IMU Thread: Started"); 

  while(1)
  {
    float* temp = imu.IMUClassloop();

    for (int i = 0; i < 3; i++) {
        roll_yaw_pitch[i] = temp[i]; // Copy each element.
    }
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  Serial.println("IMU Thread: Deleting");
  vTaskDelete( NULL );
}

void Stability_Thread( void *pvParameters ) 
{
  
  Serial.println("Stability: Started");

  while(1)
  {
    motionEq = sc.rollAngleAcceleration(roll_yaw_pitch[0], 0, velocity);
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  Serial.println("Stability Thread: Deleting");
  vTaskDelete( NULL );
}

void Wheel_Speed_Thread( void *pvParameters ) 
{
  
  Serial.println("Wheel Speed Thread: Started");

  while(1)
  {
    velocity = 1; //unsure how to do this since I have to decide between arduino wheel speed / wheel speed module
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  Serial.println("Wheel Speed Thread: Deleting");
  vTaskDelete( NULL );
}


void setup()
{
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
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

      xTaskCreate(
    Stability_Thread
    ,  "Stability Calc" 
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle
}

void loop() // Constantly prints out the change in values of each axis.
{
}
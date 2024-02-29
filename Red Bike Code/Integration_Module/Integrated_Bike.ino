#include "ControlEquation.h"
#include <FreeRTOS_SAMD21.h>
#include "IMU_Header.h"
#include "Arduino_StabilitiyCalc.ino"
#include "Arduino_Wheel_Speed.ino"
#include "Wheel_Speed_Module.ino"

volatile float velocity;
volatile float roll_yaw_pitch; 
volatile float motionEq;

void IMU_Thread( void *pvParameters ) 
{
  
  SERIAL.println("IMU Thread: Started");

  while(1)
  {
    roll_yaw_pitch = imu.IMUClassloop();
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("IMU Thread: Deleting");
  vTaskDelete( NULL );
}

void Stability_Thread( void *pvParameters ) 
{
  
  SERIAL.println("Stability: Started");

  while(1)
  {
    motionEq = sc.calculateMotionEqn(roll_yaw_pitch[0], roll_yaw_pitch[1], roll_yaw_pitch[2], velocity);
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Stability Thread: Deleting");
  vTaskDelete( NULL );
}

void Wheel_Speed_Thread( void *pvParameters ) 
{
  
  SERIAL.println("Wheel Speed Thread: Started");

  while(1)
  {
    velocity = checkFrontPWM(); //unsure how to do this since I have to decide between arduino wheel speed / wheel speed module
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Wheel Speed Thread: Deleting");
  vTaskDelete( NULL );
}


void setup()
{
  IMU imu;
  imu.IMUClasssetup();
  ControlEquation control_eq(0,0,0); 
  StabilityCalc sc;

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
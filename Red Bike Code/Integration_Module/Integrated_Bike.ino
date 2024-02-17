#include "ControlEquation.h"
#include <FreeRTOS_SAMD21.h>
#include "IMU_Header.h"


volatile float velocity;
volatile float roll_yaw_pitch; 

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

void Wheel_Speed_Thread( void *pvParameters ) 
{
  
  SERIAL.println("Wheel Speed Thread: Started");

  while(1)
  {
    velocity = 
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
}

void loop() // Constantly prints out the change in values of each axis.
{
}
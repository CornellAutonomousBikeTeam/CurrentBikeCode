#ifndef Bike_ROS_h
#define Bike_ROS_h

#include <ros.h>

typedef ros::NodeHandle_<ArduinoHardware, 1, 5, 500, 500> RosNodeHandle;

// navigation's command for what angle the front wheel should point in, updated in rosNavInstrListener (in BikeROS), used in navOrRC (in ros_arduino_wrapper.ino)
extern float nav_instr;

void initROS();

void rosPublishBikeState(float, float, float, float, float, float, float, float, float, float);
void rosPublishGps(float, float, float, float, float, float, float, float, float, float, float, float);
void rosPublishPid(const float *);
void rosPublishNmea(const char *);
void rosPublishLightSensor(const int32_t);
void rosSpinOnce();

#endif // Bike_ROS_h

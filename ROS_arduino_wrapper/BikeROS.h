#ifndef Bike_ROS_h
#define Bike_ROS_h

#include <ros.h>

typedef ros::NodeHandle_<ArduinoHardware, 1, 3, 500, 500> RosNodeHandle;
extern float nav_instr;

void initROS();

void rosPublishBikeState(float, float, float, float, float, float, float, float, float, float);
void rosPublishGps(float, float, float, float, float, float, float, float, float, float, float, float);
void rosPublishPid(const float *);
void rosPublishNmea(const char *);
void rosSpinOnce();

#endif // Bike_ROS_h

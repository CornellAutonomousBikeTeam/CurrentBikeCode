#include "BalanceController.h"

 
/* Function that returns desired angular velocity of front wheel */
float balanceController(float roll_angle, float roll_rate, float encoder_angle, float desired_steer) {
  float desiredSteerRate = (k1 * roll_angle) + (k2 * roll_rate) + k3 * (encoder_angle - desired_steer);
  if (desiredSteerRate > 10) {
    desiredSteerRate = 10;
  }
  else if (desiredSteerRate < -10) {
    desiredSteerRate = -10;
  }
  return desiredSteerRate;
}

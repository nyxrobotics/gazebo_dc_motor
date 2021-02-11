#ifndef _DC_MOTOR_DEFAULT_MODEL_H_
#define _DC_MOTOR_DEFAULT_MODEL_H_

#include <ros/ros.h>
#include <math.h>  /* M_PI */
#include "gazebo_dc_motor/low_pass_filter.h"

class DCMotorDefaultModel {
 public:
  DCMotorDefaultModel();
  ~DCMotorDefaultModel();
  void setMaxMotorSpeed(double max_motor_speed);
  void setMaxMotorTorque(double max_motor_torque);
  void setDt(double input_dt);
  double update(double input_torque,double input_position);
  
 private:
  //params
  double dt_;
  double max_motor_speed_; //[rad/s]
  double max_motor_torque_; //[N*m]
  double output_torque_;
  //internal value
  double previous_pose_;
  double current_pose_;
};

#endif

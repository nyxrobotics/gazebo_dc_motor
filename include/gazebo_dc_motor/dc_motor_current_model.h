#ifndef _DC_MOTOR_CURRENT_MODEL_H_
#define _DC_MOTOR_CURRENT_MODEL_H_

#include <ros/ros.h>
#include <math.h>  /* M_PI */
#include "gazebo_dc_motor/low_pass_filter.h"

class DCMotorCurrentModel {
 public:
  DCMotorCurrentModel();
  ~DCMotorCurrentModel();
  void setMaxMotorSpeed(double max_motor_speed);
  void setMaxMotorTorque(double max_motor_torque);

  void setDt(double input_dt);
  void setLowPassTimeConstant(double input_time_constant);
  double update(double input_torque,double input_position);
  
 private:
  //params
  double dt_;
  double max_motor_speed_; //[rad/s]
  double max_motor_torque_; //[N*m]
  double output_torque_;
  //params for low-pass filter
  LowPassFilter max_internal_torque_low_pass_filter_;
  //internal value
  double max_internal_torque_;
  double min_internal_torque_;
};

#endif

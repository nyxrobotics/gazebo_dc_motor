#ifndef _DC_MOTOR_DUTY_MODEL_H_
#define _DC_MOTOR_DUTY_MODEL_H_

#include <ros/ros.h>
#include <math.h>  /* M_PI */
#include "gazebo_dc_motor/low_pass_filter.h"

class DCMotorDutyModel {
 public:
  DCMotorDutyModel();
  ~DCMotorDutyModel();
  void setMaxMotorSpeed(double max_motor_speed);
  void setMaxMotorTorque(double max_motor_torque);

  void setDt(double input_dt);
  void setLowPassTimeConstant(double input_time_constant);
  double update(double input_duty,double input_position);
  
 private:
  //params
  double dt_;
  double max_motor_speed_; //[rad/s]
  double max_motor_torque_; //[N*m]
  double output_torque_;
  double rated_voltage_;
  //params for low-pass filter
  LowPassFilter motor_speed_low_pass_filter_;
  //internal value
  double internal_max_speed_;
  double internal_speed_;
};

#endif

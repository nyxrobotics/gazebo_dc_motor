#ifndef _DC_MOTOR_MODEL_SWITCHER_H_
#define _DC_MOTOR_MODEL_SWITCHER_H_

#include <ros/ros.h>
#include <math.h>  /* M_PI */
#include "gazebo_dc_motor/low_pass_filter.h"
// MotorModel
#include "gazebo_dc_motor/dc_motor_current_model.h"
#include "gazebo_dc_motor/dc_motor_duty_model.h"
#include "gazebo_dc_motor/dc_motor_default_model.h"

class DCMotorModelSwitcher {
 public:
  DCMotorModelSwitcher();
  ~DCMotorModelSwitcher();
  void setCurrentMode(void);
  void setDutyMode(void);
  void setDefaultMode(void);
  void setMaxMotorSpeed(double max_motor_speed);
  void setMaxMotorTorque(double max_motor_torque);
  void setDt(double input_dt);
  void setSpeedLowPassTimeConstant(double input_time_constant);
  void setTorqueLowPassTimeConstant(double input_time_constant);
  double update(double input_torque,double input_position);
  
 private:
  //params
  char mode_num_; //true:current, false:duty
  enum mode_enum_ : char {Duty, Current, Default};
  double dt_;
  DCMotorCurrentModel dc_motor_current_model_;
  DCMotorDutyModel dc_motor_duty_model_;
  DCMotorDefaultModel dc_motor_default_model_;
};

#endif

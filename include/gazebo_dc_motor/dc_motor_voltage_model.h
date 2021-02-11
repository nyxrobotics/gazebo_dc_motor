#ifndef _DC_MOTOR_VOLTAGE_MODEL_H_
#define _DC_MOTOR_VOLTAGE_MODEL_H_

#include <ros/ros.h>
#include <math.h>  /* M_PI */
#include "gazebo_dc_motor/low_pass_filter.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class DCMotorVoltageModel {
 public:
  DCMotorVoltageModel();
  ~DCMotorVoltageModel();
  void setMaxMotorSpeed(double max_motor_speed);
  void setMaxMotorTorque(double max_motor_torque);
  void setDt(double input_dt);
  double update(double input_torque,double input_position);
  void setBackEMFDamping(gazebo::physics::JointPtr joint_ptr);
  double compensateBackEMFTorque(double torque_raw);
  
 private:
  //params
  double dt_;
  double max_motor_speed_; //[rad/s]
  double max_motor_torque_; //[N*m]
  double output_torque_;
  //internal values
  double internal_speed_;
  double previous_pose_;
  double current_pose_;
};

#endif

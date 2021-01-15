#ifndef _DC_MOTOR_MODEL_H_
#define _DC_MOTOR_MODEL_H_

#include <assert.h>
#include <ros/ros.h>
#include <algorithm>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

class DCMotorModel {
 public:
  DCMotorModel();
  ~DCMotorModel();
  double motorModelUpdate(double input_duty, double dt, double actual_omega, double current_torque);

 private:


  // Measurement noise
  double velocity_noise_;
  double supply_voltage_;

  // Topic params
  std::string command_topic_;
  std::string encoder_topic_;
  std::string velocity_topic_;  /// topic for the motor shaft velocity (encoder
                                /// side, before gearbox)
  std::string current_topic_;
  std::string supply_topic_;
  std::string wrench_frame_;
  bool publish_velocity_;
  bool publish_current_;
  bool publish_encoder_;
  bool publish_motor_joint_state_;
  double input_;

  // Gearbox
  double gear_ratio_;  /// reduction ratio, eg 10.0 means 1/10-th output angular
                       /// velocity compared to motor inner vel.

  // Motor model
  double motor_nominal_voltage_;  /// the nominal voltage of the motor which
                                  /// corresponds to max angular velocity
  double moment_of_inertia_;
  double armature_damping_ratio_;
  double electromotive_force_constant_;  // Nm/A = V/(rad/s)
  double electric_resistance_;
  double electric_inductance_;
  // gazebo_ros_motors::motorModelConfig current_config_;
  // Internal state variables
  double internal_current_;
  double internal_omega_;

  // Helper variables
  // double update_period_;
  // common::Time last_update_time_;
};

#endif

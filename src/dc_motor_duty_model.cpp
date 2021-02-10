#include "gazebo_dc_motor/dc_motor_duty_model.h"


// Constructor
DCMotorDutyModel::DCMotorDutyModel():
  dt_(0.001),
  max_motor_speed_(3.1416),
  max_motor_torque_(10.000),
  output_torque_(0.0),
  internal_max_speed_(0.0),
  internal_speed_(0.0)
 {
  input_speed_low_pass_filter_.setDt(0.001);
  input_speed_low_pass_filter_.setTimeConstant(0.001);
  output_torque_low_pass_filter_.setDt(0.001);
  output_torque_low_pass_filter_.setTimeConstant(0.001);
}

// Destructor
DCMotorDutyModel::~DCMotorDutyModel() {}

void DCMotorDutyModel::setMaxMotorSpeed(double max_motor_speed) {
  max_motor_speed_ = max_motor_speed;
}
void DCMotorDutyModel::setMaxMotorTorque(double max_motor_torque) {
  max_motor_torque_ = max_motor_torque;
  input_speed_low_pass_filter_.setMinMax(-max_motor_torque_, max_motor_torque_);
}
void DCMotorDutyModel::setDt(double input_dt) {
  dt_ = input_dt;
  input_speed_low_pass_filter_.setDt(input_dt);
  output_torque_low_pass_filter_.setDt(input_dt);
}
void DCMotorDutyModel::setSpeedLowPassTimeConstant(double input_time_constant) {
  input_speed_low_pass_filter_.setTimeConstant(input_time_constant);
}
void DCMotorDutyModel::setTorqueLowPassTimeConstant(double input_time_constant) {
  output_torque_low_pass_filter_.setTimeConstant(input_time_constant);
}
double DCMotorDutyModel::update(double input_duty,double input_position){
  double input_limited = input_duty;
  if(input_limited > max_motor_torque_){
    input_limited = max_motor_torque_;
  }else if(input_limited < -max_motor_torque_){
    input_limited = -max_motor_torque_;
  }
  // Scale duty (torque:max_motor_torque_ -> duty:1.0)
  double duty = input_limited / max_motor_torque_;

  // Get motor speed
  static double previous_position = input_position;
  double position_diff = input_position - previous_position;
  previous_position = input_position;
  // Rotational motion through the origin　(only for continuous joint)
  if(position_diff > M_PI){
    position_diff -= (double)( (int)( position_diff / (2.0*M_PI) ) )*2.0*M_PI;
  }else if(position_diff < -M_PI){
    position_diff += (double)( (int)(-position_diff / (2.0*M_PI) ) )*2.0*M_PI;
  }
  double motor_speed = position_diff / dt_;
  // Calculate the characteristic curve of the DC motor.
  // 1. Obtain a graph of the relationship between angular velocity and torque using the input voltage. (input_voltage = duty * rated_voltage)
  // 2. Calculate the torque by substituting the current angular velocity.
  internal_speed_ = input_speed_low_pass_filter_.update(motor_speed);
  internal_max_speed_ = duty * max_motor_speed_;
  double output_torque_tmp = (internal_max_speed_ - internal_speed_) * max_motor_torque_ / max_motor_speed_;
  // ROS_INFO("output_torque_:%f , input_duty:%f , speed:%f", output_torque_ , input_duty , internal_speed_);
  // Escape over-speed (testing)
  if(motor_speed > max_motor_speed_){
    if(motor_speed > 2.0 * max_motor_speed_){
      motor_speed = 2.0 * max_motor_speed_;
    }
    double default_ratio = (motor_speed - max_motor_speed_) / max_motor_speed_;
    if(input_limited < 0.0){
      output_torque_tmp = (default_ratio * input_limited) + ((1.0-default_ratio)*output_torque_tmp);
    }else{
      output_torque_tmp = (1.0-default_ratio)*output_torque_tmp;
    }
  }else if(motor_speed < -max_motor_speed_){
    if(motor_speed < -2.0 * max_motor_speed_){
      motor_speed = -2.0 * max_motor_speed_;
    }
    double default_ratio = (-motor_speed - max_motor_speed_) / max_motor_speed_;
    if(input_limited > 0.0){
      output_torque_tmp = (default_ratio * input_limited) + ((1.0-default_ratio)*output_torque_tmp);
    }else{
      output_torque_tmp = (1.0-default_ratio)*output_torque_tmp;
    }
  }

  output_torque_ = output_torque_low_pass_filter_.update(output_torque_tmp);
  return output_torque_;
}


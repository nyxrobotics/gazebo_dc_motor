#include "gazebo_dc_motor/dc_motor_duty_model.h"


// Constructor
DCMotorDutyModel::DCMotorDutyModel():
  dt_(0.001),
  max_motor_speed_(3.1416),
  max_motor_torque_(10.000),
  output_torque_(0.0),
  internal_speed_(0.0),
  current_pose_(0.0),
  previous_pose_(0.0)
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
  input_speed_low_pass_filter_.setMinMax(-max_motor_speed, max_motor_speed);
}
void DCMotorDutyModel::setMaxMotorTorque(double max_motor_torque) {
  max_motor_torque_ = max_motor_torque;
  output_torque_low_pass_filter_.setMinMax(-2.0*max_motor_torque_, 2.0*max_motor_torque_);
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
  // // Scale duty (torque:max_motor_torque_ -> duty:1.0)
  double duty = input_limited / max_motor_torque_;

  // Get motor speed
  current_pose_ = input_position;
  double position_diff = current_pose_ - previous_pose_;
  double motor_speed = position_diff / dt_;
  previous_pose_ = current_pose_;
  // Calculate the characteristic curve of the DC motor.
  // 1. Obtain a graph of the relationship between angular velocity and torque using the input voltage. (input_voltage = duty * rated_voltage)
  // 2. Calculate the torque by substituting the current angular velocity.
  internal_speed_ = input_speed_low_pass_filter_.update(motor_speed);

  double internal_noload_speed = duty * max_motor_speed_;
  double internal_stall_torque = duty * max_motor_torque_;

  // PWM: ON-FREE mode (stable)
  double output_torque_on;
  if(duty > 0.0){
    output_torque_on = max_motor_torque_ - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  }else{
    output_torque_on = -max_motor_torque_ - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  }
  double output_torque_off = 0.0;
  double output_torque_tmp = std::abs(duty) * output_torque_on + (1.0 - std::abs(duty)) * output_torque_off;

  // PWM: ON-BREAK mode (unstable)
  // double output_torque_tmp = (internal_noload_speed - internal_speed_) * max_motor_torque_ / max_motor_speed_;
  // double output_torque_tmp = internal_stall_torque - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  // double output_torque_on;
  // if(duty > 0.0){
  //   output_torque_on = max_motor_torque_ - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  // }else{
  //   output_torque_on = -max_motor_torque_ - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  // }
  // double output_torque_off = - 1.0 * internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  // double output_torque_tmp = std::abs(duty) * output_torque_on + (1.0 - std::abs(duty)) * output_torque_off;
  // if(output_torque_on * output_torque_off > 0.0){
  //   output_torque_tmp = std::abs(duty) * output_torque_on;
  // }else if(output_torque_tmp * output_torque_on < 0.0) {
  //   output_torque_tmp = 0.0;
  // }

  // PWM: ON-BREAK-like mode
  // Limit back-EMF torque
  // double output_torque_tmp = (internal_noload_speed - internal_speed_) * max_motor_torque_ / max_motor_speed_;
  // double output_torque_tmp = internal_stall_torque - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  // if(std::abs(output_torque_tmp) > std::abs(2.0*internal_stall_torque)){
  //   output_torque_tmp = 2.0*internal_stall_torque;
  // }
  // if(output_torque_tmp * internal_stall_torque < 0.0){
  //   output_torque_tmp = 0.0;
  // }
  

  // ROS_INFO("output_torque_:%f , input_duty:%f ->%f, speed:%f", output_torque_tmp , duty,std::abs(duty), internal_speed_);

  output_torque_ = output_torque_low_pass_filter_.update(output_torque_tmp);
  return output_torque_;
}


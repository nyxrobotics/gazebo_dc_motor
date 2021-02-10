#include "gazebo_dc_motor/dc_motor_duty_model.h"


// Constructor
DCMotorDutyModel::DCMotorDutyModel():
  dt_(0.001),
  max_motor_speed_(3.1416),
  max_motor_torque_(10.000),
  output_torque_(0.0),
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
  static double previous_position = input_position;
  double position_diff = input_position - previous_position;
  previous_position = input_position;

  double motor_speed = position_diff / dt_;
  // Calculate the characteristic curve of the DC motor.
  // 1. Obtain a graph of the relationship between angular velocity and torque using the input voltage. (input_voltage = duty * rated_voltage)
  // 2. Calculate the torque by substituting the current angular velocity.
  internal_speed_ = input_speed_low_pass_filter_.update(motor_speed);

  double internal_noload_speed = duty * max_motor_speed_;
  double internal_stall_torque = duty * max_motor_torque_;
  // double output_torque_tmp = (internal_noload_speed - internal_speed_) * max_motor_torque_ / max_motor_speed_;
  double output_torque_tmp = internal_stall_torque - internal_speed_ * (max_motor_torque_ / max_motor_speed_ );
  // ROS_INFO("output_torque_:%f , input_duty:%f , speed:%f", output_torque_ , input_duty , internal_speed_);

  // Ignore over-speed breaking (reverse-side) torque
  if(internal_speed_ > internal_noload_speed && internal_noload_speed > 0.0){
    internal_speed_ = internal_noload_speed;
  }else if(internal_speed_ < internal_noload_speed && internal_noload_speed < 0.0){
    internal_speed_ = internal_noload_speed;
  }
  double max_internal_torque =  internal_stall_torque * ( internal_noload_speed - internal_speed_) / internal_noload_speed;
  double min_internal_torque = max_internal_torque - 2.0 * internal_stall_torque;
  if(output_torque_tmp > max_internal_torque){
    output_torque_tmp = max_internal_torque;
  }else if(output_torque_tmp < min_internal_torque){
    output_torque_tmp = min_internal_torque;
  }
  if(output_torque_tmp * input_duty < 0.000001){
    output_torque_tmp = 0.0;
  }
  if(abs(output_torque_tmp)>abs(internal_stall_torque)){
    output_torque_tmp = internal_stall_torque;
  }

  // output_torque_ = output_torque_low_pass_filter_.updateOnlyRising(output_torque_tmp);
  // output_torque_ = output_torque_tmp;
  output_torque_ = output_torque_low_pass_filter_.update(output_torque_tmp);
  return output_torque_;
}


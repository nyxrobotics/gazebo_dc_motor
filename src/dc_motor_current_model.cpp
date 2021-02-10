#include "gazebo_dc_motor/dc_motor_current_model.h"


// Constructor
DCMotorCurrentModel::DCMotorCurrentModel():
  dt_(0.001),
  max_motor_speed_(31.416),
  max_motor_torque_(100.0),
  output_torque_(0.0),
  min_internal_torque_(-1000.0),
  max_internal_torque_(1000.0)
 {
  input_speed_low_pass_filter_.setDt(0.001);
  input_speed_low_pass_filter_.setTimeConstant(0.001);
  output_torque_low_pass_filter_.setDt(0.001);
  output_torque_low_pass_filter_.setTimeConstant(0.001);
}

// Destructor
DCMotorCurrentModel::~DCMotorCurrentModel() {}

void DCMotorCurrentModel::setMaxMotorSpeed(double max_motor_speed) {
  max_motor_speed_ = max_motor_speed;
}
void DCMotorCurrentModel::setMaxMotorTorque(double max_motor_torque) {
  max_motor_torque_ = max_motor_torque;
  input_speed_low_pass_filter_.setMinMax(-max_motor_torque_, max_motor_torque_);
}
void DCMotorCurrentModel::setDt(double input_dt) {
  dt_ = input_dt;
  input_speed_low_pass_filter_.setDt(input_dt);
  output_torque_low_pass_filter_.setDt(input_dt);
}
void DCMotorCurrentModel::setSpeedLowPassTimeConstant(double input_time_constant) {
  input_speed_low_pass_filter_.setTimeConstant(input_time_constant);
}
void DCMotorCurrentModel::setTorqueLowPassTimeConstant(double input_time_constant) {
  output_torque_low_pass_filter_.setTimeConstant(input_time_constant);
}
double DCMotorCurrentModel::update(double input_torque,double input_position){
  // Get motor speed
  static double previous_position = input_position;
  double position_diff = input_position - previous_position;
  previous_position = input_position;
  double motor_speed = position_diff / dt_;
  if(motor_speed > 2.0 * max_motor_speed_){
    motor_speed = 2.0 * max_motor_speed_;
  }else if(motor_speed < -2.0 * max_motor_speed_){
    motor_speed = -2.0 * max_motor_speed_;
  }
  // Calculate the characteristic curve of the DC motor.
  // 1. Calculate maximum and minimum torque at current angular velocity. (with positive and negative rated voltage)
  // 2. Limit the output torque to the maximum torque that the motor can achieve.
  internal_speed_ = input_speed_low_pass_filter_.update(motor_speed);
  double output_torque_tmp = input_torque;
  max_internal_torque_ =  max_motor_torque_ * ( max_motor_speed_ - internal_speed_) / max_motor_speed_;
  min_internal_torque_ = max_internal_torque_ - 2.0 * max_motor_torque_;
  if(output_torque_tmp > max_internal_torque_){
    output_torque_tmp = max_internal_torque_;
    // ROS_INFO("output_tmp:%f(>max) max:%f , min:%f , speed:%f",output_torque_tmp,max_internal_torque_,min_internal_torque_,internal_speed_);
  }else if(output_torque_tmp < min_internal_torque_){
    output_torque_tmp = min_internal_torque_;
    // ROS_INFO("output_tmp:%f(<min) max:%f , min:%f , speed:%f",output_torque_tmp,max_internal_torque_,min_internal_torque_,internal_speed_);
  }
  // Escape over-speed (testing)
  double input_limited = input_torque;
  if(input_limited > 2.0 * max_motor_torque_){
    input_limited = 2.0 * max_motor_torque_;
  }else if(input_limited < -2.0 * max_motor_torque_){
    input_limited = -2.0 * max_motor_torque_;
  }
  if(internal_speed_ > max_motor_speed_){
    if(internal_speed_ > 2.0 * max_motor_speed_){
      internal_speed_ = 2.0 * max_motor_speed_;
    }
    double default_ratio = (internal_speed_ - max_motor_speed_) / max_motor_speed_;
    if(input_limited < 0.0){
      output_torque_tmp = (default_ratio * input_limited) + ((1.0-default_ratio)*output_torque_tmp);
    }else{
      output_torque_tmp = (1.0-default_ratio)*output_torque_tmp;
    }
  }else if(internal_speed_ < -max_motor_speed_){
    if(internal_speed_ < -2.0 * max_motor_speed_){
      internal_speed_ = -2.0 * max_motor_speed_;
    }
    double default_ratio = (-internal_speed_ - max_motor_speed_) / max_motor_speed_;
    if(input_limited > 0.0){
      output_torque_tmp = (default_ratio * input_limited) + ((1.0-default_ratio)*output_torque_tmp);
    }else{
      output_torque_tmp = (1.0-default_ratio)*output_torque_tmp;
    }
  }
  output_torque_ = output_torque_low_pass_filter_.update(output_torque_tmp);
  return output_torque_;
}


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
  motor_speed_low_pass_filter_.setDt(0.001);
  motor_speed_low_pass_filter_.setTimeConstant(0.001);
}

// Destructor
DCMotorDutyModel::~DCMotorDutyModel() {}

void DCMotorDutyModel::setMaxMotorSpeed(double max_motor_speed) {
  max_motor_speed_ = max_motor_speed;
}
void DCMotorDutyModel::setMaxMotorTorque(double max_motor_torque) {
  max_motor_torque_ = max_motor_torque;
  motor_speed_low_pass_filter_.setMinMax(-max_motor_torque_, max_motor_torque_);
}
void DCMotorDutyModel::setDt(double input_dt) {
  dt_ = input_dt;
  motor_speed_low_pass_filter_.setDt(input_dt);
}
void DCMotorDutyModel::setLowPassTimeConstant(double input_time_constant) {
  motor_speed_low_pass_filter_.setTimeConstant(input_time_constant);
}
double DCMotorDutyModel::update(double input_duty,double input_position){
  // scale duty_in (max_motor_torque_ -> 1.0)
  double duty = input_duty / max_motor_torque_;
  if(duty > 1.0){
    duty = 1.0;
  }else if(duty < -1.0){
    duty = -1.0;
  }
  // get motor speed
  static double previous_position = input_position;
  double position_diff = input_position - previous_position;
  previous_position = input_position;
  if(position_diff > M_PI){
    position_diff -= (double)( (int)( position_diff / (2.0*M_PI) ) )*2.0*M_PI;
  }else if(position_diff < -M_PI){
    position_diff += (double)( (int)(-position_diff / (2.0*M_PI) ) )*2.0*M_PI;
  }
  double motor_speed = position_diff / dt_;
  internal_speed_ = motor_speed_low_pass_filter_.update(motor_speed);
  // calcurate torque
  internal_max_speed_ = duty * max_motor_speed_;
  output_torque_ = (internal_max_speed_ - motor_speed) * max_motor_torque_ / max_motor_speed_;
  ROS_INFO("output_torque_:%f , input_duty:%f , speed:%f", output_torque_ , input_duty , motor_speed);
  return output_torque_;
}


#include "gazebo_dc_motor/dc_motor_model_switcher.h"


// Constructor
DCMotorModelSwitcher::DCMotorModelSwitcher():
  current_mode_flag_(true),
  dt_(0.001),
  max_motor_speed_(3.1416),
  max_motor_torque_(10.000)
 {
  dc_motor_current_model_.setDt(0.001);
  dc_motor_current_model_.setLowPassTimeConstant(0.001);
  dc_motor_duty_model_.setDt(0.001);
  dc_motor_duty_model_.setLowPassTimeConstant(0.002);
}

// Destructor
DCMotorModelSwitcher::~DCMotorModelSwitcher() {}

void DCMotorModelSwitcher::setCurrentMode(void) {
  current_mode_flag_ = true;
}
void DCMotorModelSwitcher::setDutyMode(void) {
  current_mode_flag_ = false;
}
void DCMotorModelSwitcher::setMaxMotorSpeed(double max_motor_speed) {
  if(current_mode_flag_){
    dc_motor_current_model_.setMaxMotorSpeed(max_motor_speed);
  }else{
    dc_motor_duty_model_.setMaxMotorSpeed(max_motor_speed);
  }
}
void DCMotorModelSwitcher::setMaxMotorTorque(double max_motor_torque) {
  if(current_mode_flag_){
    dc_motor_current_model_.setMaxMotorTorque(max_motor_torque);
  }else{
    dc_motor_duty_model_.setMaxMotorTorque(max_motor_torque);
  }
}
void DCMotorModelSwitcher::setDt(double input_dt) {
  if(current_mode_flag_){
    dc_motor_current_model_.setDt(input_dt);
  }else{
    dc_motor_duty_model_.setDt(input_dt);
  }
}
void DCMotorModelSwitcher::setLowPassTimeConstant(double input_time_constant) {
  if(current_mode_flag_){
    dc_motor_current_model_.setLowPassTimeConstant(input_time_constant);
  }else{
    dc_motor_duty_model_.setLowPassTimeConstant(input_time_constant);
  }
}
double DCMotorModelSwitcher::update(double input_torque,double input_position){
  double output_torque;
  if(current_mode_flag_){
    output_torque = dc_motor_current_model_.update(input_torque,input_position);
  }else{
    output_torque = dc_motor_duty_model_.update(input_torque,input_position);
  }
  return output_torque;
}


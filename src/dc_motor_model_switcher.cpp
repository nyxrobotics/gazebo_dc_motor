#include "gazebo_dc_motor/dc_motor_model_switcher.h"


// Constructor
DCMotorModelSwitcher::DCMotorModelSwitcher():
  mode_num_(true),
  dt_(0.001)
 {
  dc_motor_current_model_.setDt(0.001);
  dc_motor_current_model_.setSpeedLowPassTimeConstant(0.001);
  dc_motor_duty_model_.setDt(0.001);
  dc_motor_duty_model_.setSpeedLowPassTimeConstant(0.002);
  dc_motor_default_model_.setDt(0.001);
}

// Destructor
DCMotorModelSwitcher::~DCMotorModelSwitcher() {}

void DCMotorModelSwitcher::setCurrentMode(void) {
  mode_num_ = mode_enum_::Current;
}
void DCMotorModelSwitcher::setDutyMode(void) {
  mode_num_ = mode_enum_::Duty;
}
void DCMotorModelSwitcher::setDefaultMode(void) {
  mode_num_ = mode_enum_::Default;
}
void DCMotorModelSwitcher::setMaxMotorSpeed(double max_motor_speed) {
  if(mode_num_==mode_enum_::Current){
    dc_motor_current_model_.setMaxMotorSpeed(max_motor_speed);
  }else if(mode_num_==mode_enum_::Duty){
    dc_motor_duty_model_.setMaxMotorSpeed(max_motor_speed);
  }else if(mode_num_==mode_enum_::Default){
    dc_motor_default_model_.setMaxMotorSpeed(max_motor_speed);
  }
}
void DCMotorModelSwitcher::setMaxMotorTorque(double max_motor_torque) {
  if(mode_num_==mode_enum_::Current){
    dc_motor_current_model_.setMaxMotorTorque(max_motor_torque);
  }else if(mode_num_==mode_enum_::Duty){
    dc_motor_duty_model_.setMaxMotorTorque(max_motor_torque);
  }else if(mode_num_==mode_enum_::Default){
    dc_motor_default_model_.setMaxMotorTorque(max_motor_torque);
  }
}
void DCMotorModelSwitcher::setDt(double input_dt) {
  if(mode_num_==mode_enum_::Current){
    dc_motor_current_model_.setDt(input_dt);
  }else if(mode_num_==mode_enum_::Duty){
    dc_motor_duty_model_.setDt(input_dt);
  }else if(mode_num_==mode_enum_::Default){
    dc_motor_default_model_.setDt(input_dt);
  }
}
void DCMotorModelSwitcher::setSpeedLowPassTimeConstant(double input_time_constant) {
  if(mode_num_==mode_enum_::Current){
    dc_motor_current_model_.setSpeedLowPassTimeConstant(input_time_constant);
  }else if(mode_num_==mode_enum_::Duty){
    dc_motor_duty_model_.setSpeedLowPassTimeConstant(input_time_constant);
  }
}
void DCMotorModelSwitcher::setTorqueLowPassTimeConstant(double input_time_constant) {
  if(mode_num_==mode_enum_::Current){
    dc_motor_current_model_.setTorqueLowPassTimeConstant(input_time_constant);
  }else if(mode_num_==mode_enum_::Duty){
    dc_motor_duty_model_.setTorqueLowPassTimeConstant(input_time_constant);
  }
}
double DCMotorModelSwitcher::update(double input_torque,double input_position){
  double output_torque = input_torque;
  if(mode_num_==mode_enum_::Current){
    output_torque = dc_motor_current_model_.update(input_torque,input_position);
  }else if(mode_num_==mode_enum_::Duty){
    output_torque = dc_motor_duty_model_.update(input_torque,input_position);
  }else if(mode_num_==mode_enum_::Default){
    output_torque = dc_motor_default_model_.update(input_torque,input_position);
  }
  return output_torque;
}


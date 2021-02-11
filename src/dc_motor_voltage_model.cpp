#include "gazebo_dc_motor/dc_motor_voltage_model.h"

// Constructor
DCMotorVoltageModel::DCMotorVoltageModel():
  dt_(0.001),
  max_motor_speed_(31.416),
  max_motor_torque_(100.0),
  output_torque_(0.0),
  internal_speed_(0.0)
 {
}

// Destructor
DCMotorVoltageModel::~DCMotorVoltageModel() {}

void DCMotorVoltageModel::setMaxMotorSpeed(double max_motor_speed) {
  max_motor_speed_ = max_motor_speed;
}
void DCMotorVoltageModel::setMaxMotorTorque(double max_motor_torque) {
  max_motor_torque_ = max_motor_torque;
}
void DCMotorVoltageModel::setDt(double input_dt) {
  dt_ = input_dt;
}
double DCMotorVoltageModel::update(double input_torque,double input_position){
  // Get motor speed
  static double previous_position = input_position;
  double position_diff = input_position - previous_position;
  previous_position = input_position;
  double motor_speed = position_diff / dt_;
  internal_speed_ = motor_speed;

  // Defaul model just limits speed and torque
  output_torque_ = input_torque;
  if(motor_speed > max_motor_speed_ && output_torque_ > 0){
    output_torque_ = 0.0;
  }else if(motor_speed < -max_motor_speed_ && output_torque_ < 0){
    output_torque_ = 0.0;
  }else if(output_torque_ > max_motor_torque_){
    output_torque_ = max_motor_torque_;
  }else if(output_torque_ < -max_motor_torque_){
    output_torque_ = -max_motor_torque_;
  }
  return output_torque_;
}
// Voltage mode specific functions
// You need below lines in your xacro
/*
 *  <gazebo reference="${joint_name}">
 *    <implicitSpringDamper>1</implicitSpringDamper>
 *  </gazebo>
 */
void DCMotorVoltageModel::setBackEMFDamping(gazebo::physics::JointPtr joint_ptr) {
  double virtual_damping = joint_ptr->GetDamping(0) + (max_motor_torque_ / max_motor_speed_);
  ROS_WARN("Set joint damping -> %f",virtual_damping);
  joint_ptr->SetDamping(0, virtual_damping);
}

double DCMotorVoltageModel::compensateBackEMFTorque(double torque_raw){
  double output_torque;
  output_torque = torque_raw - max_motor_torque_ * internal_speed_ / max_motor_speed_;
  return output_torque;
}




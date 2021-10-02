#include "gazebo_dc_motor/dc_motor_voltage_model.h"

// Constructor
DCMotorVoltageModel::DCMotorVoltageModel():
  dt_(0.001),
  max_motor_speed_(31.416),
  max_motor_torque_(100.0),
  output_torque_(0.0),
  internal_speed_(0.0),
  current_pose_(0.0),
  previous_pose_(0.0)
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
  current_pose_ = input_position;
  double position_diff = current_pose_ - previous_pose_;
  double motor_speed = position_diff / dt_;
  previous_pose_ = current_pose_;
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
  gazebo::physics::LinkPtr parent_link_ptr = joint_ptr->GetParent();
  gazebo::physics::LinkPtr child_link_ptr = joint_ptr->GetChild();
  // joint_ptr->Init();
  // parent_link_ptr->Init();
  // child_link_ptr->Init();
  joint_ptr->SetDamping(0, virtual_damping);
  joint_ptr->Reset();
  parent_link_ptr->Reset();
  child_link_ptr->Reset();
}

double DCMotorVoltageModel::compensateBackEMFTorque(double torque_raw){
  double output_torque;
  output_torque = torque_raw - max_motor_torque_ * internal_speed_ / max_motor_speed_;
  return output_torque;
}




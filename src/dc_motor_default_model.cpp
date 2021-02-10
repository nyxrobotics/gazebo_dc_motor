#include "gazebo_dc_motor/dc_motor_default_model.h"


// Constructor
DCMotorDefaultModel::DCMotorDefaultModel():
  dt_(0.001),
  max_motor_speed_(31.416),
  max_motor_torque_(100.0),
  output_torque_(0.0)
 {
}

// Destructor
DCMotorDefaultModel::~DCMotorDefaultModel() {}

void DCMotorDefaultModel::setMaxMotorSpeed(double max_motor_speed) {
  max_motor_speed_ = max_motor_speed;
}
void DCMotorDefaultModel::setMaxMotorTorque(double max_motor_torque) {
  max_motor_torque_ = max_motor_torque;
}
void DCMotorDefaultModel::setDt(double input_dt) {
  dt_ = input_dt;
}
double DCMotorDefaultModel::update(double input_torque,double input_position){
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
  // Limit speed sensor (testing function)
  if(motor_speed > 2.0 * max_motor_speed_){
    motor_speed = 2.0 * max_motor_speed_;
  }else if(motor_speed < -2.0 * max_motor_speed_){
    motor_speed = -2.0 * max_motor_speed_;
  }
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


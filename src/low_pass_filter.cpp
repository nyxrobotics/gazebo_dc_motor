#include "gazebo_dc_motor/low_pass_filter.h"

// Constructor
LowPassFilter::LowPassFilter():
  dt_(0.001),
  time_constant_(0.1),
  low_pass_output_(0.0),
  output_min_(-1000.0),
  output_max_(1000.0)
 {

}

// Destructor
LowPassFilter::~LowPassFilter() {}

void LowPassFilter::setDt(double input_dt) {
  double input_check = input_dt;
  if(input_check < 0.00001){
    input_check = 0.00001;
  }
  dt_ = input_check;
}

void LowPassFilter::setTimeConstant(double input_time_constant) {
  double input_check = input_time_constant;
  if(input_check < 0.00001){
    input_check = 0.00001;
  }
  time_constant_ = input_check;
}

void LowPassFilter::setMinMax(double min, double max) {
  if(min < max){
    output_min_ = min;
    output_max_ = max;
  }
}

void LowPassFilter::setData(double input_data) {
  low_pass_output_ = input_data;
}
double LowPassFilter::getData(void) {
  return low_pass_output_;
}
double LowPassFilter::update(double input_data) {
  // Limit input
  double input_limited = input_data;
  if(input_limited > output_max_){
    input_limited = output_max_;
  }else if(input_limited < output_min_){
    input_limited = output_min_;
  }
  if( (dt_ / time_constant_) < 0.9999 ){
    low_pass_output_ += (input_limited - low_pass_output_) * dt_ / time_constant_;
  }else{
//    ROS_INFO_THROTTLE (1.0, "No Low Pass Filter");
    low_pass_output_ = input_data;
  }
  // Limit output
  if(low_pass_output_ > output_max_){
    low_pass_output_ = output_max_;
  }else if(low_pass_output_ < output_min_){
    low_pass_output_ = output_min_;
  }
  return low_pass_output_;
}
double LowPassFilter::updateOnlyRising(double input_data) {
  // Limit input
  double input_limited = input_data;
  if(input_limited > output_max_){
    input_limited = output_max_;
  }else if(input_limited < output_min_){
    input_limited = output_min_;
  }
  if( (dt_ / time_constant_) < 0.9999 ){
    low_pass_output_ += (input_limited - low_pass_output_) * dt_ / time_constant_;
  }else{
//    ROS_INFO_THROTTLE (1.0, "No Low Pass Filter");
    low_pass_output_ = input_data;
  }
  // Limit output
  if(low_pass_output_ > output_max_){
    low_pass_output_ = output_max_;
  }else if(low_pass_output_ < output_min_){
    low_pass_output_ = output_min_;
  }
  // Apply LPF only Rising
  if(low_pass_output_ * input_limited < 0.0){
    low_pass_output_ = 0.0;
  }else if(abs(low_pass_output_) > abs(input_limited)){
    low_pass_output_ = input_limited;
  }
  return low_pass_output_;
}
double LowPassFilter::updateOnlyFalling(double input_data) {
  // Limit input
  double input_limited = input_data;
  if(input_limited > output_max_){
    input_limited = output_max_;
  }else if(input_limited < output_min_){
    input_limited = output_min_;
  }
  if( (dt_ / time_constant_) < 0.9999 ){
    low_pass_output_ += (input_limited - low_pass_output_) * dt_ / time_constant_;
  }else{
//    ROS_INFO_THROTTLE (1.0, "No Low Pass Filter");
    low_pass_output_ = input_data;
  }
  // Limit output
  if(low_pass_output_ > output_max_){
    low_pass_output_ = output_max_;
  }else if(low_pass_output_ < output_min_){
    low_pass_output_ = output_min_;
  }
  // Apply LPF only Falling
  if(abs(low_pass_output_) < abs(input_limited)){
    low_pass_output_ = input_limited;
  }
  return low_pass_output_;
}
double LowPassFilter::updatePassthrough(double input_data) {
  // Limit input
  double input_limited = input_data;
  if(input_limited > output_max_){
    input_limited = output_max_;
  }else if(input_limited < output_min_){
    input_limited = output_min_;
  }
  low_pass_output_ = input_limited;
  return low_pass_output_;
}

#ifndef _LOW_PASS_FILTER_H_
#define _LOW_PASS_FILTER_H_

#include <ros/ros.h>

class LowPassFilter {
 public:
  LowPassFilter();
  ~LowPassFilter();
  void setDt(double input_dt);
  void setTimeConstant(double input_time_constant);
  void setMinMax(double min, double max);
  void setData(double input_data);
  double getData(void);
  double update(double input_data);

 private:
  double dt_;
  double time_constant_;
  double low_pass_output_;
  double output_max_;
  double output_min_;
};

#endif



#include "gazebo_dc_motor/dc_motor_model.h"


// Constructor
DCMotorModel::DCMotorModel() {
  //Params
  motor_nominal_voltage_ = 24.0;
  moment_of_inertia_ = 0.001;
  armature_damping_ratio_ = 0.0001;
  electromotive_force_constant_ = 0.08;
  electric_resistance_ = 1.0;
  electric_inductance_ = 0.001;
  velocity_noise_ = 0.0;
  gear_ratio_ = 1.0;
  //Internal value
  internal_current_ = 0.0;
  internal_omega_ = 0.0;

}

// Destructor
DCMotorModel::~DCMotorModel() {}


// Motor Model update function
double DCMotorModel::motorModelUpdate(double input_duty, double dt, double output_shaft_omega,
                                      double actual_load_torque) {

  if (input_duty > 1.0) {
    input_duty = 1.0;
  } else if (input_duty < -1.0) {
    input_duty = -1.0;
  }
  double T = actual_load_torque /
             gear_ratio_;  // external loading torque converted to internal side
  double V = input_duty * supply_voltage_;  // power supply voltage * (command input
                                        // for motor velocity)
  internal_omega_ =
      output_shaft_omega *
      gear_ratio_;  // external shaft angular veloc. converted to internal side
  // DC motor exact solution for current and angular velocity (omega)
  const double& d = armature_damping_ratio_;
  const double& L = electric_inductance_;
  const double& R = electric_resistance_;
  const double& Km = electromotive_force_constant_;
  const double& J = moment_of_inertia_;
  double i0 = internal_current_;
  double o0 = internal_omega_;
  double d2 = pow(d, 2);
  double L2 = pow(L, 2);
  double J2 = pow(J, 2);
  double R2 = pow(R, 2);
  double Km2 = pow(Km, 2);
  double Km3 = Km2 * Km;
  double Om = sqrt(d2 * L2 + J2 * R2 - 2 * J * L * (2 * Km2 + d * R));
  double eOp1 = exp((Om * dt) / (J * L)) + 1.0;
  double eOm1 = eOp1 - 2.0;  // = exp((Om*t)/(J*L)) - 1.0;
  double eA = exp(((d * L + Om + J * R) * dt) / (2.0 * J * L));
  double emA = 1.0 / eA;  // = exp(-((d*L + Om + J*R)*t)/(2.0*J*L));
  double i_t =
      (emA *
       (i0 * (Km2 + d * R) *
            (d * L * (d * eOp1 * L + eOm1 * Om) + eOp1 * J2 * R2 -
             J * (4 * eOp1 * Km2 * L + 2 * d * eOp1 * L * R + eOm1 * Om * R)) -
        d * L * (d * (-2 * eA + eOp1) * L + eOm1 * Om) * (Km * T + d * V) -
        (-2 * eA + eOp1) * J2 * R2 * (Km * T + d * V) +
        J * (Km3 * (-2 * eOm1 * o0 * Om + 4 * (-2 * eA + eOp1) * L * T) -
             Km * R * (2 * d * eOm1 * o0 * Om -
                       2 * d * (-2 * eA + eOp1) * L * T + eOm1 * Om * T) +
             2 * Km2 * (2 * d * (-2 * eA + eOp1) * L + eOm1 * Om) * V +
             d * (2 * d * (-2 * eA + eOp1) * L + eOm1 * Om) * R * V))) /
      (2. * (Km2 + d * R) *
       (d2 * L2 + J2 * R2 - 2 * J * L * (2 * Km2 + d * R)));
  double o_t =
      (emA *
       (-4 * eOp1 * J * pow(Km, 4) * L * o0 +
        J * Km2 * R * (-6 * d * eOp1 * L * o0 + eOm1 * o0 * Om -
                       4 * (-2 * eA + eOp1) * L * T) +
        J * R2 * (-2 * d2 * eOp1 * L * o0 + d * eOm1 * o0 * Om -
                  2 * d * (-2 * eA + eOp1) * L * T + eOm1 * Om * T) +
        4 * (-2 * eA + eOp1) * J * Km3 * L * V -
        J * Km * (-2 * d * (-2 * eA + eOp1) * L + eOm1 * Om) * R * V +
        J2 * R2 * (eOp1 * Km2 * o0 + d * eOp1 * o0 * R +
                   (-2 * eA + eOp1) * R * T - (-2 * eA + eOp1) * Km * V) +
        L * (pow(d, 3) * eOp1 * L * o0 * R +
             2 * eOm1 * Km2 * Om * (i0 * Km - T) +
             d2 * (eOp1 * Km2 * L * o0 - eOm1 * o0 * Om * R +
                   (-2 * eA + eOp1) * L * R * T -
                   (-2 * eA + eOp1) * Km * L * V) -
             d * eOm1 * Om * (Km2 * o0 + R * T + Km * (-2 * i0 * R + V))))) /
      (2. * (Km2 + d * R) *
       (d2 * L2 + J2 * R2 - 2 * J * L * (2 * Km2 + d * R)));
  // Update internal variables
  internal_current_ = i_t;
  internal_omega_ = o_t;
  ignition::math::Vector3d applied_torque;
  // TODO: axis as param
  applied_torque.Z() =
      Km * i_t * gear_ratio_;  // motor torque T_ext = K * i * n_gear
  double output_torque = Km * i_t * gear_ratio_;  // motor torque T_ext = K * i * n_gear
  return output_torque;
  // this->link_->AddRelativeTorque(applied_torque);
  // ROS_INFO_THROTTLE_NAMED(0.5, plugin_name_, "dt = %f ", dt);
}

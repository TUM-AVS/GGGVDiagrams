// Copyright 2025 Simon Sagmeister

#include "vehicle_model_gggv_sim_validation_cpp/validation_model.hpp"
namespace tam::sim
{
VehicleModelValidation::VehicleModelValidation()
{
  param_manager = tam::core::ParamManager();
  driver_input = tam::types::DriverInput();
  driver_input.steering_angle_rad = 0;
  driver_input.wheel_torque_Nm = {0, 0, 0, 0};
  // Declare parameters
  declare_parameters();
};
void VehicleModelValidation::set_driver_input(const tam::types::DriverInput & input)
{
  driver_input = input;
};
void VehicleModelValidation::set_external_influences(const tam::types::ExternalInfluences & input)
{
  external_influences = input;
};
tam::types::VehicleModelOutput VehicleModelValidation::get_output() const { return model_output; }
tam::types::common::TUMDebugContainer::SharedPtr VehicleModelValidation::get_debug_out() const
{
  return debug_container_;
}
tam::interfaces::ParamManagerBase * VehicleModelValidation::get_param_manager()
{
  return &param_manager;
};
Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1>
VehicleModelValidation::ode(
  double t, Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1> x_)
{
  Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1> x_dot;
  // Silence compiler warning bc t is unused
  (void)t;
  // Get parameter values
  double l_r = param_manager.get_parameter_value("vehicle_parameters.dist_cg_rear_m").as_double();
  double l_f = param_manager.get_parameter_value("vehicle_parameters.dist_cg_front_m").as_double();
  double r_tire =
    0.5 * param_manager.get_parameter_value("vehicle_dynamics_double_track.rr_w_r").as_double() +
    0.5 * param_manager.get_parameter_value("vehicle_dynamics_double_track.rr_w_f").as_double();
  double m_vehicle =
    param_manager.get_parameter_value("vehicle_dynamics_double_track.m").as_double();  // kg

  // Hardcoded values - fine for now since this is just for validation
  double a_max = 20;
  double F_drag = 2 * m_vehicle;

  // region Specification of the ODE
  x_dot[StateVectorValidationModel::x_m] =
    x_[StateVectorValidationModel::v_mps] * cos(x_[StateVectorValidationModel::psi_rad]);
  x_dot[StateVectorValidationModel::y_m] =
    x_[StateVectorValidationModel::v_mps] * sin(x_[StateVectorValidationModel::psi_rad]);

  // Calculate the new Fx request
  imr.Fx_request =
    (driver_input.wheel_torque_Nm.front_left + driver_input.wheel_torque_Nm.front_right +
     driver_input.wheel_torque_Nm.rear_left + driver_input.wheel_torque_Nm.rear_right) /
    r_tire;

  // Calculate Fy_request
  double curvature_driven_1pm = (tan(driver_input.steering_angle_rad)) / (l_f + l_r);
  imr.Fy_request = m_vehicle * curvature_driven_1pm * pow(x[StateVectorValidationModel::v_mps], 2);

  // Calculate F request
  imr.F_request = sqrt(pow(imr.Fx_request, 2) + pow(imr.Fy_request, 2));

  // Use a pacejka tire model to limit the F_request
  imr.F_actual = std::clamp(imr.F_request, 0.0, a_max * m_vehicle);

  double F_max = a_max * m_vehicle;
  imr.Fx = std::clamp(imr.Fx_request, -F_max, F_max);
  double Fy_max = std::sqrt(std::clamp(std::pow(F_max, 2) - std::pow(imr.Fx, 2), 0.0, __DBL_MAX__));
  imr.Fy = std::clamp(imr.Fy_request, -Fy_max, Fy_max);

  imr.ay = imr.Fy / m_vehicle;
  if (x_[StateVectorValidationModel::v_mps] < 1) {
    imr.yaw_rate =
      x_[StateVectorValidationModel::v_mps] * tan(driver_input.steering_angle_rad) / (l_r + l_f);
  } else {
    imr.yaw_rate = imr.ay / x_[StateVectorValidationModel::v_mps];
  }

  x_dot[StateVectorValidationModel::psi_rad] = imr.yaw_rate;
  x_dot[StateVectorValidationModel::v_mps] =
    (imr.Fx - F_drag + external_influences.external_force_N.x) / m_vehicle;
  return x_dot;
};
void VehicleModelValidation::step()
{
  // Init local variabls
  double h = param_manager.get_parameter_value("integration_step_size_s").as_double();

  // Write the current state vector to the debug out
  debug_container_->log("x_vec_current/x_mps", x[StateVectorValidationModel::x_m]);
  debug_container_->log("x_vec_current/y_mps", x[StateVectorValidationModel::y_m]);
  debug_container_->log("x_vec_current/psi_radps", x[StateVectorValidationModel::psi_rad]);
  debug_container_->log("x_vec_current/v_mps2", x[StateVectorValidationModel::v_mps]);

  // Eval x_dot once to get the current x_dot
  auto x_dot = ode(0, x);
  debug_container_->log("x_dot_vec/x_dot_mps", x_dot[StateVectorValidationModel::x_m]);
  debug_container_->log("x_dot_vec/y_dot_mps", x_dot[StateVectorValidationModel::y_m]);
  debug_container_->log("x_dot_vec/psi_dot_radps", x_dot[StateVectorValidationModel::psi_rad]);
  debug_container_->log("x_dot_vec/v_dot_mps2", x_dot[StateVectorValidationModel::v_mps]);

  // Log the imr's
  debug_container_->log("imr/Fx_request", imr.Fx_request);
  debug_container_->log("imr/Fy_request", imr.Fy_request);
  debug_container_->log("imr/F_request", imr.F_request);
  debug_container_->log("imr/F_actual", imr.F_actual);
  debug_container_->log("imr/Fx", imr.Fx);
  debug_container_->log("imr/Fy", imr.Fy);
  debug_container_->log("imr/ay", imr.ay);
  debug_container_->log("imr/yaw_rate", imr.yaw_rate);

  // Bind the function for the integrator
  // std::function<double, Eigen::Matrix<double,
  // StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1>> bound_ode =
  // std::bind(&VehicleModelValidation::ode, this, std::placeholders::_1, std::placeholders::_2);
  std::function<Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1>(
    double, Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1>)>
    bound_ode =
      std::bind(&VehicleModelValidation::ode, this, std::placeholders::_1, std::placeholders::_2);

  // Calculate the new x
  x = tam::helpers::numerical::integration_step_DoPri45<
    Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1>>(
    bound_ode, 0, x, h);

  // Set the outputs
  // NOTE this is the vehicle coordinate system around the center of gravity
  model_output.position_m.x = x[StateVectorValidationModel::x_m];
  model_output.position_m.y = x[StateVectorValidationModel::y_m];
  model_output.orientation_rad.z = x[StateVectorValidationModel::psi_rad];
  model_output.velocity_mps.x = x[StateVectorValidationModel::v_mps];
  model_output.velocity_mps.y = 0;
  model_output.angular_velocity_radps.z = x_dot[StateVectorValidationModel::psi_rad];
  model_output.acceleration_mps2.x = x_dot[StateVectorValidationModel::v_mps];
  model_output.acceleration_mps2.y = imr.ay;
  model_output.steering_angle = driver_input.steering_angle_rad;
  model_output.wheel_speed_radps.front_left = 0;
  model_output.wheel_speed_radps.front_right = 0;
  model_output.wheel_speed_radps.rear_left = 0;
  model_output.wheel_speed_radps.rear_right = 0;
};
void VehicleModelValidation::reset()
{
  x[StateVectorValidationModel::x_m] =
    param_manager.get_parameter_value("initial_state.x_m").as_double();
  x[StateVectorValidationModel::y_m] =
    param_manager.get_parameter_value("initial_state.y_m").as_double();
  x[StateVectorValidationModel::psi_rad] =
    param_manager.get_parameter_value("initial_state.psi_rad").as_double();
  x[StateVectorValidationModel::v_mps] =
    param_manager.get_parameter_value("initial_state.vehicle_dynamics.v_x_mps").as_double();
}
void VehicleModelValidation::declare_parameters()
{
  param_manager.declare_parameter(
    "integration_step_size_s", 0.001, tam::types::param::ParameterType::DOUBLE,
    "Integration step size in s");
  param_manager.declare_parameter(
    "vehicle_parameters.dist_cg_rear_m", 1.5, tam::types::param::ParameterType::DOUBLE,
    "Distance from the center of gravity to the front axle");
  param_manager.declare_parameter(
    "vehicle_parameters.dist_cg_front_m", 1.5, tam::types::param::ParameterType::DOUBLE,
    "Distance from the center of gravity to the front axle");
  param_manager.declare_parameter(
    "initial_state.x_m", 0.0, tam::types::param::ParameterType::DOUBLE,
    "Initial x Position of the vehicle in m");
  param_manager.declare_parameter(
    "initial_state.y_m", 0.0, tam::types::param::ParameterType::DOUBLE,
    "Initial y Position of the vehicle in m");
  param_manager.declare_parameter(
    "initial_state.psi_rad", 0.0, tam::types::param::ParameterType::DOUBLE,
    "Initial heading of the vehicle in rad");
  param_manager.declare_parameter(
    "initial_state.vehicle_dynamics.v_x_mps", 20.0, tam::types::param::ParameterType::DOUBLE,
    "Initial velocity of the vehicle in mps");
  param_manager.declare_parameter(
    "", 0.0, tam::types::param::ParameterType::DOUBLE, "Initial x Position of the vehicle in m");

  // Declare a few values required for sim

  param_manager.declare_parameter(
    "vehicle_dynamics_double_track.m", 782.0, tam::types::param::ParameterType::DOUBLE, "");

  param_manager.declare_parameter(
    "vehicle_dynamics_double_track.l_f", 1.5, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "vehicle_dynamics_double_track.l", 3.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "vehicle_dynamics_double_track.b_r", 1.5, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "vehicle_dynamics_double_track.rr_w_f", 0.3, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "vehicle_dynamics_double_track.rr_w_r", 0.3, tam::types::param::ParameterType::DOUBLE, "");

  // Setter values
  param_manager.declare_parameter(
    "steering_actuator.static_offset_rad", 0.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "initial_state.drivetrain.omega_FL_radps", 0.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "initial_state.drivetrain.omega_FR_radps", 0.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "initial_state.drivetrain.omega_RL_radps", 0.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager.declare_parameter(
    "initial_state.drivetrain.omega_RR_radps", 0.0, tam::types::param::ParameterType::DOUBLE, "");
};
}  // namespace tam::sim

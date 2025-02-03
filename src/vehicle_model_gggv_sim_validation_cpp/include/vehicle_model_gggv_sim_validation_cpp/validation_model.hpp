// Copyright 2025 Simon Sagmeister

#pragma once
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "param_manager_cpp/param_manager.hpp"
#include "param_manager_cpp/param_manager_base.hpp"
#include "vehicle_model_base_cpp/vehicle_model_base.h"
#include "vehicle_model_gggv_sim_validation_cpp/helpers.hpp"
namespace tam::sim
{
struct StateVectorValidationModel
{
  enum {
    x_m,
    y_m,
    psi_rad,
    v_mps,
    CNT_LENGTH_STATE_VECTOR,
  };
};
class VehicleModelValidation : public tam::interfaces::VehicleModelBase
{
private:
  struct IMR
  {
    double Fx_request = 0;
    double Fy_request = 0;
    double F_request = 0;
    double F_actual = 0;
    double Fx = 0;
    double Fy = 0;
    double ay = 0;
    double yaw_rate = 0;
  };
  IMR imr;
  // region input and outputs
  tam::core::ParamManager param_manager;
  tam::types::DriverInput driver_input;
  tam::types::ExternalInfluences external_influences;
  tam::types::VehicleModelOutput model_output;
  // debugging
  tam::types::common::TUMDebugContainer::SharedPtr debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();
  Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1> x;

  // region functions for structring
  void declare_parameters();
  Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1> ode(
    double t, Eigen::Matrix<double, StateVectorValidationModel::CNT_LENGTH_STATE_VECTOR, 1> x_);

public:
  VehicleModelValidation();

  // Model step function
  void step() override;

  // Setter
  void set_driver_input(const tam::types::DriverInput & input) override;
  void set_external_influences(const tam::types::ExternalInfluences & input) override;

  // Vehicle dynamic state:
  tam::types::VehicleModelOutput get_output() const override;

  // Debug Output
  tam::types::common::TUMDebugContainer::SharedPtr get_debug_out() const override;

  // Parameter handling
  tam::interfaces::ParamManagerBase * get_param_manager() override;

  void reset();
};
}  // namespace tam::sim

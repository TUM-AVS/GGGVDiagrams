// Copyright 2023 Simon Sagmeister
#pragma once
#include <vector>

#include "tum_types_cpp/common.hpp"
namespace tam::types
{
struct DriverInput
{
  double steering_angle_rad = 0;  // Left is positive
  common::DataPerWheel<double> wheel_torque_Nm = {0, 0, 0, 0};
};
struct ExternalInfluences
{
  common::Vector3D<double> external_force_N = {0, 0, 0};    // On the vehicle body
  common::Vector3D<double> external_torque_Nm = {0, 0, 0};  // On the vehicle body
  common::DataPerWheel<double> z_height_road_m = {0, 0, 0, 0};
  common::DataPerWheel<double> lambda_mue = {1, 1, 1, 1};
};
struct ActuatorInput
{
  double steering_angle_request = 0;
  double brake_pressure_request = 0;
};
struct ActuatorOutput
{
  double steering_angle = 0;
  double brake_pressure = 0;
};
struct DriveTrainInput
{
  double throttle;  // 0 to 1
  double brake_pressure_bar;
  common::DataPerWheel<double>
    long_tire_force_N;  // Necessary to calc the angular acceleration of the indivial wheels
  uint8_t gear;
};
struct DriveTrainOutput
{
  common::DataPerWheel<double> drive_torque_Nm = {0, 0, 0, 0};
  common::DataPerWheel<double> wheel_speed_radps = {0, 0, 0, 0};
};
struct VehicleDynamisModelInput
{
  common::DataPerWheel<double> wheel_speed_radps = {0, 0, 0, 0};
  double steering_angle = 0;
};
struct VehicleDynamicsModelOutput
{
  // Odometry
  common::Vector3D<double> position_m = {0, 0, 0};
  common::Vector3D<double> velocity_mps = {0, 0, 0};
  common::Vector3D<double> acceleration_mps2 = {0, 0, 0};
  common::Vector3D<double> orientation_rad = {0, 0, 0};
  common::Vector3D<double> angular_velocity_radps = {0, 0, 0};
  common::Vector3D<double> angular_accelaration_radps2 = {0, 0, 0};
  // Long Tire Forces
  common::DataPerWheel<double> long_tire_force_N;
};
struct VehicleModelOutput
{
  double steering_angle;                                             // From actuation model
  common::DataPerWheel<double> brake_pressure_Pa = {0, 0, 0, 0};     // From actuation model
  common::DataPerWheel<double> drive_torque_Nm = {0, 0, 0, 0};       // From drivetrain model
  common::DataPerWheel<double> wheel_speed_radps = {0, 0, 0, 0};     // From drivetrain model
  common::Vector3D<double> position_m = {0, 0, 0};                   // From vehicle dynamics model
  common::Vector3D<double> velocity_mps = {0, 0, 0};                 // From vehicle dynamics model
  common::Vector3D<double> acceleration_mps2 = {0, 0, 0};            // From vehicle dynamics model
  common::Vector3D<double> orientation_rad = {0, 0, 0};              // From vehicle dynamics model
  common::Vector3D<double> angular_velocity_radps = {0, 0, 0};       // From vehicle dynamics model
  common::Vector3D<double> angular_accelaration_radps2 = {0, 0, 0};  // From vehicle dynamics model
  double omega_engine_radps = 0;
  double gear_engaged = 0;
};
}  // namespace tam::types

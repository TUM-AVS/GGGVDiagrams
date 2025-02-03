// Copyright 2025 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "tum_sim_types_cpp/types.hpp"

namespace py = pybind11;
namespace typ = tam::types;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::class_<typ::DriverInput>(m, "DriverInput")
    .def(py::init())
    .def_readwrite("steering_angle_rad", &typ::DriverInput::steering_angle_rad)
    .def_readwrite("wheel_torque_Nm", &typ::DriverInput::wheel_torque_Nm);
  py::class_<typ::VehicleModelOutput>(m, "VehicleModelOutput")
    .def(py::init())
    .def_readwrite("steering_angle", &typ::VehicleModelOutput::steering_angle)
    .def_readwrite("brake_pressure_Pa", &typ::VehicleModelOutput::brake_pressure_Pa)
    .def_readwrite("drive_torque_Nm", &typ::VehicleModelOutput::drive_torque_Nm)
    .def_readwrite("wheel_speed_radps", &typ::VehicleModelOutput::wheel_speed_radps)
    .def_readwrite("position_m", &typ::VehicleModelOutput::position_m)
    .def_readwrite("velocity_mps", &typ::VehicleModelOutput::velocity_mps)
    .def_readwrite("acceleration_mps2", &typ::VehicleModelOutput::acceleration_mps2)
    .def_readwrite("orientation_rad", &typ::VehicleModelOutput::orientation_rad)
    .def_readwrite("angular_velocity_radps", &typ::VehicleModelOutput::angular_velocity_radps)
    .def_readwrite(
      "angular_accelaration_radps2", &typ::VehicleModelOutput::angular_accelaration_radps2)
    .def_readwrite("omega_engine_radps", &typ::VehicleModelOutput::omega_engine_radps)
    .def_readwrite("gear_engaged", &typ::VehicleModelOutput::gear_engaged);
  py::class_<typ::ExternalInfluences>(m, "ExternalInfluences")
    .def(py::init())
    .def_readwrite("external_force_N", &typ::ExternalInfluences::external_force_N)
    .def_readwrite("external_torque_Nm", &typ::ExternalInfluences::external_torque_Nm)
    .def_readwrite("z_height_road_m", &typ::ExternalInfluences::z_height_road_m)
    .def_readwrite("lambda_mue", &typ::ExternalInfluences::lambda_mue);
};

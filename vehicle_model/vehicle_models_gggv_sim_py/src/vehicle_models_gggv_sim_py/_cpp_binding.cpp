// Copyright 2025 Simon Sagmeister

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>
#include <vehicle_model_gggv_sim_validation_cpp/validation_model.hpp>

#define PYBIND_VEHICLE_MODEL_BY_INTERFACE(                                         \
  vehicle_model_class, vehicle_model_name, pybind_module_name)                     \
  pybind11::class_<vehicle_model_class>(pybind_module_name, vehicle_model_name)    \
    .def("step", &vehicle_model_class::step)                                       \
    .def("set_driver_input", &vehicle_model_class::set_driver_input)               \
    .def("set_external_influences", &vehicle_model_class::set_external_influences) \
    .def("get_output", &vehicle_model_class::get_output)                           \
    .def("get_debug_out", &vehicle_model_class::get_debug_out)                     \
    .def(                                                                          \
      "get_param_manager", &vehicle_model_class::get_param_manager,                \
      py::return_value_policy::reference_internal)                                 \
    .def("reset", &vehicle_model_class::reset)

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  PYBIND_VEHICLE_MODEL_BY_INTERFACE(tam::sim::VehicleModelValidation, "VehicleModelValidation", m)
    .def(py::init());
};

// Copyright 2023 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "param_manager_cpp/param_manager_base.hpp"
// #include "param_manager_cpp/param_manager.hpp"
// #include "param_manager_cpp/param_manager_composer.hpp"
// #include "param_manager_cpp/param_manager_simulink.hpp"
// include "pybind11_test/test_struct.hpp"

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::enum_<tam::types::param::ParameterType>(m, "ParameterType")
    .value("BOOL", tam::types::param::ParameterType::BOOL)
    .value("INTEGER", tam::types::param::ParameterType::INTEGER)
    .value("DOUBLE", tam::types::param::ParameterType::DOUBLE)
    .value("STRING", tam::types::param::ParameterType::STRING)
    .value("BOOL_ARRAY", tam::types::param::ParameterType::BOOL_ARRAY)
    .value("INTEGER_ARRAY", tam::types::param::ParameterType::INTEGER_ARRAY)
    .value("DOUBLE_ARRAY", tam::types::param::ParameterType::DOUBLE_ARRAY)
    .value("STRING_ARRAY", tam::types::param::ParameterType::STRING_ARRAY);

  py::class_<tam::types::param::ParameterValue>(m, "ParameterValue")
    .def(py::init<tam::types::param::ParameterType, tam::types::param::param_variant_t>())
    .def(
      "set_parameter_value", &tam::types::param::ParameterValue::set_parameter_value,
      "Set a parameter")
    .def("get_type", &tam::types::param::ParameterValue::get_type, "Get the type of the parameter")
    .def("as_bool", &tam::types::param::ParameterValue::as_bool, "Get parameter value as bool")
    .def("as_int", &tam::types::param::ParameterValue::as_int, "Get parameter value as int")
    .def(
      "as_double", &tam::types::param::ParameterValue::as_double, "Get parameter value as double")
    .def(
      "as_string", &tam::types::param::ParameterValue::as_string, "Get parameter value as string")
    .def(
      "as_bool_array", &tam::types::param::ParameterValue::as_bool_array,
      "Get parameter value as bool array")
    .def(
      "as_int_array", &tam::types::param::ParameterValue::as_int_array,
      "Get parameter value as int array")
    .def(
      "as_double_array", &tam::types::param::ParameterValue::as_double_array,
      "Get parameter value as double array")
    .def(
      "as_string_array", &tam::types::param::ParameterValue::as_string_array,
      "Get parameter value as string array");
  py::class_<tam::interfaces::ParamManagerBase, std::shared_ptr<tam::interfaces::ParamManagerBase>>(
    m, "ParamManagerBase")
    .def(
      "declare_parameter", &tam::interfaces::ParamManagerBase::declare_parameter,
      "Declare a parameter")
    .def(
      "set_parameter_value", &tam::interfaces::ParamManagerBase::set_parameter_value,
      "Set a parameter")
    .def(
      "get_parameter_value", &tam::interfaces::ParamManagerBase::get_parameter_value,
      "Get a parameter value")
    .def(
      "list_parameters", &tam::interfaces::ParamManagerBase::list_parameters,
      "List the available parameters")
    .def(
      "has_parameter", &tam::interfaces::ParamManagerBase::has_parameter,
      "Return whether a parameter was declared")
    .def(
      "get_parameter_description", &tam::interfaces::ParamManagerBase::get_parameter_description,
      "Return the description of a parameter")
    .def(
      "get_parameter_type", &tam::interfaces::ParamManagerBase::get_parameter_type,
      "Return the declare param type");
}

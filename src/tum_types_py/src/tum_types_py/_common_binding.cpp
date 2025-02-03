// Copyright 2025 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

namespace py = pybind11;
namespace typ = tam::types;
//
PYBIND11_MODULE(_common_binding, m)
{
  py::class_<tam::types::common::Header>(m, "Header")
    .def(py::init())
    .def_readwrite("seq", &tam::types::common::Header::seq)
    .def_readwrite("time_stamp_ns", &tam::types::common::Header::time_stamp_ns)
    .def_readwrite("frame_id", &tam::types::common::Header::frame_id);
  py::class_<tam::types::common::Vector3D<double>>(m, "Vector3D")
    .def(py::init())
    .def(py::init<double, double, double>())
    .def_readwrite("x", &tam::types::common::Vector3D<double>::x)
    .def_readwrite("y", &tam::types::common::Vector3D<double>::y)
    .def_readwrite("z", &tam::types::common::Vector3D<double>::z);
  py::class_<tam::types::common::Vector2D<double>>(m, "Vector2D")
    .def(py::init())
    .def(py::init<double, double>())
    .def_readwrite("x", &tam::types::common::Vector2D<double>::x)
    .def_readwrite("y", &tam::types::common::Vector2D<double>::y);
  py::class_<tam::types::common::DataPerWheel<double>>(m, "DataPerWheel")
    .def(py::init())
    .def(py::init<double, double, double, double>())
    .def_readwrite("front_left", &tam::types::common::DataPerWheel<double>::front_left)
    .def_readwrite("front_right", &tam::types::common::DataPerWheel<double>::front_right)
    .def_readwrite("rear_left", &tam::types::common::DataPerWheel<double>::rear_left)
    .def_readwrite("rear_right", &tam::types::common::DataPerWheel<double>::rear_right);
  py::class_<tam::types::common::EulerYPR>(m, "EulerYPR")
    .def(py::init<double, double, double>())
    .def_readwrite("yaw", &tam::types::common::EulerYPR::yaw)
    .def_readwrite("pitch", &tam::types::common::EulerYPR::pitch)
    .def_readwrite("roll", &tam::types::common::EulerYPR::roll);
  py::class_<
    tam::types::common::TUMDebugContainer,
    std::shared_ptr<tam::types::common::TUMDebugContainer>  // This holder type is important
                                                            // otherwise python casts into unique
                                                            // ptr and deallocates the memory
    // See https://pybind11.readthedocs.io/en/stable/advanced/smart_ptrs.html section shared_ptr
    >(m, "TUMDebugContainer")
    .def(py::init())
    .def("get_signal_names", &tam::types::common::TUMDebugContainer::get_signal_names)
    .def("get_values", &tam::types::common::TUMDebugContainer::get_values)
    .def("get_signal_value", &tam::types::common::TUMDebugContainer::get_signal_value)
    .def("get_config_hash", &tam::types::common::TUMDebugContainer::get_config_hash)
    .def(
      "log", pybind11::overload_cast<const std::string &, const float &>(
               &tam::types::common::TUMDebugContainer::log))
    .def(
      "log", pybind11::overload_cast<
               const std::string &, const tam::types::common::DataPerWheel<double> &>(
               &tam::types::common::TUMDebugContainer::log))
    .def(
      "log",
      pybind11::overload_cast<const std::string &, const tam::types::common::Vector3D<double> &>(
        &tam::types::common::TUMDebugContainer::log));
}

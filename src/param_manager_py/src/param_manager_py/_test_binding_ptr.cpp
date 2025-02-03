// Copyright 2024 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <variant>
#include <vector>
struct Params
{
  double a, b, c;
  Params() { print_memory_adresses(); }
  void print_memory_adresses()
  {
    std::cout << "a: " << &a << std::endl;
    std::cout << "b: " << &b << std::endl;
    std::cout << "c: " << &c << std::endl;
    std::cout << "this: " << this << std::endl;
  }
};
// void print_ptr(std::variant<double *, std::vector<double> *> ptr)
// {
//   std::cout << "Given ptr: " << std::get<double *>(ptr) << std::endl;
// }
void print_ptr(Params * ptr) { std::cout << "Given ptr: " << ptr << std::endl; }
void set_param_values(Params * p)
{
  p->a = 7;
  p->b = 9;
  p->c = 11;
}
namespace py = pybind11;
PYBIND11_MODULE(_test_binding, m)
{
  m.def("print_ptr", &print_ptr, "");
  m.def("set_param_values", &set_param_values, "");
  py::class_<Params>(m, "Params")
    .def(py::init<>())
    .def_readwrite("a", &Params::a)
    .def_readwrite("b", &Params::b)
    .def_readwrite("c", &Params::c)
    .def("print_memory_adresses", &Params::print_memory_adresses, "");
}

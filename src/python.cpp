#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "cartesian_franka/robot.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pycartesian_franka, m)
{
    m.doc() = "Basic Franka Emika interface, based on libfranka";

    using namespace cartesian_franka;
    py::class_<Robot>(m, "Robot")
        .def(py::init<const std::string&, double>(),
            py::arg("ip"),
            py::arg("duration") = 0.5)
        .def("init", &Robot::init,
            py::arg("duration") = 0.5)
        .def("translate", &Robot::translate,
            py::arg("delta"),
            py::arg("duration") = 0.5)
        .def("rotate", &Robot::rotate,
            py::arg("rpy"),
            py::arg("duration") = 0.5)
        .def("move", (void (Robot::*)(const Eigen::Vector3d&, const Eigen::Vector3d&, double)) & Robot::move,
            py::arg("delta"),
            py::arg("rpy"),
            py::arg("duration") = 0.5)
        .def("move", (void (Robot::*)(const Eigen::Affine3d&, double)) & Robot::move,
            py::arg("delta"),
            py::arg("duration") = 0.5)
        .def("affine3d", &Robot::affine3d)
        .def("position", &Robot::position)
        .def("orientation", &Robot::orientation);
}
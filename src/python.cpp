#include "Bezier/bezier.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace Bezier;
namespace py = pybind11;


PYBIND11_MODULE(bezier, m) {
    py::class_<Curve>(m, "Curve")
        .def(py::init<const Eigen::MatrixX2d&>())
        .def("controlPoints", &Curve::controlPoints)
        .def("polyline", &Curve::polyline);
}
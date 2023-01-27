#include "Bezier/bezier.h"
#include "Bezier/polycurve.h"

#include <pybind11/detail/common.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace Bezier;
namespace py = pybind11;

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

class PyCurve : public Curve {
 public:
  PyCurve(const Eigen::MatrixX2d& points) : Curve(points) {}

  Curve py_derivative(uint n) const { return *Curve::derivative(n); }
};

class PyPolyCurve : public PolyCurve {
 public:
  PyPolyCurve(std::vector<std::shared_ptr<Curve>>& curve_list) : PolyCurve(curve_list){};


};

PYBIND11_MODULE(bezier, m) {
  py::class_<PyCurve>(m, "Curve")
      .def(py::init<const Eigen::MatrixX2d&>())
      .def("controlPoints", &PyCurve::controlPoints)
      .def("derivative", &PyCurve::py_derivative)
      .def("polyline", &PyCurve::polyline)
      .def("endPoints", &PyCurve::endPoints)
      .def("applyContinuity", &PyCurve::applyContinuity);

  py::class_<PolyCurve>(m, "PolyCurve")
      .def(py::init<>())
      .def("insertBack", &PolyCurve::insertBack)
      .def("removeBack", &PolyCurve::removeBack)
      .def("polyline", &PolyCurve::polyline);
}
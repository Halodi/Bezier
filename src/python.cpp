#include "Bezier/bezier.h"
#include "Bezier/polycurve.h"

#include <pybind11/detail/common.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "Bezier/BezierTrajectory2D.h"
#include "Bezier/SplineVelocity.h"

using namespace Bezier;
namespace py = pybind11;

class PyCurve : public Curve {
 public:
  PyCurve(const Eigen::MatrixX2d& points) : Curve(points) {}

  Curve py_derivative(int n) const { return *Curve::derivative(n); }

  void py_applyContinuity(PyCurve source_curve) {
    std::vector<double> beta_coeffs = {1, 0, 0};
    applyContinuity(source_curve, beta_coeffs);
  }
};

class PyPolyCurve : public PolyCurve {
 public:
  PyPolyCurve() : PolyCurve(){};

  void py_insertBack(PyCurve curve) {
    auto ptr = std::make_shared<Curve>(curve);
    PolyCurve::insertBack(ptr);
  }
};

PYBIND11_MODULE(bezier, m) {
  py::class_<PyCurve>(m, "Curve")
      .def(py::init<const Eigen::MatrixX2d&>())
      .def("controlPoints", &PyCurve::controlPoints)
      .def("derivative", &PyCurve::py_derivative)
      .def("polyline", &PyCurve::polyline)
      .def("endPoints", &PyCurve::endPoints)
      .def("applyContinuity", &PyCurve::py_applyContinuity);

  py::class_<PyPolyCurve>(m, "PolyCurve")
      .def(py::init<>())
      .def("insertBack", &PyPolyCurve::py_insertBack)
      .def("removeBack", &PyPolyCurve::removeBack)
      .def("polyline", &PyPolyCurve::polyline);

    py::class_<HermiteVelocityProfile>(m, "HermiteVelocityProfile")
        .def(py::init<scalar_t, scalar_t, scalar_t, scalar_t>());

    py::class_<BezierSegment>(m, "BezierSegment")
        .def(py::init<vector2_array_t, HermiteVelocityProfile>());

    py::class_<BezierTrajectory2D>(m, "BezierTrajectory2D")
        .def(py::init<std::vector<BezierSegment>>())
        .def("getPosition", &BezierTrajectory2D::getPosition)
        .def("getHeading", &BezierTrajectory2D::getHeading)
        .def("getVelocity", &BezierTrajectory2D::getVelocity)
        .def("getVelocityAbs", &BezierTrajectory2D::getVelocityAbs)
        .def("getAngularVelocity", &BezierTrajectory2D::getAngularVelocity);
}
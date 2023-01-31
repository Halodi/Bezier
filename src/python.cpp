#include "Bezier/bezier.h"
#include "Bezier/polycurve.h"

#include <pybind11/detail/common.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ocs2_core/misc/LinearInterpolation.h>

using namespace Bezier;
namespace py = pybind11;

using scalar_t = double;
/** 2-size vector type. */
using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
/** 2 vector's array type. */
using vector2_array_t = std::vector<vector2_t>;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

// Implementation for 2D bezier curves
struct BezierSegment {
  vector2_array_t bezierPoints;
  scalar_t desiredStartTime;
  scalar_t desiredEndTime;

  BezierSegment(vector2_array_t bezierPoints, scalar_t desiredStartTime, scalar_t desiredEndTime)
      : bezierPoints(bezierPoints), desiredStartTime(desiredStartTime), desiredEndTime(desiredEndTime) {}
};

class LinearSplineVelocity {
 public:
  // TODO: Implement proper structure for default constructor.
  LinearSplineVelocity()
      : anchorTimes_(vector_t::Zero(2)), anchor_dxdt_(vector_t::Zero(2)), segment_dxdtt(vector_t::Zero(1)){};
  /// \brief Constract a linear velocity profile from the desired times at each anchor (vector of size N + 1)
  LinearSplineVelocity(const vector_t& anchorTimes, const vector_t& segmentLength, scalar_t startVelocity = 0)
      : anchorTimes_(anchorTimes) {
    assert(anchorTimes.size() == segmentLength.size() + 1 &&
           "anchorTimes needs to contain one more element than segmentLenght!");
    n_segments_ = anchorTimes.size() - 1;
    int n_anchors = n_segments_ + 1;
    anchor_dxdt_.resize(n_anchors);
    segment_dxdtt.resize(n_segments_);
    anchorTimes_[0] = startVelocity;
    for (int i = 0; i < n_segments_; i++) {
      scalar_t averageSegment_dxdt = 1 / (anchorTimes[i + 1] - anchorTimes[i]);
      anchor_dxdt_[i + 1] = 2 * averageSegment_dxdt - anchor_dxdt_[i];
      segment_dxdtt[i] = 0.5 * averageSegment_dxdt;
    }
    std::cout << "anchor dx/dt: " << anchor_dxdt_.transpose() << std::endl;
  }
  // returns the normalized running variable of the underlying spline(ranging from n-1 to n to parametrize position
  // along curve n)
  scalar_t getBezierRunningVariable(scalar_t time) const {
    const std::vector<scalar_t> anchorTimeVec(anchorTimes_.data(),
                                              anchorTimes_.data() + anchorTimes_.rows() * anchorTimes_.cols());
    const std::vector<scalar_t> anchorVelVec(anchor_dxdt_.data(),
                                             anchor_dxdt_.data() + anchor_dxdt_.rows() * anchor_dxdt_.cols());

    return ocs2::LinearInterpolation::interpolate(time, anchorTimeVec, anchorVelVec);
  }

  scalar_t getVelocity(scalar_t time) const {
    const std::vector<scalar_t> anchorTimeVec(anchorTimes_.data(),
                                              anchorTimes_.data() + anchorTimes_.rows() * anchorTimes_.cols());
    const std::vector<scalar_t> anchorVelVec(anchor_dxdt_.data(),
                                             anchor_dxdt_.data() + anchor_dxdt_.rows() * anchor_dxdt_.cols());

    return ocs2::LinearInterpolation::interpolate(time, anchorTimeVec, anchorVelVec);
  }

  scalar_t getAnchorTime(int anchorIndex) const { return anchorTimes_[anchorIndex]; };

 private:
  int n_segments_;
  vector_t anchorTimes_;   // of size N + 1 segments
  vector_t anchor_dxdt_;   // of size N + 1 segments
  vector_t segment_dxdtt;  // of size N, precomputes the factor 0.5 / (t_{n+1} - t_{n}) for each curve segment.
};

class BezierTrajectory2D {
 public:
  BezierTrajectory2D(std::vector<BezierSegment> bezierSegments) {
    n_curves_ = bezierSegments.size();
    std::vector<std::shared_ptr<Bezier::Curve>> curves;
    curves.reserve(n_curves_);

    vector_t anchorTimes(n_curves_ + 1);
    vector_t segmentLengths(n_curves_);

    anchorTimes[0] = bezierSegments[0].desiredStartTime;

    for (int i = 0; i < n_curves_; i++) {
      const BezierSegment& currBezierSegment = bezierSegments[i];
      curves.push_back(std::shared_ptr<Bezier::Curve>(new Bezier::Curve(currBezierSegment.bezierPoints)));
      // timing needs to be continuous between segments for now. Can be adapted in the future.
      std::cout << bezierSegments[i].desiredEndTime << std::endl;
      std::cout << bezierSegments[i + 1].desiredStartTime << std::endl;
      // Check that start and end time of adjacent segments align if not at the end point
      if (i < n_curves_ - 1) assert(bezierSegments[i].desiredEndTime == bezierSegments[i + 1].desiredStartTime);
      anchorTimes[i + 1] = bezierSegments[i].desiredEndTime;
      segmentLengths[i] = curves[i]->length();
    }
    polyCurve_ = Bezier::PolyCurve(curves);
    velocityProfile_ = LinearSplineVelocity(anchorTimes, segmentLengths);
  }

  vector2_t getValueAtTime(scalar_t time) const {
    return polyCurve_.valueAt(velocityProfile_.getBezierRunningVariable(time));
  }

  scalar_t getFinalTime() const { return velocityProfile_.getAnchorTime(n_curves_); };
  int getNumberOfCurves() const { return n_curves_; }

 private:
  Bezier::PolyCurve polyCurve_;           // contains the geometric part of the reference curve
  LinearSplineVelocity velocityProfile_;  // contains the temporal part of the reference curve

  int n_curves_;
};

class PyCurve : public Curve {
 public:
  PyCurve(const Eigen::MatrixX2d& points) : Curve(points) {}

  Curve py_derivative(uint n) const { return *Curve::derivative(n); }

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

  py::class_<BezierSegment>(m, "BezierSegment").def(py::init<vector2_array_t, scalar_t, scalar_t>());

  py::class_<BezierTrajectory2D>(m, "BezierTrajectory2D")
      .def(py::init<std::vector<BezierSegment>>())
      .def("getValueAtTime", &BezierTrajectory2D::getValueAtTime);
}
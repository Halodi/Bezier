#pragma once

#include <Bezier/bezier.h>
#include <Bezier/polycurve.h>

#include "Bezier/SplineVelocity.h"

using scalar_t = double;
/** 2-size vector type. */
using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
/** 2 vector's array type. */
using vector2_array_t = std::vector<vector2_t>;

/** 3-size vector type. */
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;

using matrix4_t = Eigen::Matrix<scalar_t, 4, 4>;

/** Dynamic-size matrix type. */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

struct HermiteVelocityProfile {
  scalar_t desiredStartTime;
  scalar_t desiredStartVel;
  scalar_t desiredEndTime;
  scalar_t desiredEndVel;

  HermiteVelocityProfile(scalar_t desiredStartTime, scalar_t desiredStartVel, scalar_t desiredEndTime,
                         scalar_t desiredEndVel)
      : desiredStartTime(desiredStartTime),
        desiredStartVel(desiredStartVel),
        desiredEndTime(desiredEndTime),
        desiredEndVel(desiredEndVel){};

  vector4_t getTimingVec() { return vector4_t(desiredStartTime, desiredStartVel, desiredEndTime, desiredEndVel); };
};

// Implementation for 2D bezier curves
struct BezierSegment {
  vector2_array_t bezierPoints;
  HermiteVelocityProfile timing;

  BezierSegment(vector2_array_t bezierPoints, HermiteVelocityProfile timing)
      : bezierPoints(bezierPoints), timing(timing) {}
};

class BezierTrajectory2D {
 public:
  // Default constructor
  BezierTrajectory2D();

  BezierTrajectory2D(std::vector<BezierSegment> bezierSegments);

  void setBezierSegments(std::vector<BezierSegment> bezierSegments);

  /// \brief Get position [x, y] in world frame at time t
  vector2_t getPosition(scalar_t time) const { return polyCurvePtr_->valueAt(velocityProfile_->getParameter(time)); };

  /// \brief Get mathematically positive (CCW) heading relative to world frame at time t
  scalar_t getHeading(scalar_t time) const;

  /// \brief Get velocity [v_x, v_y] in world frame at time t
  vector2_t getVelocity(scalar_t time) const {
    return polyCurvePtr_->derivativeAt(velocityProfile_->getParameter(time)) * getPolyCurveParameterDerivative(time);
  };

  /// \brief Get angular velocity around z axis of world frame at time t
  scalar_t getAngularVelocity(scalar_t time) const {
    return polyCurvePtr_->curvatureDerivativeAt(velocityProfile_->getParameter(time)) *
           getPolyCurveParameterDerivative(time);
  };

  /// \brief Get absolute velocity along curve (positive for moving forward along the curve) at time t
  scalar_t getVelocityAbs(scalar_t time) const {
    return polyCurvePtr_->derivativeAt(velocityProfile_->getParameter(time)).norm() *
           getPolyCurveParameterDerivative(time);
  };

  /// \brief the current curve parameter phi used to evaluate the underlying geometric curve. Element [n-1, n] on curve
  /// segment n.
  scalar_t getPolyCurveParameter(scalar_t time) const { return velocityProfile_->getParameter(time); };

  /// \brief the current curve parameter derivative d phi/dt
  scalar_t getPolyCurveParameterDerivative(scalar_t time) const {
    return velocityProfile_->getParameterVelocity(time);
  };

  /// \brief Time where the curve starts.
  scalar_t getInitialTime() const { return velocityProfile_->getAnchorTime(0); };

  /// \brief Time where the endpoint of the curve is reched.
  scalar_t getFinalTime() const { return velocityProfile_->getAnchorTime(n_curves_); };

  int getNumberOfCurves() const { return n_curves_; }

 private:
  std::unique_ptr<Bezier::PolyCurve> polyCurvePtr_;      // contains the geometric part of the reference curve
  std::unique_ptr<SplineVelocityBase> velocityProfile_;  // contains the temporal part of the reference curve

  int n_curves_;
};
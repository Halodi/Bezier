#include "Bezier/BezierTrajectory2D.h"

#include <iostream>

BezierTrajectory2D::BezierTrajectory2D() : velocityProfile_(new SplineVelocityBase(vector2_t::Zero())), n_curves_(0) {
  std::shared_ptr<Bezier::Curve> zeroCurve(new Bezier::Curve(matrix_t::Zero(4, 2)));
  polyCurve_ = Bezier::PolyCurve(zeroCurve);
};

BezierTrajectory2D::BezierTrajectory2D(std::vector<BezierSegment> bezierSegments) {
  setBezierSegments(bezierSegments);
}

void BezierTrajectory2D::setBezierSegments(std::vector<BezierSegment> bezierSegments) {
  n_curves_ = bezierSegments.size();
  std::vector<std::shared_ptr<Bezier::Curve>> curves;
  curves.reserve(n_curves_);

  vector_t anchorTimes(n_curves_ + 1);
  vector_t anchorVelocities(n_curves_ + 1);

  anchorTimes[0] = bezierSegments[0].timing.desiredStartTime;
  anchorVelocities[0] = bezierSegments[0].timing.desiredStartVel;

  for (int i = 0; i < n_curves_; i++) {
    const BezierSegment& currBezierSegment = bezierSegments[i];
    curves.push_back(std::shared_ptr<Bezier::Curve>(new Bezier::Curve(currBezierSegment.bezierPoints)));
    // timing needs to be continuous between segments for now. Can be adapted in the future.
    std::cout << bezierSegments[i].timing.desiredEndTime << std::endl;
    std::cout << bezierSegments[i + 1].timing.desiredStartTime << std::endl;
    // Check that start and end time & velocity of adjacent segments align if not at the end point
    if (i < n_curves_ - 1) {
      assert(bezierSegments[i].timing.desiredEndTime == bezierSegments[i + 1].timing.desiredStartTime);
      assert(bezierSegments[i].timing.desiredEndVel == bezierSegments[i + 1].timing.desiredStartVel);
    }
    anchorTimes[i + 1] = bezierSegments[i].timing.desiredEndTime;
    anchorVelocities[i + 1] = bezierSegments[i].timing.desiredEndVel;
  }
  polyCurve_ = Bezier::PolyCurve(curves);
  vector_t anchor_dx_dt = anchorVelocities;
  for (int i = 0; i < n_curves_ + 1; i++) {
    anchor_dx_dt[i] = anchor_dx_dt[i] / polyCurve_.derivativeAt(i).norm();
  }
  velocityProfile_.reset(new HermiteSplineVelocity(anchorTimes, anchor_dx_dt));
}

scalar_t BezierTrajectory2D::getHeading(scalar_t time) const {
  const vector_t tangent = polyCurve_.tangentAt(velocityProfile_->getParameter(time), false);
  return atan2(tangent[1], tangent[0]);
};
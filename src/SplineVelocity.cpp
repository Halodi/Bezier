
#include "Bezier/SplineVelocity.h"

#include <algorithm>
#include <functional>
#include <iostream>

SplineVelocityBase::SplineVelocityBase(const vector_t& anchorTimes)
    : anchorTimes_(anchorTimes), n_segments_((anchorTimes.size() - 1)){};

int SplineVelocityBase::getSegmentIndex(scalar_t time) const {
  int segmentIndex = 0;
  while (time > anchorTimes_[segmentIndex + 1]) segmentIndex++;
  return segmentIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

HermiteSplineVelocity::HermiteSplineVelocity()
    : SplineVelocityBase(vector_t::Zero(2)),
      hermiteMapping_(matrix4_t::Zero()),
      hermiteDerivativeMapping_(matrix3_4_t::Zero()),
      bezierPoints_({vector4_t::Zero()}){};

HermiteSplineVelocity::HermiteSplineVelocity(const vector_t& anchorTimes, const vector_t& anchor_dx_dt)
    : SplineVelocityBase(anchorTimes) {
  // clang-format off
  hermiteMapping_ << 1.0,  0.0,  0.0,  0.0,
                     0.0,  1.0,  0.0,  0.0,
                    -3.0, -2.0,  3.0, -1.0,
                     2.0,  1.0, -2.0,  1.0;
  // clang-format on
  hermiteDerivativeMapping_ = hermiteMapping_.bottomLeftCorner(3, 4);
  bezierPoints_.reserve(n_segments_);

  for (int i = 0; i < n_segments_; i++) {
    bezierPoints_.push_back(vector4_t(i, anchor_dx_dt[i], i + 1, anchor_dx_dt[i + 1]));
  }
}

scalar_t HermiteSplineVelocity::getParameter(scalar_t time) const {
  time = std::clamp(time, anchorTimes_[0], anchorTimes_[n_segments_]);
  int segmentIndex = getSegmentIndex(time);
  // compute temporal running variable (element [0, 1])
  scalar_t delta_t = getNormalizedTime(time, segmentIndex);
  scalar_t delta_t_square = delta_t * delta_t;
  scalar_t delta_t_cubic = delta_t * delta_t_square;
  const vector4_t timePolynomial(1.0, delta_t, delta_t_square, delta_t_cubic);
  return (timePolynomial.transpose() * hermiteMapping_ * bezierPoints_[segmentIndex]);
};

scalar_t HermiteSplineVelocity::getParameterVelocity(scalar_t time) const {
  time = std::clamp(time, anchorTimes_[0], anchorTimes_[n_segments_]);
  int segmentIndex = getSegmentIndex(time);
  // compute temporal running variable (element [0, 1])
  scalar_t delta_t = getNormalizedTime(time, segmentIndex);
  scalar_t delta_t_square = delta_t * delta_t;
  const vector3_t timeDerivativePolynomial(1.0, 2 * delta_t, 3 * delta_t_square);
  return (timeDerivativePolynomial.transpose() * hermiteDerivativeMapping_ * bezierPoints_[segmentIndex]);
};

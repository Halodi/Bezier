#pragma once
#include <vector>
#include "Eigen/Eigen"
#include <algorithm>
#include <functional>
#include <iostream>


using scalar_t = double;
/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
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

class SplineVelocityBase {
 public:
  SplineVelocityBase() = delete;
  SplineVelocityBase(const vector_t& anchorTimes);

  virtual scalar_t getParameter(scalar_t time) const { return 0; };
  virtual scalar_t getParameterVelocity(scalar_t time) const { return 0; };

  scalar_t getAnchorTime(int anchorIndex) const { return anchorTimes_[anchorIndex]; };
  int getNumberOfSegments() const { return n_segments_; };
  int getSegmentIndex(scalar_t time) const;

 protected:
  vector_t anchorTimes_;  // of size N + 1 segments
  int n_segments_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

class HermiteSplineVelocity final : public SplineVelocityBase {
 public:
  using matrix3_4_t = Eigen::Matrix<scalar_t, 3, 4>;

  HermiteSplineVelocity();
  HermiteSplineVelocity(const vector_t& anchorTimes, const vector_t& anchor_dx_dt);

  scalar_t getParameter(scalar_t time) const override;
  scalar_t getParameterVelocity(scalar_t time) const override;

 private:
  scalar_t getNormalizedTime(scalar_t time, int segmentIndex) const {
    return (time - anchorTimes_[segmentIndex]) / (anchorTimes_[segmentIndex + 1] - anchorTimes_[segmentIndex]);
  };

  matrix4_t hermiteMapping_;
  matrix3_4_t hermiteDerivativeMapping_;
  std::vector<vector4_t> bezierPoints_;
};
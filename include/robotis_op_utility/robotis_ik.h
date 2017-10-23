#ifndef ROBOTIS_OP_UTILITY_ROBOTIS_IK_H_
#define ROBOTIS_OP_UTILITY_ROBOTIS_IK_H_

#include <iostream>
#include <cmath>

#include <robotis_op_utility/eigen_types.h>
#include <robotis_op_utility/math/Matrix.h>
#include <robotis_op_utility/math/Plane.h>
#include <robotis_op_utility/math/Point.h>
#include <robotis_op_utility/math/Vector.h>


namespace robotis {

class RobotisIk {
public:
  RobotisIk() {}
  ~RobotisIk() {}

  bool computeIK(double out[], Affine3& af);
  bool computeIKold(double out[], Affine3& af);
};



} // namespace robotis

#endif
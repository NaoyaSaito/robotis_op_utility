#ifndef ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_
#define ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_

#include <robotis_op_utility/eigen_types.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace robotis {

class RobotisBody {
public:
  RobotisBody();
  const Vector3& calcCenterOfMass();

private:
  robot_model::RobotModelPtr body;
};

} // namespace robotis

#endif

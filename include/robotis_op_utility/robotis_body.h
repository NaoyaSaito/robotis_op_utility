#ifndef ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_
#define ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_

#include <robotis_op_utility/eigen_types.h>

#include <vector>
#include <string>
#include <map>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace robotis {

class RobotisBody {
public:
  RobotisBody(ros::NodeHandle nnh);
  ~RobotisBody();
  const Vector3& calcCenterOfMass();

  std::vector<std::string> link_names;
  std::map<std::string, double> link_masses;

  double total_mass;  // model mass[kg]

private:
  double setModelMass();

  ros::NodeHandle nh;
  robot_model::RobotModelPtr body;
};

} // namespace robotis

#endif

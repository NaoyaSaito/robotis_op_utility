#ifndef ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_
#define ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_

#include <robotis_op_utility/eigen_types.h>

#include <vector>
#include <string>
#include <map>

#include <urdf/model.h>
#include <urdf_model/link.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace robotis {

class RobotisBody {
public:
  RobotisBody(ros::NodeHandle nnh);
  ~RobotisBody();

  Vector3 calcCenterOfMass();
  void update(std::string joint_name[], double position[]);

  std::vector<std::string> link_names;
  std::map<std::string, double> link_masses;  // link_name : link_mass[kg]
  std::map<std::string, Vector3> link_cogs;  // link CoG[m](link coodinate)
  std::map<std::string, Affine3d> link_trans; // link affine matrix

  double total_mass;  // model mass[kg]
  int dof;

private:
  urdf::Model readUrdfFile(const std::string param_name);
  double setModelMass(urdf::Model model);
  void setLinksCoGVector(urdf::Model model);

  ros::NodeHandle nh;
  robot_state::RobotStatePtr body;
};

} // namespace robotis

#endif

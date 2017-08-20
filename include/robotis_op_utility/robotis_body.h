#ifndef ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_
#define ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_

#include <robotis_op_utility/eigen_types.h>

#include <vector>
#include <string>
#include <unordered_map>

#include <urdf/model.h>
#include <urdf_model/link.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace robotis {

class RobotisBody : ros::NodeHandle{
public:
  RobotisBody();
  ~RobotisBody();

  EVector3 calcCenterOfMass();
  void update(const sensor_msgs::JointState& msg);
  void moveInitPosition(const sensor_msgs::JointState& jsmsg);

  std::vector<std::string> link_names;
  std::unordered_map<std::string, double> link_masses;  // link_name : link_mass[kg]
  std::unordered_map<std::string, EVector3> link_cogs;  // link CoG[m](link coodinate)
  std::unordered_map<std::string, Affine3d> link_affine; // link affine matrix
  std::unordered_map<std::string, double> init_joint_angle;

  std::vector<std::string> joint_names;

  double total_mass;  // model mass[kg]
  int dof;

private:
  urdf::Model readUrdfFile(const std::string& param_name);
  double setModelMass(const urdf::Model& model);
  void setLinksCoGVector(const urdf::Model& model);

  robot_state::RobotStatePtr body;
  std::unordered_map<std::string, ros::Publisher> joint_cmd_pubs;
};

} // namespace robotis

#endif

#ifndef ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_
#define ROBOTIS_OP_UTILITY_ROBOTIS_BODY_H_

#include <robotis_op_utility/eigen_types.h>

#include <string>
#include <unordered_map>
#include <vector>

#include <urdf/model.h>
#include <urdf_model/link.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace robotis {

class RobotisBody : ros::NodeHandle {
 public:
  RobotisBody();
  ~RobotisBody() {}

  Vector3 calcCenterOfMass();
  void update(const sensor_msgs::JointState& msg);
  void moveInitPosition(const sensor_msgs::JointState& jsmsg);
  bool calkIKofWalkingMotion(
      const Vector3& com, const Affine3& right_leg, const Affine3& left_leg,
      std::unordered_map<std::string, double>& joint_values);
  void publishJointCommand(
      std::unordered_map<std::string, double>& joint_values);

  std::vector<std::string> link_names;
  std::unordered_map<std::string, double>
      link_masses;  // link_name : link_mass[kg]
  std::unordered_map<std::string, Vector3>
      link_cogs;  // link CoG[m](link coodinate)
  std::unordered_map<std::string, Affine3d> link_affine;  // link affine matrix
  std::unordered_map<std::string, double> init_joint_angle;

  std::vector<std::string> joint_names;

  double total_mass;  // model mass[kg]
  int dof;

 private:
  urdf::Model readUrdfFile(const std::string& param_name);
  double setModelMass(const urdf::Model& model);
  void setLinksCoGVector(const urdf::Model& model);

  robot_state::RobotStatePtr body;
  const robot_state::JointModelGroup* right_leg_joint_group;
  const robot_state::JointModelGroup* left_leg_joint_group;

  std::vector<std::string> right_leg_joint_names;
  std::vector<std::string> left_leg_joint_names;
  std::unordered_map<std::string, ros::Publisher> joint_cmd_pubs;
};

}  // namespace robotis

#endif

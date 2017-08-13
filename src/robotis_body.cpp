#include <robotis_op_utility/robotis_body.h>

#include <ros/ros.h>

namespace robotis {

RobotisBody::RobotisBody(ros::NodeHandle nnh) {
  nh = nnh;

  // load body model to "body"
  const std::string read_urdf_rosparam = "robot_description";
  robot_model_loader::RobotModelLoader robot_model_loader(read_urdf_rosparam);
  robot_model::RobotModelPtr body_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr tmp(new robot_state::RobotState(body_model));
  body = tmp;

  // set body state
  link_names = body_model->getLinkModelNames();
  dof = 24;
  urdf::Model urdf_model = readUrdfFile(read_urdf_rosparam);
  total_mass = setModelMass(urdf_model);
  setLinksCoGVector(urdf_model);

  // initialize joint position
  std::string name[] = {
      "j_shoulder_l", "j_high_arm_l", "j_low_arm_l",  "j_wrist_l",
      "j_gripper_l",  "j_shoulder_r", "j_high_arm_r", "j_low_arm_r",
      "j_wrist_r",    "j_gripper_r",  "j_pan",        "j_tilt",
      "j_pelvis_l",   "j_thigh1_l",   "j_thigh2_l",   "j_tibia_l",
      "j_ankle1_l",   "j_ankle2_l",   "j_pelvis_r",   "j_thigh1_r",
      "j_thigh2_r",   "j_tibia_r",    "j_ankle1_r",   "j_ankle2_r"};
  double position[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  update(name, position);
}

RobotisBody::~RobotisBody() {}

urdf::Model RobotisBody::readUrdfFile(const std::string& param_name) {
  urdf::Model model;
  if (!model.initParam(param_name)) {
    ROS_WARN("[RobotisBody] Don't read URDF file");
  }
  return model;
}

// set links mass("link_masses") and model total mass
// return: model total mass
double RobotisBody::setModelMass(const urdf::Model& model) {
  double m = 0.0;
  auto links = model.links_;  // std::map <string, urdf::Link>

  for (std::string link_name : link_names) {
    if (link_name == "base_link" || link_name == "MP_PMDCAMBOARD") continue;

    link_masses[link_name] = links[link_name]->inertial->mass;
    m += links[link_name]->inertial->mass;
  }

  return m;
}

// set center of gravity from urdf model
// set to "link_cogs"
void RobotisBody::setLinksCoGVector(const urdf::Model& model) {
  auto links = model.links_;

  for (std::string link_name : link_names) {
    if (link_name == "base_link" || link_name == "MP_PMDCAMBOARD") continue;

    EVector3 v = EVector3::Zero();
    v << links[link_name]->inertial->origin.position.x,
        links[link_name]->inertial->origin.position.y,
        links[link_name]->inertial->origin.position.z;
    link_cogs[link_name] = v;
  }
}

EVector3 RobotisBody::calcCenterOfMass() {
  EVector3 mc = EVector3::Zero();

  for (std::string link_name : link_names) {
    EAffine3 j = link_trans[link_name];
    EVector3 c = j.translation() + j.rotation() * link_cogs[link_name];
    mc += link_masses[link_name] * c;
    // std::cout << link_name << j.matrix() << std::endl;
  }

  return (mc / total_mass);
}

// update body state. You must call this method in loop.
// Recommend call in joint_state subscriber
void RobotisBody::update(const std::string (&joint_name)[24], double position[]) {
  for (int i = 0; i < dof; i++) {
    body->setJointPositions(joint_name[i], &position[i]);
  }

  body->update();
  for (std::string link_name : link_names) {
    link_trans[link_name] = body->getGlobalLinkTransform(link_name);
  }
}

}  // namespace robotis

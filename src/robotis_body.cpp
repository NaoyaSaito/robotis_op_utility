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
  urdf::Model urdf_model = readUrdfFile(read_urdf_rosparam);
  total_mass = setModelMass(urdf_model);
  setLinksCoGVector(urdf_model);
}

RobotisBody::~RobotisBody() {}

urdf::Model RobotisBody::readUrdfFile(const std::string param_name) {
  urdf::Model model;
  if (!model.initParam(param_name)) {
    ROS_WARN("[RobotisBody] Don't read URDF file");
  }
  return model;
}

// set links mass("link_masses") and model total mass
// return: model total mass
double RobotisBody::setModelMass(urdf::Model model) {
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
void RobotisBody::setLinksCoGVector(urdf::Model model) {
  auto links = model.links_;

  for (std::string link_name : link_names) {
    if (link_name == "base_link" || link_name == "MP_PMDCAMBOARD") continue;

    Vector3 v = Vector3::Zero();
    v << links[link_name]->inertial->origin.position.x,
         links[link_name]->inertial->origin.position.y,
         links[link_name]->inertial->origin.position.z;
    link_cogs[link_name] = v;
  }
}

Vector3 RobotisBody::calcCenterOfMass() {
  Vector3 mc = Vector3::Zero();
  for (std::string link_name : link_names) {
    Affine3 j = body->getGlobalLinkTransform(link_name);
    Vector3 c = j.translation() + j.rotation()*link_cogs[link_name];
    mc += link_masses[link_name]*c;
  }
  Vector3 com = mc/total_mass;
  return com;
}

}  // namespace robotis

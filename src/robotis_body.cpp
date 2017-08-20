#include <robotis_op_utility/robotis_body.h>

#include <ros/ros.h>

namespace robotis {

RobotisBody::RobotisBody() {
  // load body model to "body"
  const std::string read_urdf_rosparam = "robot_description";
  robot_model_loader::RobotModelLoader robot_model_loader(read_urdf_rosparam);
  robot_model::RobotModelPtr body_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr tmp(new robot_state::RobotState(body_model));
  body = tmp;

  // set body state
  link_names = body_model->getLinkModelNames();

  auto joint_models = body_model->getActiveJointModels();
  for (auto joint : joint_models ) {joint_names.push_back(joint->getName());}
  dof = joint_names.size();

  for (auto jname : joint_names ) {
    std::string pub_name = "/robotis_op/"+jname+"_position_controller/command";
    joint_cmd_pubs[jname] = advertise<std_msgs::Float64>(pub_name, 50);
  }

  urdf::Model urdf_model = readUrdfFile(read_urdf_rosparam);
  total_mass = setModelMass(urdf_model);
  setLinksCoGVector(urdf_model);

  // set init joint position
  // angle get by "walker.py", self.ready_pos
  init_joint_angle["j_pelvis_l"] = 0.0;
  init_joint_angle["j_thigh1_l"] = 0.0;
  init_joint_angle["j_thigh2_l"] = 0.55;
  init_joint_angle["j_tibia_l"] = -1.1;
  init_joint_angle["j_ankle1_l"] = -0.55;
  init_joint_angle["j_ankle2_l"] = 0.0;
  init_joint_angle["j_pelvis_r"] = 0.0;
  init_joint_angle["j_thigh1_r"] = 0.0;
  init_joint_angle["j_thigh2_r"] = -0.55;
  init_joint_angle["j_tibia_r"] = 1.1;
  init_joint_angle["j_ankle1_r"] = 0.55;
  init_joint_angle["j_ankle2_r"] = 0.0;

  // initialize joint position
  sensor_msgs::JointState js;
  js.name = {"j_shoulder_l", "j_high_arm_l", "j_low_arm_l",  "j_wrist_l",
             "j_gripper_l",  "j_shoulder_r", "j_high_arm_r", "j_low_arm_r",
             "j_wrist_r",    "j_gripper_r",  "j_pan",        "j_tilt",
             "j_pelvis_l",   "j_thigh1_l",   "j_thigh2_l",   "j_tibia_l",
             "j_ankle1_l",   "j_ankle2_l",   "j_pelvis_r",   "j_thigh1_r",
             "j_thigh2_r",   "j_tibia_r",    "j_ankle1_r",   "j_ankle2_r"};
  js.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  update(js);
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
    EAffine3 j = link_affine[link_name];
    EVector3 c = j.translation() + j.rotation() * link_cogs[link_name];
    mc += link_masses[link_name] * c;
    // std::cout << link_name << j.matrix() << std::endl;
  }

  return (mc / total_mass);
}

// update body state. You must call this method in loop.
// Recommend call in joint_state subscriber
void RobotisBody::update(const sensor_msgs::JointState& msg) {
  for (int i = 0; i < dof; ++i) {
    body->setJointPositions(msg.name[i], &msg.position[i]);
  }

  body->update();
  for (std::string link_name : link_names) {
    link_affine[link_name] = body->getGlobalLinkTransform(link_name);
  }
}

void RobotisBody::moveInitPosition(const sensor_msgs::JointState& jsmsg) {
  // set now joint_state to "js"
  std::unordered_map<std::string, double> js;
  for (int i = 0; i < dof; ++i) {
    js[jsmsg.name[i]] = jsmsg.position[i];
  }

  // create joint angle track and publish
  const double freq = 200, dt = 1/freq;
  double delay = 2.0, roop_time = 0.0;
  ros::Rate rate(freq); // TODO! use param
  while (ros::ok() && roop_time <= delay) {
    double ratio = roop_time / delay;
    for (std::string jname : joint_names) {
      std_msgs::Float64 angle;
      angle.data = js[jname] * (1 - ratio) + init_joint_angle[jname] * ratio;
      joint_cmd_pubs[jname].publish(angle);
    }
    roop_time += dt;
    rate.sleep();
  }
  ROS_INFO("[RobotisBody] complete move init position");
}

}  // namespace robotis

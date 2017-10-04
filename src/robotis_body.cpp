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

  // set link names
  link_names = body_model->getLinkModelNames();

  // set joint names that can move
  auto joint_models = body_model->getActiveJointModels();
  for (auto joint : joint_models) {
    joint_names.push_back(joint->getName());
  }
  dof = joint_names.size();

  // prepare joint command publisher
  for (auto jname : joint_names) {
    std::string pub_name =
        "/robotis_op/" + jname + "_position_controller/command";
    joint_cmd_pubs[jname] = advertise<std_msgs::Float64>(pub_name, 50);
  }

  // set total_mass and all links CoG
  urdf::Model urdf_model = readUrdfFile(read_urdf_rosparam);
  total_mass = setModelMass(urdf_model);
  setLinksCoGVector(urdf_model);

  // set init joint position. use "moveInitPosition()"
  // angle gotten by "walker.py", self.ready_pos
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

  // setting to Inverce Kinematics of Leg
  right_leg_joint_group = body->getJointModelGroup("right_leg");
  left_leg_joint_group = body->getJointModelGroup("left_leg");
  const std::vector<std::string>& right_tmp =
      right_leg_joint_group->getVariableNames();
  for (std::string name : right_tmp) {
    right_leg_joint_names.push_back(name);
  }
  const std::vector<std::string>& left_tmp =
      left_leg_joint_group->getVariableNames();
  for (std::string name : left_tmp) {
    left_leg_joint_names.push_back(name);
  }

  // initialize joint position
  std::unordered_map<std::string, double> js;
  for (auto joint : joint_names) {
    js[joint] = 0.0;
  }
  update(js);
}

// calculate Center of Mass
// @return : now Center of Mass
Vector3 RobotisBody::calcCenterOfMass() {
  Vector3 mc = Vector3::Zero();

  for (std::string link_name : link_names) {
    Affine3 j = link_affine[link_name];
    Vector3 c = j.translation() + j.rotation() * link_cogs[link_name];
    mc += link_masses[link_name] * c;
  }

  return (mc / total_mass);
}

// update body state. You must call this method in loop.
// Recommend call in joint_state subscriber
void RobotisBody::update(const sensor_msgs::JointState& msgs) {
  std::unordered_map<std::string, double> values;
  for (int i = 0; i < dof; ++i) {
    values[msgs.name[i]] = msgs.position[i];
  }
  update(values);
}

void RobotisBody::update(const unordered_map<std::string, double>& msgs) {
  for (auto msg : msgs) {
    body->setJointPositions(msg.first, &msg.second);
  }
  body->update();
  for (std::string link_name : link_names) {
    link_affine[link_name] = body->getGlobalLinkTransform(link_name);
  }
}

// move initial joint position
// @param jsmsg : now joint state
void RobotisBody::moveInitPosition(const sensor_msgs::JointState& jsmsg) {
  // set now joint_state to "js"
  std::unordered_map<std::string, double> js;
  for (int i = 0; i < dof; ++i) {
    js[jsmsg.name[i]] = jsmsg.position[i];
  }

  // create joint angle track and publish
  const double freq = 200, dt = 1 / freq;
  double delay = 2.0, roop_time = 0.0;
  ros::Rate rate(freq);  // TODO! use param
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

// calculate Inverse Kinematics of walking motion
// @param com : next Center of Mass position (world coodinate)
// @param right_leg : next right leg position (world coodinate)
// @param left_leg : next left leg position (world coodinate)
// @param(return) joint_values : unordered_map<key:joint name, value:joint angle>
// @return : true: success IK, false: miss IK
bool RobotisBody::calcIKofWalkingMotion(
    const Vector3& com, const Affine3& right_leg, const Affine3& left_leg,
    std::unordered_map<std::string, double>& joint_values) {
  std::vector<double> right_joint_values, left_joint_values;

  // set translation amount
  Translation3 right_tr = Translation3(
    right_leg.translation().x() - com.x(),
    right_leg.translation().y() - com.y(),
    right_leg.translation().z());
  Translation3 left_tr = Translation3(
    left_leg.translation().x() - com.x(),
    left_leg.translation().y() - com.y(),
    left_leg.translation().z());

  // calc IK
  Affine3 right_end_state;
  right_end_state = right_tr * right_leg.rotation();
  bool found_right_ik =
      body->setFromIK(right_leg_joint_group, right_end_state, 50, 0.003);

  Affine3 left_end_state;
  left_end_state = left_tr * left_leg.rotation();
  bool found_left_ik =
      body->setFromIK(left_leg_joint_group, left_end_state, 50, 0.003);

  // if success IK, push joint value to "joint_values"
  if (found_right_ik) {
    body->copyJointGroupPositions(right_leg_joint_group, right_joint_values);
    for (std::size_t i = 0, n = right_leg_joint_names.size(); i < n; ++i) {
      joint_values[right_leg_joint_names[i]] = right_joint_values[i];
    }
  } else {
    ROS_WARN("[RobotisBody] Did not find IK solution right");
    return false;
  }
  if (found_left_ik) {
    body->copyJointGroupPositions(left_leg_joint_group, left_joint_values);
    for (std::size_t i = 0, n = left_leg_joint_names.size(); i < n; ++i) {
      joint_values[left_leg_joint_names[i]] = left_joint_values[i];
    }
  } else {
    ROS_WARN("[RobotisBody] Did not find IK solution left");
    return false;
  }
  return true;
}

// publish joint angle to ROBOTIS-OP2
// @param joint_values: unordered_map<key: joint name, value: joint angle>
void RobotisBody::publishJointCommand(
    std::unordered_map<std::string, double>& joint_values) {
  for (auto joint : joint_values) {
    std_msgs::Float64 angle;
    angle.data = joint.second;
    joint_cmd_pubs[joint.first].publish(angle);
  }
}

urdf::Model RobotisBody::readUrdfFile(const std::string& param_name) {
  urdf::Model model;
  if (!model.initParam(param_name)) {
    ROS_ERROR("[RobotisBody] Don't read URDF file");
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

    Vector3 v = Vector3::Zero();
    v << links[link_name]->inertial->origin.position.x,
        links[link_name]->inertial->origin.position.y,
        links[link_name]->inertial->origin.position.z;
    link_cogs[link_name] = v;
  }
}

}  // namespace robotis

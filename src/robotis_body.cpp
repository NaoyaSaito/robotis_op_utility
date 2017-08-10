#include <robotis_op_utility/robotis_body.h>

#include <ros/ros.h>

namespace robotis {

RobotisBody::RobotisBody(ros::NodeHandle nnh) {
  nh = nnh;

  // load body model to "body"
  std::string read_urdf_rosparam = "robot_description";
  robot_model_loader::RobotModelLoader robot_model_loader(read_urdf_rosparam);
  body = robot_model_loader.getModel();

  //
  link_names = body->getLinkModelNames();
  total_mass = setModelMass();
}

RobotisBody::~RobotisBody() {}

double RobotisBody::setModelMass() {
  double m = 0.0;

  for (std::string link_name : link_names) {
    double link_mass = 0.0;
    if (ros::param::get("/robot_description_model/link_mass/" + link_name,
                        link_mass)) {
      link_masses[link_name] = link_mass;
      m += link_mass;
    }
  }

  return m;
}

const Vector3& RobotisBody::calcCenterOfMass() {
  Vector3 v = Vector3::Zero();
  return v;
}

}  // namespace robotis

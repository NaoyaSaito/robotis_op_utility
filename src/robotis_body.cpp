#include <robotis_op_utility/robotis_body.h>

#include <ros/ros.h>


namespace robotis{

RobotisBody::RobotisBody() {
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  body = robot_model_loader.getModel();
}

const Vector3& RobotisBody::calcCenterOfMass(){
  Vector3 v = Vector3::Zero();
  return v;
}

}

#include <robotis_op_utility/robotis_body.h>

#include <gtest/gtest.h>

using namespace robotis;

// check total mass
TEST(RobotisBodyTest, ModelMass) {
  ros::NodeHandle nh;
  RobotisBody body(nh);
  EXPECT_NEAR(body.total_mass, 3.21961, 0.001);
}

TEST(RobotisBodyTest, CenterOfMass) {
  ros::NodeHandle nh;
  RobotisBody body(nh);
  Vector3 v = body.calcCenterOfMass();
  std::cout << "CoM : " << v(0) << ", " << v(1) << ", " << v(2) << std::endl;
  SUCCEED();  // CoM test is difficult....
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_robotis_body");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#include <robotis_op_utility/robotis_body.h>

#include <gtest/gtest.h>

using namespace robotis;

TEST(RobotisBodyTest, ModelMass) {
  ros::NodeHandle nh;
  RobotisBody body(nh);
  EXPECT_NEAR(body.total_mass, 3.129058, 0.000001);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_robotis_body");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class OdomPub {
 public:
  OdomPub(std::string arg) {
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    if (arg == "cmd_vel") {
      sub = nh.subscribe("/robotis_op/cmd_vel", 1000, &OdomPub::velCallback,
                         this);
    } else if (arg == "Pose2D") {
      sub = nh.subscribe("/robotis_op/Pose2D", 1000, &OdomPub::pose2dCallback,
                         this);
    } else {
      ROS_ERROR("[odom_pub] Choose robot order topic(cmd_vel or Pose2D).");
    }

    current_time = ros::Time::now();
    last_time = ros::Time::now();
  }

  void odom_publisher(double x, double y, double th, double vx, double vy,
                      double vth) {
    // ---- from Odom Tutorial ---- //
    // http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/Odom

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // publish the message
    odom_pub.publish(odom);

    last_time = current_time;
  }

  void pose2dCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    current_time = ros::Time::now();
    // robot initial position
    // init position is origin, init velocity is ZERO
    static double lx = 0.0, ly = 0.0, lth = 0.0, x = 0.0, y = 0.0, th = 0.0;
    double dt = (current_time - last_time).toSec();
    double nx = msg->x;
    double ny = msg->y;
    double nth = msg->theta;

    double vx = (nx - lx) / dt;
    double vy = (ny - ly) / dt;
    double vth = (nth - lth) / dt;

    x += nx;
    y += ny;
    th += nth;
    lx = nx;
    ly = ny;
    lth = nth;

    odom_publisher(x, y, th, vx, vy, vth);
  }

  void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // init robot position is origin
    static double x = 0.0, y = 0.0, th = 0.0;
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double vth = msg->angular.z;

    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    odom_publisher(x, y, th, vx, vy, vth);
  }

 private:
  ros::Publisher odom_pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robotis_op_odom_pub");

  if (argc != 2) {
    ROS_ERROR("[odom_pub] Choose robot order topic(cmd_vel or Pose2D).Finish.");
    return 0;
  }

  std::string arg = std::string(argv[1]);

  OdomPub odompub(arg);
  ros::spin();
}
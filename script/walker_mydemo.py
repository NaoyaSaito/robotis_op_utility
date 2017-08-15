#!/usr/bin/python
import rospy
from robotis_op_simulation_walking.darwin import Darwin


if __name__ == "__main__":
    rospy.init_node("walker_demo")

    rospy.loginfo("Instantiating Darwin Client")
    darwin = Darwin()
    rospy.sleep(1)

    rospy.loginfo("Darwin Walker Demo Starting")

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        darwin.set_walk_velocity(0.5, 0, 0)
        rate.sleep()

    rospy.loginfo("Darwin Walker Demo Finished")

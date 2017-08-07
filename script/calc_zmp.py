#!/usr/bin/python
import rospy
import csv
from geometry_msgs.msg import WrenchStamped


ws = [None, None]
zmp_stock = []


def rleg_ft_callback(rleg_ft):
    ws[0] = rleg_ft.wrench


def lleg_ft_callback(lleg_ft):
    ws[1] = lleg_ft.wrench
    time = lleg_ft.header.stamp.secs + (lleg_ft.header.stamp.nsecs/1e9)
    if ws[0] is not None and ws[1] is not None:
        calc_zmp(time)


def calc_zmp(time):
    zmp_sep = [[None, None], [None, None]]
    zmp = [None, None]
    fz = [ws[0].force.x, -ws[1].force.x]
    d = 0.02

    zmp_sep = [
        time,
        (-ws[0].torque.y - ws[0].force.z*d)/fz[0],  # Right leg X ZMP
        ( ws[0].torque.z - ws[0].force.y*d)/fz[0],  # Right leg Y ZMP
        ( ws[1].torque.y - ws[1].force.z*d)/fz[1],  # Left  leg X ZMP
        ( ws[1].torque.z + ws[1].force.y*d)/fz[1]   # Left  leg Y ZMP
    ]
    zmp = [
        (zmp_sep[1]*fz[0] + zmp_sep[2]*fz[1])/(fz[0]+fz[1]),  # X ZMP [m]
        (zmp_sep[3]*fz[0] + zmp_sep[4]*fz[1])/(fz[0]+fz[1]),  # Y ZMP [m]
        fz[0]+fz[1]  # Reactive Force (Z)  [kg]
    ]
    zmp_sep.append(zmp[0])
    zmp_sep.append(zmp[1])
    zmp_sep.append(zmp[2])
    zmp_stock.append(zmp_sep)


if __name__ == '__main__':
    rospy.init_node("calc_zmp_by_ft")
    rleg = rospy.Subscriber(
        "/robotis_op/rleg/wrench", WrenchStamped, lleg_ft_callback
    )
    lleg = rospy.Subscriber(
        "/robotis_op/lleg/wrench", WrenchStamped, rleg_ft_callback
    )

    rospy.spin()

    f = open("zmp.csv", "wb")
    writer = csv.writer(f)
    writer.writerow(
        ["time [s]", "r-ZMP(x)", "r-ZMP(y)", "l-ZMP(x)",
         "l-ZMP(y)", "ZMP(x)", "ZMP(y)", "Force"]
    )
    writer.writerows(zmp_stock)
    f.close()

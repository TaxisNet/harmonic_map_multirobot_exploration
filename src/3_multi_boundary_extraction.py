#!/usr/bin/env python3

from multi_boundary_extraction import *


if __name__=='__main__':

    rospy.init_node('boundary_comp_node', anonymous=True)
    robot_radius = 0.35
    robots = ['tb3_0', 'tb3_1', 'tb3_2']
    mc_list = []

    for robot in robots:
        
        other_robots = robots.copy()   
        other_robots.remove(robot)
        mc = MergingComputation(robot, other_robots)
        mc.robot_radius = robot_radius
        mc_list.append(mc)
        

    rate = rospy.Rate(1)

    while(not rospy.is_shutdown()):
        for mc in mc_list:
            mc.publish_data()
        rate.sleep()
#!/usr/bin/env python3
from multi_boundary_extraction import *



if __name__=='__main__':
    rospy.init_node('boundary_comp_node', anonymous=True)
    
    ns = rospy.get_namespace()[1:-1]
    no_of_robots = rospy.get_param('no_of_robots', 2)
    
    other_robots = ['amigo_{}'.format(i+1) for i in range(no_of_robots)]
    other_robots.remove(ns)

    print("robot: "+ns)
    print("other robot: "+other_robots.__str__())


    mergComp = MergingComputation(ns, other_robots)
    mergComp.robot_radius=(0.33/2)
    mergComp.tf_base_frame ='/base_link'
    mergComp.tf_robot_frame = mergComp.namespace + mergComp.tf_base_frame

    rate = rospy.Rate(0.25)

    while(not rospy.is_shutdown()):
        mergComp.publish_data()
    
        rate.sleep()
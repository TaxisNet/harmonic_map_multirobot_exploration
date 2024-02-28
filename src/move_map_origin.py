#!/usr/bin/env python3
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
class Transformer():
    def __init__(self):
        rospy.init_node('map_origin_node', anonymous=True)
        rospy.Subscriber('map_merge/map',OccupancyGrid, self.callback )
        self.pub = rospy.Publisher('map_merge/new_map',OccupancyGrid, queue_size=1)
        self.newMap = OccupancyGrid()
    
    def callback(self, msg):
        
        p = Pose()
        p.position.x = -20
        p.position.y = -20
        msg.info.origin = p
        self.newMap = msg
        self.pub.publish(self.newMap)


if __name__=='__main__':

    trans = Transformer()
    rospy.spin()
    # rate = rospy.Rate(0.1)
    # while not rospy.is_shutdown():
    #     trans.publishMap()
    #     rate.sleep()
    # rospy.spin()


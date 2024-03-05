#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header



class MapMerger():
    def __init__(self, robot_ns, other_robots, map_topic_name = "map", map_metadata_topic_name = "map_metadata"):

        self.robot_ns = robot_ns
        self.other_robots = other_robots
        self.maps = dict()
        
        self.map_metadata = None
        self.merged_map_frame = "world"

        #Get maps using subs
        self.map_subs = dict()
        self.my_map_sub = rospy.Subscriber((robot_ns+"/"+map_topic_name),OccupancyGrid,callback=self.map_callback, callback_args=robot_ns)
        for robot in other_robots:
            sub = rospy.Subscriber((robot+"/"+map_topic_name),OccupancyGrid,callback=self.map_callback, callback_args=robot)
            self.map_subs[robot] = sub
        

        #Publisher for merged map
        self.merged_map_pub = rospy.Publisher((robot_ns+"/merged_map"),OccupancyGrid, queue_size=1)
        
    def map_callback(self, msg, robot_name):
        map_data = np.reshape(msg.data, (msg.info.width, msg.info.height ), order='C')
        # plt.imshow(map_data, cmap='gray')
        # plt.show()
        self.maps[robot_name] = map_data
        
        if(robot_name == self.robot_ns):
            self.map_metadata = msg.info

    def generate_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.merged_map_frame
        return header
    
    def merge_maps(self):
        merged_map = np.full(self.maps[self.robot_ns].shape, -1, dtype=np.int8)

        for robot in self.other_robots:
            if robot in self.maps.keys():
                is_free = np.logical_and(self.maps[robot]>=0, self.maps[robot]<50)
                is_free = np.logical_and(merged_map<50, is_free)
                is_occ =  self.maps[robot]>=50

                merged_map[is_free] = 0
                merged_map[is_occ] = 100
        

        is_free = np.logical_and(self.maps[self.robot_ns]>=0, self.maps[self.robot_ns]<50)
        #is_free = np.logical_and(merged_map<50, is_free)    
        #self perseption is best
        is_occ =  self.maps[self.robot_ns]>=50

        merged_map[is_free] = 0
        merged_map[is_occ] = 100

        return merged_map


    def publish_merged_map(self):
        if(self.map_metadata is not None):
          
            msg = OccupancyGrid()
            msg.data = self.merge_maps().flatten(order='C')
            msg.info = self.map_metadata
            msg.header = self.generate_header()

            self.merged_map_pub.publish(msg)




if __name__=='__main__':
    rospy.init_node('my_map_merger', anonymous=True)
    robots = ['tb3_0', 'tb3_1', 'tb3_2']
    merger_list = []

    for robot in robots:
        
        other_robots = robots.copy()   
        other_robots.remove(robot)
        merger = MapMerger(robot, other_robots)
        merger_list.append(merger)
        

    rate = rospy.Rate(1)

    while(not rospy.is_shutdown()):
        for merger in merger_list:
            merger.publish_merged_map()
        rate.sleep()
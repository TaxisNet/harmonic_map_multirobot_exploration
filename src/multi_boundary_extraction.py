#!/usr/bin/env python3
from boundary_extraction import *

class MergingComputation(Computation):
    def __init__(self,ns,robot_list):
        self.other_robots_ns = robot_list

        self.merge_ns = '/map_merge'
        self.merged_map_topic = self.merge_ns+'/new_map'
        self.tf_merged_map_frame = 'world'
        self.merged_map_sub = rospy.Subscriber(self.merged_map_topic,OccupancyGrid,self.merged_msg_callback)

        super(MergingComputation, self).__init__(ns)

    def merged_msg_callback(self,msg):
        self.merged_map_data_tmp = msg.data
        self.mergedMapSize_py = [msg.info.width, msg.info.height]
        self.mergedMapResolution = msg.info.resolution
        self.mergedMapOrigin = [-int(msg.info.origin.position.x),-int(msg.info.origin.position.y)]
        try:
            self.checkMerge()
        except:
            print("Merge Check Failed!")


    def checkMerge(self):
        map_data = self.prefilter_map(self.merged_map_data_tmp, self.mergedMapSize_py)
        map_data[map_data>0] = 1 #occupied 
        map_data[map_data<0] = -1 # unoccupied
        #zero is unknown
        
        # DILATE IMAGE 
        bound = np.array(map_data>0, dtype=np.uint8)
        kernel = np.ones((2,2), np.uint8)
        img_dilate = cv2.dilate(bound, kernel, iterations=2)
        map_data[img_dilate==1]=1

        # ERODE IMAGE
        kernel = np.zeros((2,2), np.uint8)
        img_dilate = cv2.erode(np.uint8(map_data.copy()), kernel, iterations=1)

         # DILATE FREE
        img_dilatefree = cv2.dilate(np.uint8(map_data.copy()), 255*np.ones((2,2), np.uint8), iterations=1) #(2,2)
        map_data[np.logical_and(img_dilatefree==255,map_data == 0)]= -1


        # DILATE UNK
        # kernel = np.ones((5,5), np.uint8)
        bound = np.array(map_data ==0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound.copy(), kernel, iterations=1)
        map_data[np.logical_and(img_dilate==1,map_data<0)] = 0  

        #DILATE BOUNDARIES BY ROBOT RADIUS
        kernel = np.ones((self.robot_radius_in_cells, self.robot_radius_in_cells), dtype=np.uint8)
        bound = np.array(map_data>0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound, kernel)
        map_data[img_dilate==1]=1



        free = map_data.copy() < 0
        convMat = np.array([[0,1,0],[1,-4,1],[0,1,0]]) 
        bound = self.conv2(free,convMat,mode='same')
        
        obsBound = bound.copy() >0
        obsBound = np.logical_and(obsBound,(map_data>0))

        unknown = map_data.copy()==0
        bound = self.conv2(unknown,convMat,mode='same')
        freeBound = bound.copy() > 0
        freeBound = np.logical_and(freeBound.copy(),(map_data<0))

        boundary = np.zeros(map_data.shape)
        boundary[freeBound] = -1
        boundary[obsBound] = 1

        # Find the connected boundary components 
        bb = (boundary.copy() != 0 )
        bb = np.array(bb, dtype=np.uint8)

        #GET CONTOURS
        contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
        # GET ROBOT POSITION
        try:
            (trans,rot) =  self.listener.lookupTransform(self.tf_merged_map_frame, self.tf_robot_frame, rospy.Time(0))
            #pos in image pixels
            robot_position= np.double([trans[0]/self.mapResolution,trans[1]/self.mapResolution]) + np.double(self.mapOrigin)/self.mapResolution
            robot_position = np.round(robot_position).astype(int)
            #do not remove because it cv2 pointPolygonTest produces an error.
            robot_position = (int(robot_position[0]), int(robot_position[1]))
        except:
            print("Error getting position of {}".format(self.namespace))
        



        #find outer side of outer contour 
        for i, hierarchy_vec in enumerate(hierarchy[0]):
            if(hierarchy_vec[3]==-1):
                #check if 1st  robot is inside:
                if(cv2.pointPolygonTest(contours[i], robot_position, measureDist=False) >= 0):
                    outer_outer_bound_indx = i
                    break


        # now try to find inner side outer of outer contour
        if(hierarchy[0][outer_outer_bound_indx][2]== -1):
            #if no children outer outer will become outer
            outer_bound_indx = outer_outer_bound_indx
            
        else:
            cur_bound_indx = hierarchy[0][outer_outer_bound_indx][2]
            outer_bound_indx = cur_bound_indx
            
            # if mutiple inner outer level boundaries
            while(True):
                if(cv2.pointPolygonTest(contours[cur_bound_indx], robot_position, measureDist=False) >= 0):
                    outer_bound_indx = cur_bound_indx
                    break
                if(hierarchy[0][cur_bound_indx][0]== -1): break
                else: cur_bound_indx = hierarchy[0][cur_bound_indx][0]

        # GET OTHER ROBOT POSITION AND CHECK IF MERGED
        for robot_ns in self.other_robots_ns:
            try:
                (trans,rot) =  self.listener.lookupTransform(self.tf_merged_map_frame, robot_ns+self.tf_base_frame, rospy.Time(0))
                #pos in image pixels
                robot_pos = np.double([trans[0]/self.mapResolution,trans[1]/self.mapResolution]) + np.double(self.mapOrigin)/self.mapResolution
                robot_pos = np.round(robot_pos).astype(int)
                #do not remove because it cv2 pointPolygonTest produces an error.
                robot_pos = (int(robot_pos[0]), int(robot_pos[1]))
                if(cv2.pointPolygonTest(contours[outer_bound_indx], robot_pos, measureDist=False) >= 0):
                    self.updateToMerged()
                    break
            except:
                print("Error getting pos of {}!".format(robot_ns))
        
    def updateToMerged(self):
        print("Merged {}!".format(self.namespace))
        self.map_sub.unregister()
        self.merged_map_sub.unregister()

        self.tf_map_frame = self.tf_merged_map_frame
        self.map_sub = rospy.Subscriber(self.merged_map_topic,OccupancyGrid,self.map_msg_callback)



if __name__=='__main__':

    rospy.init_node('boundary_comp_node', anonymous=True)

    mc0 = MergingComputation('tb3_0', ['tb3_1'])
    mc1 = MergingComputation('tb3_1', ['tb3_0'])
    rate = rospy.Rate(0.25)

    while(not rospy.is_shutdown()):
        mc0.publish_data()
        mc1.publish_data()

        rate.sleep()
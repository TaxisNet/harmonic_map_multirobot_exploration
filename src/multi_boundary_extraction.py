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
        # prefilter_map
        # IMPLEMENTATION OF THE occGridMapping_py FUNCTION
        map_output = np.transpose(np.reshape(self.merged_map_data_tmp, (self.mergedMapSize_py[0],self.mergedMapSize_py[1]),order='F'))
        map_data = map_output.copy()
        map_data[map_output.copy() == -1] = 0
        map_data[ np.logical_and(map_output.copy() < 50,map_output.copy() != -1)] = self.lo_min
        map_data[map_output.copy() >= 50] =self.lo_max

        map_data_uint8 = np.uint8(map_data.copy())


        # INFLATE THE BOUNDARY
        kernel = np.ones((1,1), np.uint8) #kernel = np.ones((3,3), np.uint8)
        map_uint8_dilate = cv2.dilate(map_data_uint8, kernel, iterations=1)
        map_dilate = map_data.copy()
        map_dilate[map_uint8_dilate == np.max(map_uint8_dilate)] = self.lo_max
        map_data = map_dilate
        #map_data = self.prefilter_map(self.merged_map_data_tmp,self.mergedMapSize_py)

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
        # kernel = np.ones((self.robot_radius_in_cells, self.robot_radius_in_cells), dtype=np.uint8)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.robot_radius_in_cells, self.robot_radius_in_cells))    
        # bound = np.array(map_data>0, dtype=np.uint8)
        # img_dilate = cv2.dilate(bound, kernel)
        # map_data[img_dilate==1]=1
        
        
        # convMat = np.array([[0,1,0],[1,-4,1],[0,1,0]]) 
        # convMat = np.ones((3,3))
        # convMat[1,1] = -8
        # convMat = np.ones((4,4))
        # convMat[1:3, 1:3] = -3


        # merged map might be fuzzy and need a larger kernel for edge detection
        convMat = np.ones((5,5))
        convMat[2,2] = -24

        free = map_data.copy() < 0
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

        
        # plt.subplot(121)
        # plt.imshow(map_data)
        # plt.subplot(122)
        # plt.imshow(bb)
        # plt.show()

        #FINALLY FIND THE CONTOURS
        contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                
        # GET ROBOT POSITION
        try:
            (trans,rot) =  self.listener.lookupTransform(self.tf_merged_map_frame, self.tf_robot_frame, rospy.Time(0))
            #pos in image pixels
            robot_position= np.double([trans[0]/self.mergedMapResolution,trans[1]/self.mergedMapResolution]) + np.double(self.mergedMapOrigin)/self.mergedMapResolution
            robot_position = np.round(robot_position).astype(int)
            #do not remove because it cv2 pointPolygonTest produces an error.
            robot_position = (int(robot_position[0]), int(robot_position[1]))
        except:
            print("Error getting position of {}".format(self.namespace))
            return        
        
        
      
        
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
            (trans,rot) =  self.listener.lookupTransform(self.tf_merged_map_frame, robot_ns+self.tf_base_frame, rospy.Time(0))
            #pos in image pixels
            robot_pos = np.double([trans[0]/self.mergedMapResolution,trans[1]/self.mergedMapResolution]) + np.double(self.mergedMapOrigin)/self.mergedMapResolution
            robot_pos = np.round(robot_pos).astype(int)
            #do not remove because it cv2 pointPolygonTest produces an error.
            robot_pos = (int(robot_pos[0]), int(robot_pos[1]))
            if(cv2.pointPolygonTest(contours[outer_bound_indx], robot_pos, measureDist=False) >= 0):
                self.updateToMerged()
                break
            try:
                pass
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
    mc0.robot_radius = 0.05
    mc1 = MergingComputation('tb3_1', ['tb3_0'])
    mc1.robot_radius = 0.05
    rate = rospy.Rate(0.2)

    while(not rospy.is_shutdown()):
        mc0.publish_data()
        mc1.publish_data()

        rate.sleep()
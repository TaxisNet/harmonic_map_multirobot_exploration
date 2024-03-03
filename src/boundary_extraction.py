#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from math import ceil
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

import cv2
from scipy.ndimage import convolve
from tf import TransformListener
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Twist
import numpy as np
import tf

from boundary_compute.msg import boundary_info
from geometry_msgs.msg import PoseStamped


class Computation():
    def __init__(self, ns=''):
        # define ros namespace
        self.namespace =  ns
        self.tf_map_frame = self.namespace +'/map'
        self.tf_base_frame = '/base_footprint'
        self.tf_robot_frame = self.namespace +self.tf_base_frame 
        self.map_topic = self.namespace +'/map'

        #TF LISTENER
        self.listener = tf.TransformListener()
        

        # MAP SUBSCRIBER
        self.map_sub = rospy.Subscriber(self.map_topic,OccupancyGrid,self.map_msg_callback)
        self.map_size_x = None
        self.map_size_y = None
        self.map_data = None
        self.position = None

        self.mapSize_py = None
        self.mapResolution = None
        self.mapOrigin = None
        self.map_data_tmp = None

        ## OCG PARAMETERS
        self.lo_occ = 50
        self.lo_free = 0
        self.lo_max = 5000
        self.lo_min = -5000

        #Robot Params (from https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#data-of-turtlebot3-burger)
        self.robot_radius = 0.2 # in meters
        self.robot_radius_in_cells = None

        self.boundary_info_pub = rospy.Publisher(self.namespace +'/boundary_info', boundary_info, queue_size = 1)
        self.boundary_info_msg = boundary_info()

        self.image = None
        self.br = CvBridge()

        self.image_pub = rospy.Publisher(self.namespace+'/image_bou',Image,queue_size=1)
        self.image_msg = Image()

    def map_msg_callback(self,msg):
        self.map_data_tmp = msg.data
        self.mapSize_py = [msg.info.width, msg.info.height]
        self.mapResolution = msg.info.resolution
        self.mapOrigin = [-int(msg.info.origin.position.x),-int(msg.info.origin.position.y)]
        self.robot_radius_in_cells = ceil(self.robot_radius/msg.info.resolution)
        # self.robot_radius_in_cells+=1

    def conv2(self,x,y,mode='same'):

        if not(mode == 'same'):
            raise Exception("Mode not supported")

        # Add singleton dimensions
        if (len(x.shape) < len(y.shape)):
            dim = x.shape
            for i in range(len(x.shape),len(y.shape)):
                dim = (1,) + dim
            x = x.reshape(dim)
        elif (len(y.shape) < len(x.shape)):
            dim = y.shape
            for i in range(len(y.shape),len(x.shape)):
                dim = (1,) + dim
            y = y.reshape(dim)

        origin = ()

        # Apparently, the origin must be set in a special way to reproduce
        # the results of scipy.signal.convolve and Matlab
        for i in range(len(x.shape)):
            if ( (x.shape[i] - y.shape[i]) % 2 == 0 and
                x.shape[i] > 1 and
                y.shape[i] > 1):
                origin = origin + (-1,)
            else:
                origin = origin + (0,)

        z = convolve(x,y, mode='constant', origin=origin)

        return z    
    
    def boundaryExtraction(self, map_data):
        # Uses the robot's position and cv.findContours to locate the map contours
        
        
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
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.robot_radius_in_cells, self.robot_radius_in_cells))
        bound = np.array(map_data>0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound, kernel)
        map_data[img_dilate==1]=1


       
        #convMat = np.array([[0,1,0],[1,-4,1],[0,1,0]]) 
        # merged map might be fuzzy and need a larger kernel for edge detection
        convMat = np.ones((5,5))
        convMat[2,2] = -24
        
        free = map_data.copy() < 0
        # bound = self.conv2(free,convMat,mode='same')
        bound= cv2.Canny(np.array(map_data<0, dtype=np.uint8),0,1)
        b1 = bound
        obsBound = bound.copy() >0
        #obsBound = np.logical_and(obsBound,(map_data>0))

        unknown = map_data.copy()==0
        #bound = self.conv2(unknown,convMat,mode='same')
        bound= cv2.Canny(np.array(map_data==0, dtype=np.uint8),0,1)
        b2 = bound
        freeBound = bound.copy() > 0
        #freeBound = np.logical_and(freeBound.copy(),(map_data<0))

        boundary = np.zeros(map_data.shape)
        boundary[freeBound] = -1
        boundary[obsBound] = 1

        # Find the connected boundary components 
        bb = (boundary.copy() != 0 )
        bb = np.logical_or(map_data==1, obsBound)
        bb = np.array(bb, dtype=np.uint8)

        #close any holes
        bb = cv2.morphologyEx(bb, cv2.MORPH_CLOSE, np.ones((5,5)))


        # #debug plots
        # plt.subplot(221)
        # plt.imshow(map_data)
        # plt.subplot(222)
        # plt.imshow(bb)
        
        # plt.subplot(223)
        # plt.imshow(obsBound)
        # plt.subplot(224)
        # plt.imshow(freeBound)
        # plt.show()

        # plt.subplot(325)
        # plt.imshow(b1)
        # plt.subplot(326)
        # plt.imshow(b2)
        # plt.show()



        #FINALLY FIND THE CONTOURS
        contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        

        #find outer side of outer contour
        for i, hierarchy_vec in enumerate(hierarchy[0]):
            if(hierarchy_vec[3]==-1):
                #check if robot is inside:                
                if(cv2.pointPolygonTest(contours[i], self.robot_position, measureDist=False) >= 0):
                    outer_outer_bound_indx = i
                    break

        #now try to find inner side outer of outer contour
        if(hierarchy[0][outer_outer_bound_indx][2]== -1):
            #if no children outer outer will become outer
            outer_bound_indx = outer_outer_bound_indx
            
        else:
            cur_bound_indx = hierarchy[0][outer_outer_bound_indx][2]
            outer_bound_indx = cur_bound_indx
            
            #if mutiple inner outer level boundaries
            while(True):
                if(cv2.pointPolygonTest(contours[cur_bound_indx], self.robot_position, measureDist=False) >= 0):
                    outer_bound_indx = cur_bound_indx
                    break
                if(hierarchy[0][cur_bound_indx][0]== -1): break
                else: cur_bound_indx = hierarchy[0][cur_bound_indx][0]

        #append outer boundary to msg
        tmpout = np.array(contours[outer_bound_indx].copy()).reshape(contours[outer_bound_indx].shape[0],contours[outer_bound_indx].shape[2])
        xl = np.ascontiguousarray(np.flipud(tmpout[:,0]))
        yl = np.ascontiguousarray(np.flipud(tmpout[:,1]))
        nl = np.ascontiguousarray(np.size(xl))  
        nb = 1

        in_l = list()
        #get inner obstacles
        inner_bou_indx = hierarchy[0][outer_bound_indx][2]
        while True:
            if(inner_bou_indx!=-1):
                #append inner boundary to msg
                tmp = np.array(contours[inner_bou_indx].copy().reshape(contours[inner_bou_indx].shape[0],contours[inner_bou_indx].shape[2]))
                xl = np.append(xl,tmp[:,0])
                yl = np.append(yl,tmp[:,1])
                nl = np.append(nl,np.size(xl))
                nb += 1

                in_l.append(inner_bou_indx)

                inner_bou_indx = hierarchy[0][inner_bou_indx][0]
            else: break


        #check is free
        frontiers = np.logical_and(freeBound,obsBound)
        #frontiers = cv2.dilate(np.array(frontiers, dtype=np.uint8),np.ones((2,2)))   
        isFree = np.full(np.shape(xl),False)
        isFree[freeBound[yl,xl]] = True
        
        # # debug
        # print([outer_outer_bound_indx, outer_bound_indx, in_l])
        # print(hierarchy)
        # print("--------------\n")
    

        # ##########
        # self.robot_position= np.double([1/self.mapResolution,2/self.mapResolution]) + np.double(self.mapOrigin)/self.mapResolution
        # self.robot_position = np.round(self.robot_position).astype(int)
        # bou = boundary.copy()
        # x = self.robot_position[0]
        # y = self.robot_position[1]
        # print("{} {}".format(x,y))
        # bou[x,y] = 2
        # plt.imshow(bou, cmap='gray')
        # plt.show()
        # ###############

                

        bbcopy = np.array(255*np.ones((np.shape(map_data.copy())[0],np.shape(map_data.copy())[1],3)),dtype=np.uint8)
        bbcopy[freeBound,0] = 0
        bbcopy[freeBound,2] = 0
        bbcopy[obsBound,0] = 0
        bbcopy[obsBound,1] = 0
        # bbcopy[int(position[1])-1:int(position[1])+1,int(position[0])-1:int(position[0])+1,:] = 0
        
        #pub map image
        #used for debug only!
        self.image_msg =  self.br.cv2_to_imgmsg(bbcopy,"bgr8")
        self.image_pub.publish(self.image_msg)
        
        # # plots for debugging
        # plt.subplot(231,title="freeBound")
        # plt.imshow(freeBound)
        # plt.subplot(232, title="obsBound")
        # plt.imshow(obsBound)
        # plt.subplot(233,title="booundary")
        # plt.imshow(boundary)
        # plt.subplot(212)
        # plt.plot(xl,yl, 'b-')
        # plt.plot(xl[boundary_out[yl,xl] < 0],yl[boundary_out[yl,xl] < 0],'g*') 
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()

        return xl,yl,nb,nl,isFree
    
    def get_contours_by_hierarchy(self, contours, hierarchy ):
        #!!!!!!!!do not use!!!!!!


        # Organize contours by hierarchy level
        organized_contours = []

        # Create a dictionary to hold the hierarchy levels and corresponding contours
        hierarchy_dict = {}

        for i, contour in enumerate(contours):
            level = hierarchy[0][i][3]  # Get the hierarchy level of the contour
            if level not in hierarchy_dict:
                hierarchy_dict[level] = []
            hierarchy_dict[level].append(i) 

        # Sort contours within each hierarchy level
        for level in sorted(hierarchy_dict.keys()):
            organized_contours.append(hierarchy_dict[level])

        return organized_contours
    

    def prefilter_map(self, map_data, map_size):
        #GET ROBOT POSITION
        try:
            (trans,rot) =  self.listener.lookupTransform(self.tf_map_frame, self.tf_robot_frame, rospy.Time(0))
            #pos in image pixels
            self.robot_position= np.double([trans[0]/self.mapResolution,trans[1]/self.mapResolution]) + np.double(self.mapOrigin)/self.mapResolution
            self.robot_position = np.round(self.robot_position).astype(int)
            #do not remove because it cv2 pointPolygonTest produces an error.
            self.robot_position = (int(self.robot_position[0]), int(self.robot_position[1]))
           
        except:
            print("Error: Cannot get {}'s positions".format(self.namespace))
            return

        # IMPLEMENTATION OF THE occGridMapping_py FUNCTION
        map_output = np.transpose(np.reshape(map_data, (map_size[0],map_size[1]),order='F'))
        map_data = map_output.copy()
        
        map_data[map_output.copy() == -1] = 0
        map_data[ np.logical_and(map_output.copy() < 50,map_output.copy() != -1)] = self.lo_min
        map_data[map_output.copy() >= 50] =self.lo_max
        

        #space occupied by self is free
        # plt.subplot(1,2,1)
        # plt.imshow(map_data)
        # rows, cols = np.indices(map_data.shape)
        # distances = np.sqrt((rows - self.robot_position[1])**2 + (cols - self.robot_position[0])**2)
        # mask = distances <= self.robot_radius_in_cells
        # map_data[mask] = self.lo_min
        # plt.subplot(1,2,2)
        # plt.imshow(map_data)
        # plt.show()
        
        
        
        map_data_uint8 = np.uint8(map_data.copy())
       # INFLATE THE BOUNDARY
        kernel = np.ones((1,1), np.uint8) #kernel = np.ones((3,3), np.uint8)
        map_uint8_dilate = cv2.dilate(map_data_uint8, kernel, iterations=1)
        map_dilate = map_data.copy()
        map_dilate[map_uint8_dilate == np.max(map_uint8_dilate)] = self.lo_max
        

        
        return map_dilate



    def publish_data(self):
        failed_comp = False
        # CHECK IF MAP HAS BEEN RECEIVED
        if (not self.map_data_tmp == None):
            prefiltered_map  = self.prefilter_map(self.map_data_tmp,self.mapSize_py)
           
            # TRY COMPUTATION
            # xl_py, yl_py,nb_py,nl_py,boundary_py = self.boundaryExtraction(prefiltered_map)
            try:
                xl_py, yl_py,nb_py,nl_py,is_bou_free = self.boundaryExtraction(prefiltered_map)    
                failed_comp = False
            except:
                print("boundary extraction exception")
                failed_comp = True

            # PARTITION THE BOUNDARY TO FREE AND OCC
            if not failed_comp:
                self.boundary_info_msg.xl = xl_py
                self.boundary_info_msg.yl = yl_py
                self.boundary_info_msg.boundary_index = nl_py
                self.boundary_info_msg.isfree = is_bou_free
                # self.boundary_info_msg.pos_x = self.position[0]
                # self.boundary_info_msg.pos_y = self.position[1]
                self.boundary_info_msg.map_x0 = self.mapOrigin[0]
                self.boundary_info_msg.map_y0 = self.mapOrigin[1]
                self.boundary_info_msg.map_width = self.mapSize_py[0]
                self.boundary_info_msg.map_height = self.mapSize_py[1]
                self.boundary_info_msg.map_resolution = self.mapResolution
                self.boundary_info_msg.comp_failed = failed_comp
                self.boundary_info_pub.publish(self.boundary_info_msg)
               
                #print("Boundary Info published")
            else: 
                print('Computation Failed')
                self.boundary_info_msg.comp_failed = failed_comp
                self.boundary_info_msg.map_resolution = self.mapResolution
                self.boundary_info_msg.map_x0 = self.mapOrigin[0]
                self.boundary_info_msg.map_y0 = self.mapOrigin[1]
                self.boundary_info_msg.map_width = self.mapSize_py[0]
                self.boundary_info_msg.map_height = self.mapSize_py[1]
                self.boundary_info_pub.publish(self.boundary_info_msg)
                
            
            return failed_comp
        else:
            return False
            


if __name__=='__main__':
    rospy.init_node('boundary_comp_node', anonymous=True)
    ns = rospy.get_namespace()[:-1]
    computation = Computation(ns)
    computation.robot_radius=(0.25/2)
    rate = rospy.Rate(0.2)
    
    while not rospy.is_shutdown():
        #about 0.3 to 0.7 secs
        computation.publish_data()
        
        rate.sleep()


# if __name__=='__main__':
#     rospy.init_node('boundary_comp_node', anonymous=True)

#     ns = rospy.get_namespace()[:-1]
#     computation = Computation(ns)
#     computation.robot_radius=(0.56/2)
#     computation.tf_robot_frame = computation.namespace +'/base_link'
#     rate = rospy.Rate(1)
    
#     while not rospy.is_shutdown():
#         computation.publish_data()
        
#         rate.sleep()


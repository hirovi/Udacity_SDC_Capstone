#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml


from scipy.spatial import KDTree

import math


STATE_COUNT_THRESHOLD = 3

#used to switch traffic light ground truth or state from classifier
TL_CLASSIFFIER_ON =0


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        #sub7 = rospy.Subscriber('/image_raw', Image, self.image_cb)
        
                
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        #for KD TREE used in get_closest_point method
        self.waypoints_2d = None
        self.waypoint_tree = None      
        
        self.img_count=0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        
        
        #setup KDTREE of waypoints
        #self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree  =  KDTree(self.waypoints_2d)        
        
        

    def traffic_cb(self, msg):
        self.lights = msg.lights



    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state , dist_stop_line= self.process_traffic_lights()
        
        
        rospy.logwarn("dist to stop line is:{}".format(dist_stop_line))        
        
        rospy.logwarn("light state is:{}".format(state))
        
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        '''
        ########################################
        #This section is setup to capture images for traning
        #from simulator
        #RED=0,YELLOW=1,GREEN=2,UNKNOWN=3
        
        cv_image=self.bridge.imgmsg_to_cv2(self.camera_image,"bgr8")
        if dist_stop_line < 100:
            if state==0:
                img_file='img_{}_RED.png'.format(self.img_count)
            elif state==1:
                img_file='img_{}_YELLOW.png'.format(self.img_count)
            elif state==2:
                img_file='img_{}_GREEN.png'.format(self.img_count)
            else:
                img_file='img_{}_UX.png'.format(self.img_count)

        else:
            img_file='img_{}_UX.png'.format(self.img_count)
            
        dir_to_save_img='/home/atpandey/Group_Project/Udacity_SDC_Capstone/ros/training_images/'
        save_file=dir_to_save_img+img_file
        cv2.imwrite(save_file,cv_image)
        
        rospy.logwarn("img_file_name is:{}".format(save_file))
        
        self.img_count += 1

        ####################################3        
        
        '''
        
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    #def get_closest_waypoint(self, pose):
    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        #return 0
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return  closest_idx       
        
        
        

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        #if data available from classifier use it else use ground truth from simulator
        if TL_CLASSIFFIER_ON:        
            if(not self.has_image):
                self.prev_light_loc = None
                return False

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            #Get classification
            return self.light_classifier.get_classification(cv_image)
        
        else :
            return light.state
        
        
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #light = None
        #return values
        #closest light index
        closest_light = None
        
        #closest stop_line waypoint idx
        line_wp_idx= None        
        
        #closest stop line position
        ret_stop_line_position=None
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)

            # find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i,light in enumerate(self.lights):
                #Get stop line waypoint idx
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0],line[1])
                d= temp_wp_idx-car_position
                #find closest stop line waypoint index
                if d >=0 and d < diff:
                        diff=d
                        closest_light = light
                        line_wp_idx= temp_wp_idx
                        #ret_stop_line_position=line
                        ret_stop_line_position=math.sqrt((self.pose.pose.position.x-line[0])**2+(self.pose.pose.position.y-line[1])**2)
                
            
            
            
            
        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state,ret_stop_line_position
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN,None

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

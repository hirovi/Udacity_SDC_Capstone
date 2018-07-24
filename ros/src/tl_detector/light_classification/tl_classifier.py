
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#from utils import visualization_utils as vis_util
#from utils import label_map_util

#from matplotlib import  pyplot as plt
#from matplotlib import  image as mpimg
####using object detection example in TF API

import tensorflow as tf
import numpy as np

from PIL import Image
import datetime

import rospy
import yaml
import time



from styx_msgs.msg import TrafficLight


def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)



#save_img_to_visualize=1
save_img_to_visualize=0
NUM_CLASSES=4
verbose=1

class TLClassifier(object):
    def __init__(self):
        #load classifier
        #Load a frozen graph
    
        self.image_counter=0    
    
        get_conf=rospy.get_param("/traffic_light_config")
        self.conf=yaml.load(get_conf)
        
        
        
        self.PATH_TO_CKPT =r"{}".format(self.conf["model"])
        self.PATH_TO_LABEL=r"{}".format(self.conf["label"])
        
        
#        self.label_map=label_map_util.load_labelmap(self.PATH_TO_LABEL)
        
#        self.categories=label_map_util.convert_label_map_to_categories(self.label_map,max_num_classes=NUM_CLASSES,use_display_name=True)
#        self.category_index=label_map_util.create_category_index(self.categories)
        
        
        #threshold of detection score for trafffic light state selection
        self.threshold_l=0.5
        
        
        #TF graph and session related operations
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            try:
                with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')
                    rospy.loginfo("TL Classifier using: {} trained model".format(self.PATH_TO_CKPT))
            except EnvironmentError:
                rospy.logfatal("TL Classifier can't load model: {}".format(self.PATH_TO_CKPT))
            
            
            
            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
            
            
        self.sess=tf.Session(graph=self.detection_graph)
        

	
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # implement light color prediction
        #start of inference
        start_time=datetime.datetime.now()  
        with self.detection_graph.as_default():
            
            #with tf.Session(graph=self.detection_graph) as sess:
            # the array based representation of the image will be used later in order to prepare the
            # result image with boxes and labels on it.
            #image_np = load_image_into_numpy_array(image)
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image, axis=0)
            # Actual detection.
            (boxes, scores, classes, num) = self.sess.run(
             		 [self.detection_boxes, self.detection_scores, 
                       self.detection_classes, self.num_detections],
              		 feed_dict={self.image_tensor: image_np_expanded})
            
            '''
            #tis part was used to generated inferred images from 
            #simulator
            #to verify inference works
            if save_img_to_visualize:
                vis_util.visualize_boxes_and_labels_on_image_array(
                image,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=8
                )
                
                img_name_to_save= str(self.image_counter)+".png" 
                im_2_save=image.copy()
                
                mpimg.imsave(img_name_to_save,image)
                
                self.image_counter +=1
            '''                
        
	
    	
     
     
     
        
        boxes=np.squeeze(boxes)
        scores=np.squeeze(scores)
        classes=np.squeeze(classes)
        #num=np.squeeze(num)
	
	
        #rospy.loginfo("In TL detected {} TLs".format(num))
        '''
        #disable prints        
        if verbose:
            rospy.loginfo("highest score: {}, mostprobable class: {}".format(scores[0],classes[0]))
        '''
        state=TrafficLight.UNKNOWN
        #class map from label_map.pbtxt file
        #check if score[0] is greater than threshold and if yes then check class[0]
        #the scores/classes are sorted so looking at first detection[0] and deciding should be okay
        if scores[0] > self.threshold_l:
            if classes[0]==1:
                state=TrafficLight.GREEN
                #rospy.loginfo("In TL: {} detected".format('GREEN'))
            elif classes[0]==2:
                state=TrafficLight.RED
                #rospy.loginfo("In TL: {} detected".format('RED'))
            elif classes[0]==3:
                state=TrafficLight.YELLOW
                #rospy.loginfo("In TL: {} detected".format('YELLOW'))	
            else:
                state=TrafficLight.UNKNOWN
        else:
            state=TrafficLight.UNKNOWN
        #rospy.loginfo("In TL: {} detected".format(state))
        #return TrafficLight.UNKNOWN
        #end of inference
        
        '''
        #disable prints        
        if verbose:
            end_time=datetime.datetime.now()
            time_diff=end_time-start_time
            rospy.loginfo("time taken for inference is:{} seconds ".format(time_diff.seconds))
        '''
        return state

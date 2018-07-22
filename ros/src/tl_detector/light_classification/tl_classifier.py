from styx_msgs.msg import TrafficLight
import os
import numpy as np
import tensorflow as tf
from PIL import Image
import cv2
import lib.label_map_util


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        cur_path = os.path.dirname(os.path.abspath(__file__))
        model_path = cur_path + "/models/ssd_mobilenet_v1_coco_2017_11_17"
        self.PATH_TO_CKPT = model_path + '/frozen_inference_graph.pb'
        self.PATH_TO_LABELS = model_path + '/mscoco_label_map.pbtxt'
        self.NUM_CLASSES = 90

        self.traffic_class_id = 10

        self.load_labels()

        self.threshold=0.5
        
        # load model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        #image = Image.open(os.path.dirname(os.path.abspath(__file__))+"/red-light.jpg")
        # green light simulator
        #image = Image.open(os.path.dirname(os.path.abspath(__file__))+"/tf1.jpg")
        #image = Image.open(os.path.dirname(os.path.abspath(__file__))+"/Darth-Vader.jpg")
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        #self.image_np = self.load_image_into_numpy_array(image)
        self.image_np = image

        # Actual detection.
        output_dict = self.run_inference_for_single_image(self.image_np, self.detection_graph)
        
        height, width, _ = self.image_np.shape

        filtered_results = []
        for i in range(0, output_dict['num_detections']):
            score = output_dict['detection_scores'][i]
            # traffic light with good score
            if score >= self.threshold and self.traffic_class_id == output_dict['detection_classes'][i]:
                y1, x1, y2, x2 = output_dict['detection_boxes'][i]
                y1_o = int(y1 * height)
                x1_o = int(x1 * width)
                y2_o = int(y2 * height)
                x2_o = int(x2 * width)
                # predicted_class = self.category_index[output_dict['detection_classes'][i]]['name']
                # filtered_results.append({
                #         "score": score,
                #         "bb": output_dict['detection_boxes'][i],
                #         "bb_o": [y1_o, x1_o, y2_o, x2_o],
                #         "img_size": [height, width],
                #         "class": predicted_class,
                #         "class_id": output_dict['detection_classes'][i]
                #     })
                # print('[INFO] %s: %s' % (predicted_class, score))

                if self.is_light_red_or_yellow( y1_o, x1_o, y2_o, x2_o):
                    return TrafficLight.RED

        return TrafficLight.UNKNOWN

    # returns true if it finds a green light
    def is_light_red_or_yellow(self, y1, x1, y2, x2):
        #cv2.rectangle(self.image_np,(x1,y1),(x2,y2),(0,255,0),3)
        # crop
        crop_img = self.image_np[y1:y2, x1:x2]
        # convert from RGB to BGR Opencv format
        bgr = crop_img[...,::-1]
        # convert from BGR to HSV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # green HSV 0-180 range
        color = 60 # green
        sensitivity = 20
        lower_color = np.array([color - sensitivity, 100, 100]) 
        upper_color = np.array([color + sensitivity, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        output = cv2.bitwise_and(hsv, hsv, mask = mask)
        gray = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)

        # blur image
        blur = cv2.blur(gray,(5,5))

        #detect circle
        circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1.2,100,
                            param1=50,param2=30,minRadius=3,maxRadius=0)

        # if we found circles the traffic light is green
        if circles is not None:
            return False
        else:
            return True

        #cv2.imwrite('/tmp/tt1.jpg',blur)

    def run_inference_for_single_image(self, image, graph):
        with graph.as_default():
            with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                    'num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes', 'detection_masks'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                            tensor_name)

                # if 'detection_masks' in tensor_dict:
                #     # The following processing is only for single image
                #     detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                #     detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                #     # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                #     real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                #     detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                #     detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                #     detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                #         detection_masks, detection_boxes, image.shape[0], image.shape[1])
                #     detection_masks_reframed = tf.cast(
                #         tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                #     # Follow the convention by adding back the batch dimension
                #     tensor_dict['detection_masks'] = tf.expand_dims(
                #         detection_masks_reframed, 0)
                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

                # Run inference
                output_dict = sess.run(tensor_dict,
                                 feed_dict={image_tensor: np.expand_dims(image, 0)})

                # all outputs are float32 numpy arrays, so convert types as appropriate
                output_dict['num_detections'] = int(output_dict['num_detections'][0])
                output_dict['detection_classes'] = output_dict[
                    'detection_classes'][0].astype(np.uint8)
                output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                output_dict['detection_scores'] = output_dict['detection_scores'][0]
                
                if 'detection_masks' in output_dict:
                    output_dict['detection_masks'] = output_dict['detection_masks'][0]
        return output_dict

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

    def load_labels(self):
        self.label_map = lib.label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = lib.label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = lib.label_map_util.create_category_index(self.categories)
        #print(self.category_index)
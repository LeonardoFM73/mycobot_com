import rclpy
import numpy as np
import os
from rclpy.node import Node
from std_msgs.msg import String
import time
import pyrealsense2 as rs
import cv2
from tensorflow.lite.python.interpreter import Interpreter
from geometry_msgs.msg import Point


frame_rate_calc = 1

cond = False
show_image = True
MODEL_NAME = 'model'
SOURCE = 'src'
FOLDER_NAME = 'mycobot_com'
GRAPH_NAME = 'detect.tflite'
LABELMAP_NAME = 'labelmap.txt'
min_conf_threshold = float(0.95)
imW, imH = int(640), int(480)

class Sender(Node):
    def __init__(self):
        super().__init__('Camera')

        self.pub_camera = self.create_publisher(
            msg_type=Point,
            topic='camera',
            qos_profile=10
        )

        timer_period = 0.01
        self.obj_boxes = []
        self.obj_classes = []
        self.obj_centers = []
        self.obj_contours = []
        self.camera_coordinate = []
        self.coordinates = None
        


        try:
            self.get_logger().info("INTEL Loading")
            self.pc = rs.pointcloud()
            self.points = rs.points()    
            self.pipe = rs.pipeline()
            
            self.cfg  = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            self.pipe_profile=self.pipe.start(self.cfg)
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            self.create_timer(timer_period, self.timer_callback)
            
        except Exception as e:
            print(e)
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED")

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        self.camera_coordinate = None

        self.intric = depth_frame.profile.as_video_stream_profile().intrinsics
        

        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None
        


        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)


        # Create colormap to show the depth of the Objects
        colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        # Convert images to numpy arrays
        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        bgr_frame= self.detect(color_image,depth_image)
        # print(self.camera_coordinate)
        if self.camera_coordinate:
            # Create a Point message and publish it
            point_msg = Point()
            point_msg.x = float(self.camera_coordinate[0])
            point_msg.y = float(self.camera_coordinate[1])
            point_msg.z = float(self.camera_coordinate[2])
            self.pub_camera.publish(point_msg)
            self.camera_coordinate=[]
        

        # Display depth image
        # cv2.imshow('Depth Image', depth_image)
        
        # Display color image
        cv2.imshow('BGR Image', bgr_frame)
        cv2.imshow('BGR Image', bgr_frame)

        cv2.waitKey(1)  # Wait for a key press to update the display


    def detect(self,images,depth):
        # Get path to current working directory
        CWD_PATH = os.getcwd()

        # Path to .tflite file, which contains the model that is used for object detection
        PATH_TO_CKPT = os.path.join(CWD_PATH,SOURCE,FOLDER_NAME,FOLDER_NAME,MODEL_NAME,GRAPH_NAME)

        # Path to label map file
        PATH_TO_LABELS = os.path.join(CWD_PATH,SOURCE,FOLDER_NAME,FOLDER_NAME,MODEL_NAME,LABELMAP_NAME)

        # Load the label map
        with open(PATH_TO_LABELS, 'r') as f:
            labels = [line.strip() for line in f.readlines()]

        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if labels[0] == '???':
            del(labels[0])

        interpreter = Interpreter(model_path=PATH_TO_CKPT)

        interpreter.allocate_tensors()

        # Get model details
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        floating_model = (input_details[0]['dtype'] == np.float32)

        input_mean = 127.5
        input_std = 127.5

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            boxes_idx, classes_idx, scores_idx = 1, 3, 0
        else: # This is a TF1 model
            boxes_idx, classes_idx, scores_idx = 0, 1, 2


        # Grab frame from video stream
        frame1 = images

        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

        depth_sensor = self.pipe_profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
                
                cx = (xmin + xmax) // 2
                cy = (ymin + ymax) // 2
                depth_mm = depth[cy, cx]
                self.camera_coordinate = rs.rs2_deproject_pixel_to_point(self.intric, [cx, cy], depth_mm*depth_scale*1000)
                print(self.camera_coordinate)
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # Draw label
                object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                cv2.putText(frame, "{} cm".format(depth_mm / 10), (xmin + 5, ymin + 60), 0, 1.0, (255, 255, 255), 2)
                # cv2.putText(frame, f"Dis: {depth_mm}", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0))
                # cv2.putText(frame, f"X: {self.camera_coordinate[0]:.2f}", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255))
                # cv2.putText(frame, f"Y: {self.camera_coordinate[1]:.2f}", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255))
                # cv2.putText(frame, f"Z: {self.camera_coordinate[2]:.2f}", (80, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255))

            return frame
            
def main(args=None):
    rclpy.init(args=args)
    sender = Sender()
    rclpy.spin(sender)

    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Camera Node for Shelf Inventory Vision System
This node processes camera feeds and detects products on shelves
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import os
import yaml
from threading import Lock

# Import custom message types
from siwvs_msgs.msg import ProductDetection

class CameraNode(Node):
    """ROS 2 node for camera input processing and product detection."""
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('label_path', '')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('use_gpu', True)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.model_path = self.get_parameter('model_path').value
        self.label_path = self.get_parameter('label_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.use_gpu = self.get_parameter('use_gpu').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load the TensorFlow model
        self.load_model()
        
        # Load product labels
        self.load_labels()
        
        # Initialize mutex for thread safety
        self.lock = Lock()
        
        # Create subscribers
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Create publishers
        self.detection_pub = self.create_publisher(
            ProductDetection,
            '/siwvs/product_detection',
            10
        )
        
        self.processed_image_pub = self.create_publisher(
            Image,
            '/siwvs/processed_image',
            10
        )
        
        # Initialize camera info
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Initialize frame counter for processing rate control
        self.frame_count = 0
        self.process_every_n_frames = int(30 / self.frame_rate)
        if self.process_every_n_frames < 1:
            self.process_every_n_frames = 1
        
        self.get_logger().info('Camera node initialized')
        self.get_logger().info(f'Processing every {self.process_every_n_frames} frames')
        
    def load_model(self):
        """Load the TensorFlow model for product detection."""
        try:
            # Configure GPU memory growth if using GPU
            if self.use_gpu:
                gpus = tf.config.experimental.list_physical_devices('GPU')
                if gpus:
                    for gpu in gpus:
                        tf.config.experimental.set_memory_growth(gpu, True)
                    self.get_logger().info(f'Found {len(gpus)} GPU(s), configured memory growth')
            
            # Load saved model
            if os.path.exists(self.model_path):
                self.model = tf.saved_model.load(self.model_path)
                self.get_logger().info(f'Loaded model from {self.model_path}')
                
                # Get concrete function for detection
                self.detect_fn = self.model.signatures['serving_default']
                
            else:
                self.get_logger().error(f'Model not found at {self.model_path}')
                self.model = None
                self.detect_fn = None
                
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            self.model = None
            self.detect_fn = None
    
    def load_labels(self):
        """Load product labels from file."""
        try:
            if os.path.exists(self.label_path):
                with open(self.label_path, 'r') as f:
                    self.labels = [line.strip() for line in f.readlines()]
                self.get_logger().info(f'Loaded {len(self.labels)} labels from {self.label_path}')
            else:
                self.get_logger().error(f'Label file not found at {self.label_path}')
                self.labels = []
        except Exception as e:
            self.get_logger().error(f'Failed to load labels: {str(e)}')
            self.labels = []
    
    def camera_info_callback(self, msg):
        """Process camera calibration information."""
        with self.lock:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
    def image_callback(self, msg):
        """Process incoming camera images."""
        # Count frames to control processing rate
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image with the model
            with self.lock:
                if self.model is not None:
                    detections = self.detect_products(cv_image)
                    
                    # Draw detections on the image
                    annotated_image = self.draw_detections(cv_image, detections)
                    
                    # Convert back to ROS image and publish
                    img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                    img_msg.header = msg.header
                    self.processed_image_pub.publish(img_msg)
                    
                    # Publish individual detections
                    for detection in detections:
                        det_msg = ProductDetection()
                        det_msg.header = msg.header
                        det_msg.label = detection['label']
                        det_msg.score = float(detection['score'])
                        det_msg.x = int(detection['box'][0])
                        det_msg.y = int(detection['box'][1])
                        det_msg.width = int(detection['box'][2] - detection['box'][0])
                        det_msg.height = int(detection['box'][3] - detection['box'][1])
                        
                        self.detection_pub.publish(det_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_products(self, image):
        """Perform product detection on the image."""
        if self.detect_fn is None:
            return []
            
        # Prepare image for model input
        height, width, _ = image.shape
        input_tensor = tf.convert_to_tensor(image)
        input_tensor = input_tensor[tf.newaxis, ...]
        
        # Run inference
        detections = self.detect_fn(input_tensor)
        
        # Process results
        boxes = detections['detection_boxes'][0].numpy()
        scores = detections['detection_scores'][0].numpy()
        classes = detections['detection_classes'][0].numpy().astype(np.int32)
        
        results = []
        for i in range(len(scores)):
            if scores[i] >= self.confidence_threshold:
                # Convert normalized box coordinates to pixel values
                ymin, xmin, ymax, xmax = boxes[i]
                box = [
                    int(xmin * width),
                    int(ymin * height),
                    int(xmax * width),
                    int(ymax * height)
                ]
                
                # Get class label
                class_id = int(classes[i])
                if 0 <= class_id - 1 < len(self.labels):
                    label = self.labels[class_id - 1]
                else:
                    label = f"class_{class_id}"
                
                results.append({
                    'box': box,
                    'score': float(scores[i]),
                    'class': class_id,
                    'label': label
                })
        
        return results
        
    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on the image."""
        img_copy = image.copy()
        
        for detection in detections:
            box = detection['box']
            label = detection['label']
            score = detection['score']
            
            # Draw box
            cv2.rectangle(
                img_copy, 
                (box[0], box[1]), 
                (box[2], box[3]), 
                (0, 255, 0), 
                2
            )
            
            # Draw label
            label_text = f"{label}: {score:.2f}"
            cv2.putText(
                img_copy,
                label_text,
                (box[0], box[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
            
        return img_copy

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

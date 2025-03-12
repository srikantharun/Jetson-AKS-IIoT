#!/usr/bin/env python3
"""
Weight Sensor Node for Shelf Inventory Weight Verification System (SIWVS)
This node manages communication with weight sensors and publishes weight measurements.
"""

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
import threading
import time
from std_msgs.msg import Float64, String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from siwvs_msgs.msg import WeightSensor

class WeightSensorNode(Node):
    """ROS 2 node for handling weight sensor data."""

    def __init__(self):
        super().__init__('weight_sensor_node')
        
        # Declare parameters
        self.declare_parameter('config_file', '/config/sensor_config.yaml')
        self.declare_parameter('sampling_rate', 10.0)
        self.declare_parameter('sensor_id', 'sensor_01')
        self.declare_parameter('shelf_id', 'shelf_01')
        self.declare_parameter('row', 1)
        self.declare_parameter('column', 1)
        
        # Load configuration
        config_file = self.get_parameter('config_file').value
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load config file: {e}')
            self.config = {
                'weight_threshold': 0.05,
                'calibration_factor': 1.0,
                'sampling_rate': 10,
                'filtering': True,
                'filter_type': 'moving_average',
                'filter_window': 5,
                'units': 'kg'
            }
        
        # Initialize sensor parameters
        self.sensor_id = self.get_parameter('sensor_id').value
        self.shelf_id = self.get_parameter('shelf_id').value
        self.row = self.get_parameter('row').value
        self.column = self.get_parameter('column').value
        self.sampling_rate = self.get_parameter('sampling_rate').value
        
        # Sensor state
        self.calibration_factor = self.config.get('calibration_factor', 1.0)
        self.weight_threshold = self.config.get('weight_threshold', 0.05)
        self.filter_window = self.config.get('filter_window', 5)
        self.weight_buffer = []
        self.last_published_weight = 0.0
        self.is_calibrated = False
        
        # Create publishers
        self.weight_pub = self.create_publisher(
            WeightSensor, 
            f'/siwvs/weight/{self.sensor_id}', 
            10
        )
        
        self.raw_weight_pub = self.create_publisher(
            Float64,
            f'/siwvs/weight_raw/{self.sensor_id}',
            10
        )
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timers
        self.timer = self.create_timer(1.0 / self.sampling_rate, self.timer_callback)
        
        self.get_logger().info(f'Weight sensor node initialized: {self.sensor_id}')
    
    def timer_callback(self):
        """Timer callback for sensor readings."""
        # Simulate or read from actual sensor
        raw_weight = self.read_sensor()
        
        # Apply filtering if enabled
        if self.config.get('filtering', True):
            filtered_weight = self.apply_filter(raw_weight)
        else:
            filtered_weight = raw_weight
        
        # Apply calibration factor
        calibrated_weight = filtered_weight * self.calibration_factor
        
        # Check if weight changed significantly
        weight_changed = abs(calibrated_weight - self.last_published_weight) > self.weight_threshold
        
        # Create message
        msg = WeightSensor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'sensor_{self.sensor_id}'
        msg.sensor_id = self.sensor_id
        msg.shelf_id = self.shelf_id
        msg.row = self.row
        msg.column = self.column
        msg.raw_weight = raw_weight * 1000.0  # Convert to grams
        msg.calibrated_weight = calibrated_weight
        msg.confidence = self.calculate_confidence(filtered_weight)
        msg.is_calibrated = self.is_calibrated
        msg.is_stable = self.is_stable()
        msg.weight_changed = weight_changed
        
        # Publish weight data
        self.weight_pub.publish(msg)
        
        # Publish raw weight for debugging
        raw_msg = Float64()
        raw_msg.data = raw_weight
        self.raw_weight_pub.publish(raw_msg)
        
        # Update last published weight if changed
        if weight_changed:
            self.last_published_weight = calibrated_weight
            self.get_logger().info(f'Weight changed: {calibrated_weight:.3f} kg')
        
        # Publish transform
        self.publish_transform()
    
    def read_sensor(self):
        """Read raw sensor value (simulated or hardware)."""
        # In simulation, generate random weight with some noise
        # In real hardware, this would read from GPIO, I2C, etc.
        
        # Simulation code
        base_weight = 0.5  # 500g base weight
        noise = np.random.normal(0, 0.01)  # 10g standard deviation
        return base_weight + noise
    
    def apply_filter(self, raw_value):
        """Apply filtering to the raw sensor value."""
        # Add to buffer
        self.weight_buffer.append(raw_value)
        
        # Keep buffer at window size
        if len(self.weight_buffer) > self.filter_window:
            self.weight_buffer.pop(0)
        
        # Apply selected filter
        filter_type = self.config.get('filter_type', 'moving_average')
        
        if filter_type == 'moving_average':
            return sum(self.weight_buffer) / len(self.weight_buffer)
        elif filter_type == 'median':
            return sorted(self.weight_buffer)[len(self.weight_buffer) // 2]
        else:
            return raw_value
    
    def calculate_confidence(self, weight):
        """Calculate confidence level of the measurement."""
        if not self.weight_buffer:
            return 0.0
        
        # Simple confidence based on standard deviation
        if len(self.weight_buffer) < 2:
            return 0.5
        
        std_dev = np.std(self.weight_buffer)
        mean = np.mean(self.weight_buffer)
        
        # Normalized confidence (lower std_dev = higher confidence)
        if mean == 0:
            return 1.0
        
        cv = std_dev / abs(mean)  # Coefficient of variation
        confidence = max(0.0, min(1.0, 1.0 - (cv * 5.0)))
        
        return confidence
    
    def is_stable(self):
        """Check if weight measurement is stable."""
        if len(self.weight_buffer) < self.filter_window:
            return False
        
        std_dev = np.std(self.weight_buffer)
        return std_dev < (self.weight_threshold / 2.0)
    
    def publish_transform(self):
        """Publish the transform for this sensor."""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'shelf_{self.shelf_id}'
        t.child_frame_id = f'sensor_{self.sensor_id}'
        
        # Set sensor position based on row and column
        # This assumes 30cm spacing between sensors
        t.transform.translation.x = (self.column - 1) * 0.3
        t.transform.translation.y = (self.row - 1) * 0.3
        t.transform.translation.z = 0.0
        
        # Identity quaternion (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WeightSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Inventory Manager Node for Shelf Inventory System
This node fuses weight and vision data for accurate inventory tracking
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import sqlite3
import pandas as pd
import json
import os
from threading import Lock
from datetime import datetime

# Import custom message types
from siwvs_msgs.msg import WeightSensor, ProductDetection
from std_msgs.msg import String
from std_srvs.srv import Trigger

class InventoryManager(Node):
    """ROS 2 node for fusing weight and vision data to track inventory."""
    
    def __init__(self):
        super().__init__('inventory_manager')
        
        # Declare parameters
        self.declare_parameter('database_path', '/data/inventory.db')
        self.declare_parameter('product_catalog_path', '/data/product_catalog.json')
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('weight_confidence_threshold', 0.8)
        self.declare_parameter('vision_confidence_threshold', 0.7)
        self.declare_parameter('weight_timeout', 1.0)  # seconds
        self.declare_parameter('vision_timeout', 2.0)  # seconds
        
        # Get parameters
        self.database_path = self.get_parameter('database_path').value
        self.product_catalog_path = self.get_parameter('product_catalog_path').value
        self.update_rate = self.get_parameter('update_rate').value
        self.weight_confidence_threshold = self.get_parameter('weight_confidence_threshold').value
        self.vision_confidence_threshold = self.get_parameter('vision_confidence_threshold').value
        self.weight_timeout = self.get_parameter('weight_timeout').value
        self.vision_timeout = self.get_parameter('vision_timeout').value
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize inventory database
        self.init_database()
        
        # Load product catalog
        self.product_catalog = self.load_product_catalog()
        
        # Initialize data structures
        self.weight_data = {}  # Key: sensor_id, Value: weight data
        self.vision_data = {}  # Key: product_id, Value: detection data
        self.inventory_state = {}  # Key: shelf_id, Value: dict of products
        self.last_weight_timestamp = {}  # Key: sensor_id, Value: timestamp
        self.last_vision_timestamp = {}  # Key: product_id, Value: timestamp
        
        # Mutex for thread safety
        self.lock = Lock()
        
        # Create subscribers
        self.weight_subscriber = self.create_subscription(
            WeightSensor,
            '/siwvs/weight_data',
            self.weight_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.vision_subscriber = self.create_subscription(
            ProductDetection,
            '/siwvs/product_detection',
            self.vision_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create publishers
        self.inventory_publisher = self.create_publisher(
            String,
            '/siwvs/inventory_state',
            10
        )
        
        # Create services
        self.sync_service = self.create_service(
            Trigger,
            '/siwvs/sync_inventory',
            self.sync_inventory_callback,
            callback_group=self.callback_group
        )
        
        # Create timers
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_inventory,
            callback_group=self.callback_group
        )
        
        self.cleanup_timer = self.create_timer(
            5.0,  # Run every 5 seconds
            self.cleanup_stale_data,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Inventory manager node initialized')
        self.get_logger().info(f'Using database: {self.database_path}')
        
    def init_database(self):
        """Initialize the SQLite database for inventory tracking."""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.database_path), exist_ok=True)
            
            # Connect to database
            self.conn = sqlite3.connect(self.database_path)
            self.cursor = self.conn.cursor()
            
            # Create tables if they don't exist
            self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS inventory (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                shelf_id TEXT,
                sensor_id TEXT,
                product_id TEXT,
                quantity INTEGER,
                confidence REAL
            )
            ''')
            
            self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                event_type TEXT,
                shelf_id TEXT,
                sensor_id TEXT,
                product_id TEXT,
                quantity_change INTEGER,
                details TEXT
            )
            ''')
            
            self.conn.commit()
            self.get_logger().info('Database initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Database initialization failed: {str(e)}')
    
    def load_product_catalog(self):
        """Load product catalog from JSON file."""
        try:
            if os.path.exists(self.product_catalog_path):
                with open(self.product_catalog_path, 'r') as f:
                    catalog = json.load(f)
                self.get_logger().info(f'Loaded {len(catalog)} products from catalog')
                return catalog
            else:
                self.get_logger().warning(f'Product catalog not found at {self.product_catalog_path}')
                # Return empty catalog with defaults
                return {}
                
        except Exception as e:
            self.get_logger().error(f'Failed to load product catalog: {str(e)}')
            return {}
            
    def weight_callback(self, msg):
        """Process weight sensor data."""
        with self.lock:
            # Store weight data indexed by sensor ID
            sensor_id = msg.sensor_id
            self.weight_data[sensor_id] = {
                'shelf_id': msg.shelf_id,
                'row': msg.row,
                'column': msg.column,
                'weight': msg.calibrated_weight,
                'raw_weight': msg.raw_weight,
                'confidence': msg.confidence,
                'stable': msg.is_stable,
                'changed': msg.weight_changed,
                'timestamp': self.get_clock().now()
            }
            self.last_weight_timestamp[sensor_id] = self.get_clock().now()
            
            # Log significant weight changes
            if msg.weight_changed and msg.is_stable and msg.confidence >= self.weight_confidence_threshold:
                self.get_logger().info(f'Weight change detected on {sensor_id}: {msg.calibrated_weight:.2f} kg')
                
                # Try to infer product from weight
                products = self.infer_product_from_weight(msg.calibrated_weight)
                if products:
                    self.get_logger().info(f'Possible products: {", ".join([p["id"] for p in products])}')
    
    def vision_callback(self, msg):
        """Process vision detection data."""
        with self.lock:
            # Extract product ID from the label
            product_id = msg.label
            
            # Store vision data indexed by product ID
            if product_id not in self.vision_data:
                self.vision_data[product_id] = {
                    'detections': 0,
                    'confidence': 0.0,
                    'boxes': [],
                    'timestamps': []
                }
            
            # Update detection info
            self.vision_data[product_id]['detections'] += 1
            self.vision_data[product_id]['confidence'] = max(
                self.vision_data[product_id]['confidence'], 
                msg.score
            )
            self.vision_data[product_id]['boxes'].append([msg.x, msg.y, msg.width, msg.height])
            self.vision_data[product_id]['timestamps'].append(self.get_clock().now())
            
            # Limit history
            max_history = 10
            if len(self.vision_data[product_id]['boxes']) > max_history:
                self.vision_data[product_id]['boxes'] = self.vision_data[product_id]['boxes'][-max_history:]
                self.vision_data[product_id]['timestamps'] = self.vision_data[product_id]['timestamps'][-max_history:]
            
            self.last_vision_timestamp[product_id] = self.get_clock().now()
            
            # Log new product detections
            if self.vision_data[product_id]['detections'] == 1:
                self.get_logger().info(f'New product detected: {product_id} (confidence: {msg.score:.2f})')
    
    def update_inventory(self):
        """Update inventory state by fusing weight and vision data."""
        with self.lock:
            # Create shelf representation
            shelves = {}
            
            # Process weight data first
            for sensor_id, data in self.weight_data.items():
                shelf_id = data['shelf_id']
                row = data['row']
                column = data['column']
                
                # Initialize shelf if needed
                if shelf_id not in shelves:
                    shelves[shelf_id] = {}
                
                # Create position key
                position = f"{row}_{column}"
                
                # Add sensor data to shelf
                if position not in shelves[shelf_id]:
                    shelves[shelf_id][position] = {
                        'sensor_id': sensor_id,
                        'weight': data['weight'],
                        'products': []
                    }
                
                # Try to match weight to products in catalog
                if data['weight'] > 0.01:  # Ignore very small weights
                    products = self.infer_product_from_weight(data['weight'])
                    shelves[shelf_id][position]['inferred_products'] = products
            
            # Now add vision data
            for product_id, data in self.vision_data.items():
                # Skip low confidence detections
                if data['confidence'] < self.vision_confidence_threshold:
                    continue
                    
                # Find which shelf position this product is likely on
                # This requires mapping camera coordinates to shelf coordinates
                # For simplicity, we're using the existing weight data to correlate
                product_info = self.product_catalog.get(product_id, {})
                expected_weight = product_info.get('weight', 0.0)
                
                if expected_weight > 0.0:
                    # Find sensors with weight close to this product
                    for shelf_id, positions in shelves.items():
                        for position, pos_data in positions.items():
                            # Check if weight is within tolerance
                            weight_diff = abs(pos_data['weight'] - expected_weight)
                            weight_tolerance = expected_weight * 0.1  # 10% tolerance
                            
                            if weight_diff <= weight_tolerance:
                                # This is a potential match
                                pos_data['products'].append({
                                    'id': product_id,
                                    'confidence': data['confidence'],
                                    'count': 1  # Assuming single item for now
                                })
            
            # Update inventory state
            self.inventory_state = shelves
            
            # Publish inventory state
            self.publish_inventory_state()
    
    def publish_inventory_state(self):
        """Publish current inventory state as JSON."""
        try:
            # Convert inventory state to JSON
            inventory_json = json.dumps(self.inventory_state)
            
            # Create and publish message
            msg = String()
            msg.data = inventory_json
            self.inventory_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing inventory state: {str(e)}')
    
    def infer_product_from_weight(self, weight):
        """Infer possible products based on weight."""
        matching_products = []
        
        for product_id, product_info in self.product_catalog.items():
            if 'weight' in product_info:
                product_weight = float(product_info['weight'])
                
                # Calculate how many items might be present based on weight
                if product_weight > 0:
                    count = round(weight / product_weight)
                    
                    # If weight is close to a multiple of product weight, add to matches
                    if count > 0 and abs(weight - (count * product_weight)) < (product_weight * 0.15):
                        matching_products.append({
                            'id': product_id,
                            'name': product_info.get('name', product_id),
                            'expected_weight': product_weight,
                            'count': count,
                            'confidence': 1.0 - (abs(weight - (count * product_weight)) / (product_weight * count))
                        })
        
        # Sort by confidence
        matching_products.sort(key=lambda x: x['confidence'], reverse=True)
        
        return matching_products
    
    def cleanup_stale_data(self):
        """Remove stale data that hasn't been updated recently."""
        with self.lock:
            current_time = self.get_clock().now()
            
            # Clean up weight data
            stale_weight_sensors = []
            for sensor_id, timestamp in self.last_weight_timestamp.items():
                time_diff = (current_time - timestamp).nanoseconds / 1e9
                if time_diff > self.weight_timeout:
                    stale_weight_sensors.append(sensor_id)
            
            for sensor_id in stale_weight_sensors:
                if sensor_id in self.weight_data:
                    del self.weight_data[sensor_id]
                    del self.last_weight_timestamp[sensor_id]
            
            # Clean up vision data
            stale_vision_products = []
            for product_id, timestamp in self.last_vision_timestamp.items():
                time_diff = (current_time - timestamp).nanoseconds / 1e9
                if time_diff > self.vision_timeout:
                    stale_vision_products.append(product_id)
            
            for product_id in stale_vision_products:
                if product_id in self.vision_data:
                    del self.vision_data[product_id]
                    del self.last_vision_timestamp[product_id]
    
    def sync_inventory_callback(self, request, response):
        """Service callback to sync inventory with database."""
        try:
            # Get current timestamp
            timestamp = datetime.now().isoformat()
            
            # Begin transaction
            self.cursor.execute("BEGIN TRANSACTION")
            
            # Clear current inventory
            self.cursor.execute("DELETE FROM inventory")
            
            # Insert current inventory state
            for shelf_id, positions in self.inventory_state.items():
                for position, data in positions.items():
                    sensor_id = data['sensor_id']
                    
                    # Handle products detected by vision
                    for product in data.get('products', []):
                        self.cursor.execute('''
                        INSERT INTO inventory (timestamp, shelf_id, sensor_id, product_id, quantity, confidence)
                        VALUES (?, ?, ?, ?, ?, ?)
                        ''', (
                            timestamp,
                            shelf_id,
                            sensor_id,
                            product['id'],
                            product.get('count', 1),
                            product['confidence']
                        ))
                    
                    # Add inferred products from weight if no vision detections
                    if not data.get('products') and data.get('inferred_products'):
                        for product in data.get('inferred_products', [])[:1]:  # Just use top match
                            self.cursor.execute('''
                            INSERT INTO inventory (timestamp, shelf_id, sensor_id, product_id, quantity, confidence)
                            VALUES (?, ?, ?, ?, ?, ?)
                            ''', (
                                timestamp,
                                shelf_id,
                                sensor_id,
                                product['id'],
                                product['count'],
                                product['confidence'] * 0.7  # Lower confidence for weight-only
                            ))
            
            # Commit transaction
            self.conn.commit()
            
            # Log sync
            self.get_logger().info('Inventory synced to database')
            
            # Set response
            response.success = True
            response.message = "Inventory successfully synced to database"
            
        except Exception as e:
            self.get_logger().error(f'Error syncing inventory: {str(e)}')
            
            # Rollback transaction
            self.conn.rollback()
            
            # Set response
            response.success = False
            response.message = f"Error syncing inventory: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    node = InventoryManager()
    
    # Use MultiThreadedExecutor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

/**
 * Weight Sensor Node Implementation
 * 
 * This node handles weight sensor data and publishes it to ROS 2 topics.
 */

#include <rclcpp/rclcpp.hpp>
#include <siwvs_msgs/msg/weight_sensor.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <siwvs_control/control_interfaces.hpp>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Weight Sensor Node
 * 
 * This class implements a ROS 2 node for processing weight sensor data.
 */
class WeightSensorNode : public rclcpp::Node, public siwvs::WeightSensorInterface {
public:
  /**
   * @brief Construct a new Weight Sensor Node
   * 
   * @param node_name Name of the ROS node
   * @param sensor_id Unique identifier for this sensor
   * @param shelf_id Identifier for the shelf this sensor belongs to
   * @param row Row position in the shelf
   * @param column Column position in the shelf
   */
  WeightSensorNode(const std::string& node_name = "weight_sensor_node",
                  const std::string& sensor_id = "sensor_01",
                  const std::string& shelf_id = "shelf_01",
                  int row = 1, int column = 1)
  : Node(node_name),
    sensor_id_(sensor_id),
    shelf_id_(shelf_id),
    row_(row),
    column_(column),
    calibration_factor_(1.0),
    weight_threshold_(0.05),
    is_calibrated_(false),
    tf_broadcaster_(this)
  {
    // Initialize random generator for simulation
    std::random_device rd;
    gen_ = std::mt19937(rd());
    
    // Declare parameters
    this->declare_parameter("config_file", "/config/sensor_config.yaml");
    this->declare_parameter("sampling_rate", 10.0);
    this->declare_parameter("sensor_id", sensor_id_);
    this->declare_parameter("shelf_id", shelf_id_);
    this->declare_parameter("row", row_);
    this->declare_parameter("column", column_);
    
    // Get parameters
    sensor_id_ = this->get_parameter("sensor_id").as_string();
    shelf_id_ = this->get_parameter("shelf_id").as_string();
    row_ = this->get_parameter("row").as_int();
    column_ = this->get_parameter("column").as_int();
    double sampling_rate = this->get_parameter("sampling_rate").as_double();
    
    // Create publishers
    weight_pub_ = this->create_publisher<siwvs_msgs::msg::WeightSensor>(
      "/siwvs/weight/" + sensor_id_, 
      10
    );
    
    raw_weight_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/siwvs/weight_raw/" + sensor_id_,
      10
    );
    
    // Create timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / sampling_rate)),
      std::bind(&WeightSensorNode::timerCallback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Weight sensor node initialized: %s", sensor_id_.c_str());
  }
  
  /**
   * @brief Initialize the sensor
   * 
   * @param node_name Name of the ROS node
   * @param sensor_id Unique identifier for this sensor
   * @param shelf_id Identifier for the shelf this sensor belongs to
   * @param row Row position in the shelf
   * @param column Column position in the shelf
   * @return true If initialization was successful
   * @return false If initialization failed
   */
  bool initialize(const std::string& node_name, 
                const std::string& sensor_id,
                const std::string& shelf_id,
                int row, int column) override {
    sensor_id_ = sensor_id;
    shelf_id_ = shelf_id;
    row_ = row;
    column_ = column;
    return true;
  }
  
  /**
   * @brief Read the current weight from the sensor
   * 
   * @return double Raw weight reading in kilograms
   */
  double readWeight() override {
    // In simulation, generate random weight with some noise
    // In real hardware, this would read from GPIO, I2C, etc.
    
    // Simulation code
    double base_weight = 0.5;  // 500g base weight
    std::normal_distribution<double> noise(0.0, 0.01);  // 10g standard deviation
    return base_weight + noise(gen_);
  }
  
  /**
   * @brief Apply calibration to the raw weight reading
   * 
   * @param raw_weight Raw weight in kilograms
   * @return double Calibrated weight in kilograms
   */
  double calibrateWeight(double raw_weight) override {
    return raw_weight * calibration_factor_;
  }
  
  /**
   * @brief Publish the current weight sensor state
   * 
   * @param weight_msg The weight sensor message to publish
   */
  void publishWeight(const siwvs_msgs::msg::WeightSensor& weight_msg) override {
    weight_pub_->publish(weight_msg);
    
    // Also publish raw weight for debugging
    auto raw_msg = std_msgs::msg::Float64();
    raw_msg.data = weight_msg.raw_weight / 1000.0;  // Convert back to kg for consistency
    raw_weight_pub_->publish(raw_msg);
  }
  
  /**
   * @brief Publish the transform for this sensor
   */
  void publishTransform() override {
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->now();
    t.header.frame_id = "shelf_" + shelf_id_;
    t.child_frame_id = "sensor_" + sensor_id_;
    
    // Set sensor position based on row and column
    // This assumes 30cm spacing between sensors
    t.transform.translation.x = (column_ - 1) * 0.3;
    t.transform.translation.y = (row_ - 1) * 0.3;
    t.transform.translation.z = 0.0;
    
    // Identity quaternion (no rotation)
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    
    // Send the transform
    tf_broadcaster_.sendTransform(t);
  }
  
  /**
   * @brief Check if the weight is stable
   * 
   * @return true If the weight reading is stable
   * @return false If the weight reading is fluctuating
   */
  bool isStable() override {
    if (weight_buffer_.size() < filter_window_) {
      return false;
    }
    
    double sum = 0.0;
    for (const auto& weight : weight_buffer_) {
      sum += weight;
    }
    double mean = sum / weight_buffer_.size();
    
    double sum_sq_diff = 0.0;
    for (const auto& weight : weight_buffer_) {
      double diff = weight - mean;
      sum_sq_diff += diff * diff;
    }
    double std_dev = std::sqrt(sum_sq_diff / weight_buffer_.size());
    
    return std_dev < (weight_threshold_ / 2.0);
  }

private:
  /**
   * @brief Timer callback for sensor readings
   */
  void timerCallback() {
    // Read from sensor
    double raw_weight = readWeight();
    
    // Apply filtering if enabled
    if (weight_buffer_.size() >= filter_window_) {
      weight_buffer_.erase(weight_buffer_.begin());
    }
    weight_buffer_.push_back(raw_weight);
    
    // Calculate filtered weight
    double filtered_weight = 0.0;
    for (const auto& weight : weight_buffer_) {
      filtered_weight += weight;
    }
    filtered_weight /= weight_buffer_.size();
    
    // Apply calibration
    double calibrated_weight = calibrateWeight(filtered_weight);
    
    // Check if weight changed significantly
    bool weight_changed = std::abs(calibrated_weight - last_published_weight_) > weight_threshold_;
    
    // Create message
    auto msg = siwvs_msgs::msg::WeightSensor();
    msg.header.stamp = this->now();
    msg.header.frame_id = "sensor_" + sensor_id_;
    msg.sensor_id = sensor_id_;
    msg.shelf_id = shelf_id_;
    msg.row = row_;
    msg.column = column_;
    msg.raw_weight = raw_weight * 1000.0;  // Convert to grams for the message
    msg.calibrated_weight = calibrated_weight;
    msg.confidence = calculateConfidence();
    msg.is_calibrated = is_calibrated_;
    msg.is_stable = isStable();
    msg.weight_changed = weight_changed;
    
    // Publish weight data
    publishWeight(msg);
    
    // Update last published weight if changed
    if (weight_changed) {
      last_published_weight_ = calibrated_weight;
      RCLCPP_INFO(this->get_logger(), "Weight changed: %.3f kg", calibrated_weight);
    }
    
    // Publish transform
    publishTransform();
  }
  
  /**
   * @brief Calculate confidence level of the measurement
   * 
   * @return double Confidence value (0.0-1.0)
   */
  double calculateConfidence() {
    if (weight_buffer_.empty()) {
      return 0.0;
    }
    
    // Simple confidence based on standard deviation
    if (weight_buffer_.size() < 2) {
      return 0.5;
    }
    
    double sum = 0.0;
    for (const auto& weight : weight_buffer_) {
      sum += weight;
    }
    double mean = sum / weight_buffer_.size();
    
    double sum_sq_diff = 0.0;
    for (const auto& weight : weight_buffer_) {
      double diff = weight - mean;
      sum_sq_diff += diff * diff;
    }
    double std_dev = std::sqrt(sum_sq_diff / weight_buffer_.size());
    
    // Normalized confidence (lower std_dev = higher confidence)
    if (mean == 0) {
      return 1.0;
    }
    
    double cv = std_dev / std::abs(mean);  // Coefficient of variation
    double confidence = std::max(0.0, std::min(1.0, 1.0 - (cv * 5.0)));
    
    return confidence;
  }

  // Sensor parameters
  std::string sensor_id_;
  std::string shelf_id_;
  int row_;
  int column_;
  double calibration_factor_;
  double weight_threshold_;
  bool is_calibrated_;
  
  // Sensor state
  std::vector<double> weight_buffer_;
  double last_published_weight_ = 0.0;
  const size_t filter_window_ = 5;
  
  // Publishers
  rclcpp::Publisher<siwvs_msgs::msg::WeightSensor>::SharedPtr weight_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_weight_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  
  // Random generator for simulation
  std::mt19937 gen_;
};

/**
 * @brief Main function
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit code
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<WeightSensorNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

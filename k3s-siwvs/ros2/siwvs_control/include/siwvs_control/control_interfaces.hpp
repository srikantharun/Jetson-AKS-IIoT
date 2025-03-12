#ifndef SIWVS_CONTROL_INTERFACES_HPP
#define SIWVS_CONTROL_INTERFACES_HPP

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <siwvs_msgs/msg/weight_sensor.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace siwvs {

/**
 * @brief Interface for weight sensor nodes
 * 
 * This interface defines the common functionality for weight sensor nodes,
 * whether they are implemented in C++ or Python.
 */
class WeightSensorInterface {
public:
  virtual ~WeightSensorInterface() = default;
  
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
  virtual bool initialize(const std::string& node_name, 
                         const std::string& sensor_id,
                         const std::string& shelf_id,
                         int row, int column) = 0;
  
  /**
   * @brief Read the current weight from the sensor
   * 
   * @return double Raw weight reading in kilograms
   */
  virtual double readWeight() = 0;
  
  /**
   * @brief Apply calibration to the raw weight reading
   * 
   * @param raw_weight Raw weight in kilograms
   * @return double Calibrated weight in kilograms
   */
  virtual double calibrateWeight(double raw_weight) = 0;
  
  /**
   * @brief Publish the current weight sensor state
   * 
   * @param weight_msg The weight sensor message to publish
   */
  virtual void publishWeight(const siwvs_msgs::msg::WeightSensor& weight_msg) = 0;
  
  /**
   * @brief Publish the transform for this sensor
   */
  virtual void publishTransform() = 0;
  
  /**
   * @brief Check if the weight is stable
   * 
   * @return true If the weight reading is stable
   * @return false If the weight reading is fluctuating
   */
  virtual bool isStable() = 0;
};

} // namespace siwvs

#endif // SIWVS_CONTROL_INTERFACES_HPP

/*
 * Weight Sensor Plugin for Gazebo
 * This plugin simulates weight sensors for the SIWVS (Shelf Inventory Weight Verification System)
 */

#ifndef WEIGHT_SENSOR_PLUGIN_HPP
#define WEIGHT_SENSOR_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// ROS 2 integration
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_node.hpp>
#include <rclcpp/utilities.hpp>
#include <siwvs_msgs/msg/weight_sensor.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>

namespace gazebo
{
  class WeightSensorPlugin : public ModelPlugin
  {
    public: 
      WeightSensorPlugin();
      virtual ~WeightSensorPlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnUpdate();

    private:
      std::vector<std::string> SplitString(const std::string& str, char delim);

      struct SensorData {
        std::string sensor_id;
        size_t row;
        size_t col;
        double raw_weight;
        double calibrated_weight;
        double last_weight;
        bool is_stable;
        double confidence;
      };

      physics::ModelPtr model_;
      physics::WorldPtr world_;
      physics::PhysicsEnginePtr physics_;
      std::vector<physics::LinkPtr> sensor_links_;
      event::ConnectionPtr update_connection_;
      std::unique_ptr<transport::Node> transport_node_;
      std::string model_name_;
      std::string sensor_namespace_;
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
      double noise_mean_;
      double noise_stddev_;
      std::map<std::string, SensorData> sensors_;
      std::shared_ptr<rclcpp::Node> node_;
      std::map<std::string, rclcpp::Publisher<siwvs_msgs::msg::WeightSensor>::SharedPtr> publishers_;
      std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> raw_publishers_;
      std::mutex mutex_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WeightSensorPlugin)
}

#endif // WEIGHT_SENSOR_PLUGIN_HPP

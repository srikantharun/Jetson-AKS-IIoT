/*
 * Weight Sensor Plugin for Gazebo
 * This plugin simulates weight sensors for the SIWVS (Shelf Inventory Weight Verification System)
 * It detects objects on top of the sensor surface and calculates their weight.
 */

#include <weight_sensor_plugin/weight_sensor_plugin.hpp>

namespace gazebo
{
  WeightSensorPlugin::WeightSensorPlugin() : ModelPlugin()
  {
    // Initialize ROS node
    if (!rclcpp::is_initialized()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  WeightSensorPlugin::~WeightSensorPlugin()
  {
    update_connection_.reset();
    
    // Finalize ROS
    node_->~Node();
    rclcpp::shutdown();
  }

  void WeightSensorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the model pointer for convenience
    model_ = _model;
    world_ = model_->GetWorld();
    physics_ = world_->Physics();
    
    // Get the model name
    model_name_ = model_->GetName();
    
    // Default parameters
    sensor_namespace_ = "siwvs";
    update_rate_ = 30.0;
    noise_mean_ = 0.0;
    noise_stddev_ = 0.01; // 10 grams std deviation
    
    // Parse SDF parameters
    if (_sdf->HasElement("sensorNamespace"))
      sensor_namespace_ = _sdf->GetElement("sensorNamespace")->Get<std::string>();
    
    if (_sdf->HasElement("updateRate"))
      update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    
    if (_sdf->HasElement("noiseMean"))
      noise_mean_ = _sdf->GetElement("noiseMean")->Get<double>();
    
    if (_sdf->HasElement("noiseStdDev"))
      noise_stddev_ = _sdf->GetElement("noiseStdDev")->Get<double>();
    
    // Find all weight sensor links in the model
    auto link_names = model_->GetLinkNames();
    for (const auto& link_name : link_names) {
      if (link_name.find("sensor_") != std::string::npos) {
        sensor_links_.push_back(model_->GetLink(link_name));
        
        // Extract sensor ID, row, and column from the link name (format: "sensor_<row>_<col>")
        std::string sensor_id = link_name;
        size_t row = 1, col = 1;
        
        // Parse the row and column from the sensor ID if possible
        auto parts = SplitString(link_name, '_');
        if (parts.size() >= 3) {
          try {
            row = std::stoi(parts[1]);
            col = std::stoi(parts[2]);
          } catch (const std::exception& e) {
            gzerr << "Failed to parse row/column from sensor name: " << link_name << std::endl;
          }
        }
        
        // Initialize sensor data
        SensorData sensor_data;
        sensor_data.sensor_id = sensor_id;
        sensor_data.row = row;
        sensor_data.col = col;
        sensor_data.raw_weight = 0.0;
        sensor_data.calibrated_weight = 0.0;
        sensor_data.last_weight = 0.0;
        sensor_data.is_stable = true;
        sensor_data.confidence = 1.0;
        
        sensors_[link_name] = sensor_data;
        
        gzmsg << "Found weight sensor: " << link_name << " (row: " << row << ", col: " << col << ")" << std::endl;
      }
    }
    
    // Create ROS 2 node for the plugin
    node_ = std::make_shared<rclcpp::Node>("gazebo_weight_sensor_plugin");
    
    // Create publishers for each sensor
    for (const auto& sensor_pair : sensors_) {
      const std::string& sensor_name = sensor_pair.first;
      const SensorData& sensor_data = sensor_pair.second;
      
      // Create main weight sensor message publisher
      std::string topic_name = "/" + sensor_namespace_ + "/weight/" + sensor_data.sensor_id;
      auto publisher = node_->create_publisher<siwvs_msgs::msg::WeightSensor>(topic_name, 10);
      publishers_[sensor_name] = publisher;
      
      // Create raw weight publisher (for debugging)
      std::string raw_topic = "/" + sensor_namespace_ + "/weight_raw/" + sensor_data.sensor_id;
      auto raw_publisher = node_->create_publisher<std_msgs::msg::Float64>(raw_topic, 10);
      raw_publishers_[sensor_name] = raw_publisher;
      
      gzmsg << "Created publisher for sensor: " << sensor_name << " on topic: " << topic_name << std::endl;
    }
    
    // Create a new transport node
    transport_node_ = std::make_unique<transport::Node>();
    transport_node_->Init(model_->GetWorld()->Name());
    
    // Listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WeightSensorPlugin::OnUpdate, this));
    
    // Initialize the last update time
    last_update_time_ = world_->SimTime();
    
    // Set up the update period
    update_period_ = 1.0 / update_rate_;
    
    gzmsg << "Weight Sensor Plugin loaded successfully for model: " << model_name_ << std::endl;
  }

  void WeightSensorPlugin::OnUpdate()
  {
    // Get the current simulation time
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();
    
    // Check if it's time to update
    if (dt < update_period_)
      return;
    
    // Lock mutex for thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update each sensor
    for (const auto& link_pair : sensor_links_) {
      // Get the sensor link
      auto link = link_pair;
      std::string link_name = link->GetName();
      
      // Skip if this is not a valid sensor
      if (sensors_.find(link_name) == sensors_.end())
        continue;
      
      // Get the sensor data
      SensorData& sensor_data = sensors_[link_name];
      
      // Get sensor pose
      ignition::math::Pose3d sensor_pose = link->WorldPose();
      ignition::math::Vector3d sensor_pos = sensor_pose.Pos();
      
      // Calculate sensor dimensions from collision box
      auto collision = link->GetCollision("");
      ignition::math::Vector3d sensor_size = collision->BoundingBox().Size();
      double sensor_height = sensor_size.Z();
      
      // Get all models in the world
      auto models = world_->Models();
      
      // Reset weight
      double total_weight = 0.0;
      bool item_detected = false;
      
      // Check all models to see if they're on the sensor
      for (const auto& model : models) {
        // Skip the sensor model itself
        if (model->GetId() == model_->GetId())
          continue;
        
        // Check each link in the model
        for (const auto& model_link : model->GetLinks()) {
          ignition::math::Pose3d link_pose = model_link->WorldPose();
          ignition::math::Vector3d link_pos = link_pose.Pos();
          
          // Get bounding box of the link
          auto bbox = model_link->BoundingBox();
          ignition::math::Vector3d link_min = bbox.Min();
          ignition::math::Vector3d link_max = bbox.Max();
          
          // Calculate the link dimensions
          ignition::math::Vector3d link_size = bbox.Size();
          
          // Calculate the center of the link in world coordinates
          ignition::math::Vector3d link_center = link_pos;
          
          // Check if the link is on top of the sensor
          // Basic collision check: if the object's bottom is close to the sensor's top
          // and the horizontal position overlaps with the sensor
          bool is_above_sensor = link_center.Z() - link_size.Z()/2.0 <= (sensor_pos.Z() + sensor_height/2.0 + 0.02) && 
                                link_center.Z() - link_size.Z()/2.0 >= (sensor_pos.Z() + sensor_height/2.0 - 0.02);
          
          bool is_horizontally_aligned = 
              std::abs(link_center.X() - sensor_pos.X()) < (sensor_size.X() + link_size.X())/2.0 &&
              std::abs(link_center.Y() - sensor_pos.Y()) < (sensor_size.Y() + link_size.Y())/2.0;
          
          if (is_above_sensor && is_horizontally_aligned) {
            // Get the mass of the link
            double link_mass = model_link->GetInertial()->Mass();
            total_weight += link_mass;
            item_detected = true;
            
            // Debug output
            gzdbg << "Object detected on sensor " << link_name 
                 << ": " << model_link->GetName() 
                 << " (mass: " << link_mass << " kg)" << std::endl;
          }
        }
      }
      
      // Convert to kg and add noise
      double weight_kg = total_weight;
      double noise = ignition::math::Rand::DblNormal(noise_mean_, noise_stddev_);
      double measured_weight = weight_kg + noise;
      
      // Apply filtering (simple exponential filter)
      double alpha = 0.3; // Filter coefficient
      sensor_data.raw_weight = alpha * measured_weight + (1.0 - alpha) * sensor_data.raw_weight;
      
      // Apply calibration (identity for simulation)
      sensor_data.calibrated_weight = sensor_data.raw_weight;
      
      // Check if the weight is stable
      sensor_data.is_stable = std::abs(sensor_data.raw_weight - sensor_data.last_weight) < 0.01;
      
      // Calculate confidence level (higher for stable readings)
      sensor_data.confidence = sensor_data.is_stable ? 1.0 : 0.7;
      
      // Create and publish weight sensor message
      auto msg = std::make_shared<siwvs_msgs::msg::WeightSensor>();
      
      // Set header
      msg->header.stamp = node_->now();
      msg->header.frame_id = "sensor_" + sensor_data.sensor_id;
      
      // Set sensor information
      msg->sensor_id = sensor_data.sensor_id;
      msg->shelf_id = "shelf_01"; // Hardcoded for simulation
      msg->row = sensor_data.row;
      msg->column = sensor_data.col;
      
      // Set weight data
      msg->raw_weight = sensor_data.raw_weight * 1000.0; // Convert to grams
      msg->calibrated_weight = sensor_data.calibrated_weight;
      msg->confidence = sensor_data.confidence;
      
      // Set flags
      msg->is_calibrated = true; // Always calibrated in simulation
      msg->is_stable = sensor_data.is_stable;
      msg->weight_changed = std::abs(sensor_data.calibrated_weight - sensor_data.last_weight) > 0.05;
      
      // Publish the message
      publishers_[link_name]->publish(*msg);
      
      // Publish raw weight for debugging
      auto raw_msg = std::make_shared<std_msgs::msg::Float64>();
      raw_msg->data = sensor_data.raw_weight;
      raw_publishers_[link_name]->publish(*raw_msg);
      
      // Update last weight
      sensor_data.last_weight = sensor_data.calibrated_weight;
    }
    
    // Update the last update time
    last_update_time_ = current_time;
    
    // Process ROS 2 callbacks
    rclcpp::spin_some(node_);
  }

  std::vector<std::string> WeightSensorPlugin::SplitString(const std::string& str, char delim)
  {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string item;
    
    while (std::getline(ss, item, delim)) {
      result.push_back(item);
    }
    
    return result;
  }

} // namespace gazebo

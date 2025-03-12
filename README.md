# Jetson-AKS-IIoT: Weight Sensor System for Shelf Inventory

This repository provides a complete solution for deploying a Shelf Inventory Weight Verification System (SIWVS) using ROS 2 and k3s on Jetson Nano, with development and testing capabilities on Azure AKS.

## Project Overview

This system leverages weight sensors for real-time inventory tracking, using AI/ML for accurate product identification. The architecture follows a two-part approach:

1. **AKS Development Environment**: Terraform scripts to provision an ARM-based Azure Kubernetes Service cluster that mimics the resource constraints and architecture of the Jetson Nano.

2. **k3s Deployment for Jetson Nano**: Complete k3s setup for AI/ML workloads on the Jetson Nano, including ROS 2 integration, Gazebo simulation, and ML inference.

## System Features

- **Weight-based Inventory Tracking**: Monitor shelf inventory using an array of weight sensors
- **Robotic Arm Integration**: Control a robotic arm for automated shelf management
- **Real-time Simulation**: Use ROS 2 and Gazebo to simulate the full system
- **ML-powered Inference**: Analyze weight sensor data to infer product identity and inventory state
- **Edge Processing**: Run all components directly on the Jetson Nano

## Repository Structure

- [`terraform/`](terraform/): AKS infrastructure for development and testing
- [`k3s-siwvs/`](k3s-siwvs/): Complete k3s setup for Jetson Nano deployment

## Getting Started

### Prerequisites

- Azure CLI and subscription for AKS deployment
- Terraform 1.0+ for infrastructure provisioning
- Jetson Nano with JetPack 4.6+
- Docker and kubectl

### Development Environment Setup (AKS)

1. **Configure Azure credentials**:
   ```bash
   az login
   az account set --subscription "<your-subscription-id>"
   ```

2. **Initialize Terraform**:
   ```bash
   cd terraform
   terraform init
   ```

3. **Create Azure resources**:
   ```bash
   terraform apply
   ```

4. **Connect to AKS cluster**:
   ```bash
   az aks get-credentials --resource-group jetson-aks-rg --name jetson-aks-cluster
   ```

### Building Docker Images for ARM64

```bash
# Set up buildx for multi-arch builds
docker buildx create --name armbuilder
docker buildx use armbuilder
docker buildx inspect --bootstrap

# Build and push ROS 2 control image
cd k3s-siwvs/ros2
docker buildx build --platform linux/arm64 -t <your-registry>/siwvs/ros2-control:latest -f Dockerfile.arm64 --push .

# Build and push Gazebo simulation image
cd ../gazebo
docker buildx build --platform linux/arm64 -t <your-registry>/siwvs/gazebo-simulation:latest -f Dockerfile.arm64 --push .

# Build and push ML inference image
cd ../ml
docker buildx build --platform linux/arm64 -t <your-registry>/siwvs/weight-sensor-ml:latest -f Dockerfile.arm64 --push .
```

### Deploying to Jetson Nano

1. **Install k3s with optimized settings**:
   ```bash
   cd k3s-siwvs/setup
   sudo ./install.sh
   ```

2. **Deploy the SIWVS application**:
   ```bash
   # Set the ACR name in the deployment yaml
   export ACR_NAME=<your-acr-name>
   envsubst < ../kubernetes/deployments/siwvs-deployment.yaml | kubectl apply -f -
   
   # Apply other resources
   kubectl apply -f ../kubernetes/namespaces.yaml
   kubectl apply -f ../kubernetes/storage-classes/
   kubectl apply -f ../kubernetes/services/
   ```

3. **Access the application**:
   ```bash
   kubectl port-forward -n siwvs-system svc/ros2-control-service 9090:9090
   ```

## System Architecture

![SIWVS Architecture](docs/images/siwvs-architecture.png)

The system uses a layered architecture:

1. **Hardware Layer**: Weight sensors, Jetson Nano, robotic arm
2. **Kubernetes Layer**: k3s orchestrating containerized workloads
3. **Application Layer**: ROS 2 nodes for control, simulation, and ML inference
4. **Interface Layer**: APIs and visualization for monitoring inventory

## Development Workflow

### 1. Testing in AKS Before Deployment to Jetson

The AKS environment is configured to closely match the constraints of a Jetson Nano:

- **ARM64 Node Pool**: Tests ARM compatibility before deploying to Jetson
- **Resource Limits**: Memory and CPU limits match Jetson Nano specifications
- **Storage Configuration**: Similar storage setup to what will be used on Jetson

This allows you to develop and test your applications in the cloud before deploying to edge devices.

### 2. ROS 2 and Gazebo Development

For development of the ROS 2 components and simulation:

1. **Modify the simulation models**:
   ```bash
   cd k3s-siwvs/gazebo/models
   # Edit model files as needed
   ```

2. **Update ROS 2 nodes**:
   ```bash
   cd k3s-siwvs/ros2/siwvs_control/src
   # Edit Python or C++ files
   ```

3. **Test locally using ROS 2 launch files**:
   ```bash
   cd k3s-siwvs/ros2
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ros2 launch siwvs_bringup gazebo_simulation.launch.py
   ```

### 3. ML Model Development

To develop and test ML models for weight sensor data:

1. **Collect training data** from simulated or real sensors.
2. **Train models** using the scripts in `k3s-siwvs/ml/training/`.
3. **Convert models to ONNX format** for optimized inference on Jetson.
4. **Test inference** using the ML node.

## Performance Considerations for Jetson Nano

When deploying to Jetson Nano, consider these optimization strategies:

1. **GPU Acceleration**: The ML containers are configured to use CUDA for inference acceleration.
2. **Resource Management**: k3s is configured with resource limits appropriate for Jetson Nano.
3. **Storage Optimization**: Use persistent volumes sparingly and monitor disk usage.
4. **Power Management**: Configure Jetson power mode based on workload needs.

## Custom Gazebo Plugin for Weight Sensors

A key component of this system is the custom Gazebo plugin that simulates the weight sensors. The plugin:

1. Detects objects placed on the sensor surface
2. Calculates their weight based on physics properties
3. Publishes the weight data to ROS 2 topics

The plugin code is available in `k3s-siwvs/gazebo/plugins/weight_sensor_plugin/`.

## Training the ML Model

The system includes a pipeline for training ML models to recognize products based on weight signatures:

1. **Data Collection**:
   ```bash
   cd k3s-siwvs/ml/training
   python collect_weight_data.py --output-file weight_data.csv
   ```

2. **Model Training**:
   ```bash
   python train_model.py --input-file weight_data.csv --output-model weight_model.onnx
   ```

3. **Model Deployment**:
   ```bash
   # Copy model to the persistent volume
   kubectl cp weight_model.onnx siwvs-system/weight-sensor-ml-pod:/models/
   ```

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

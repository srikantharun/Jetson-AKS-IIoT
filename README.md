# SIWVS: Shelf Inventory Weight Verification System

## Setup Instructions

This document provides instructions for setting up and running the SIWVS (Shelf Inventory Weight Verification System) on both Jetson Nano hardware and Azure AKS for development.

## Repository Overview

The SIWVS system consists of the following key components:

1. **ROS 2 Control System** - Manages the weight sensors and robotic arm
2. **Gazebo Simulation** - Provides a physics-based simulation of the inventory shelf and sensors
3. **ML Inference Engine** - Analyzes weight patterns to identify products
4. **Kubernetes Deployment** - Containerizes and orchestrates the components

## Prerequisites

- For Jetson Nano deployment:
  - Jetson Nano with JetPack 4.6+
  - Docker
  - At least 32GB storage

- For AKS development:
  - Azure CLI and subscription
  - Terraform 1.0+
  - Docker with buildx for multi-arch builds

- For local development:
  - ROS 2 Humble
  - Gazebo 11
  - Python 3.8+

## Quick Start

Follow these steps to get started:

### 1. Run the Setup Script

```bash
# Clone the repository if you haven't already
git clone <repository-url>
cd jetson-aks-iiot-workspace

# Run the setup script
./scripts/setup.sh
```

This script will:
- Check for and install dependencies
- Create missing files and fix corrupted ones
- Prepare Gazebo models
- Set up build scripts for ROS 2, Docker, and Terraform

### 2. Fix Package XML Files

Some XML files in the repository have syntax issues. Run:

```bash
./scripts/fix-package-xml.sh
```

### 3. Build ROS 2 Packages (For Local Development)

```bash
./build_ros2.sh
```

### 4. Build Docker Images

```bash
# Build images locally
./build_docker.sh

# Or, to build and push to a registry
./build_docker.sh your-registry.azurecr.io true
```

### 5. Deploy to Jetson Nano

If you're on a Jetson Nano:

```bash
# Install k3s
sudo /tmp/k3s-install.sh

# After k3s is installed, deploy the SIWVS application
export ACR_NAME=<your-registry>  # Use "localhost:5000" for local registry
kubectl apply -f k3s-siwvs/kubernetes/namespaces.yaml
envsubst < k3s-siwvs/kubernetes/deployments/siwvs-deployment.yaml | kubectl apply -f -
kubectl apply -f k3s-siwvs/kubernetes/services/
```

### 6. Deploy to AKS (for Development)

```bash
# Initialize Terraform
./init_terraform.sh

# Deploy AKS cluster
cd terraform
terraform apply

# Get credentials
az aks get-credentials --resource-group $(terraform output -raw resource_group_name) --name $(terraform output -raw kubernetes_cluster_name)

# Deploy to AKS
export ACR_NAME=$(terraform output -raw acr_login_server)
kubectl apply -f k3s-siwvs/kubernetes/namespaces.yaml
envsubst < k3s-siwvs/kubernetes/deployments/siwvs-deployment.yaml | kubectl apply -f -
kubectl apply -f k3s-siwvs/kubernetes/services/
```

## Running the Simulation Locally

If you want to run the Gazebo simulation locally (without Kubernetes):

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source k3s-siwvs/ros2/install/setup.bash

# Launch Gazebo simulation
ros2 launch siwvs_bringup gazebo_simulation.launch.py
```

## Web Interface

The system exposes a ROS Bridge WebSocket interface:

- On Kubernetes: Access at `http://<node-ip>:30090`
- On local development: Access at `http://localhost:9090`

## Troubleshooting

### Missing or Broken Files

If you encounter issues with missing or corrupted files:

```bash
# Run the setup script again
./scripts/setup.sh
```

### Docker Build Issues

For ARM64 build issues:

```bash
# Setup buildx for multi-arch builds
docker buildx create --name armbuilder
docker buildx use armbuilder
docker buildx inspect --bootstrap
```

### Kubernetes Deployment Issues

```bash
# Check pod status
kubectl get pods -n siwvs-system

# View logs for a specific pod
kubectl logs -n siwvs-system <pod-name>
```

## Development Workflow

1. Make changes to the ROS 2 code
2. Rebuild packages with `./build_ros2.sh`
3. Test locally using launch files
4. Build Docker images with `./build_docker.sh`
5. Deploy to k3s or AKS

## Architecture

The SIWVS system uses a layered architecture:

1. **Hardware Layer**: Weight sensors, Jetson Nano, robotic arm
2. **Kubernetes Layer**: k3s orchestrating containerized workloads
3. **Application Layer**: ROS 2 nodes for control, simulation, and ML inference
4. **Interface Layer**: APIs and visualization for monitoring inventory

## License

This project is licensed under the MIT License - see the LICENSE file for details.

#!/bin/bash
# Complete setup script for the Jetson-AKS-IIoT SIWVS project
# This script handles both development environment and Jetson Nano deployment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

print_section() {
    echo -e "\n\e[1;36m===== $1 =====\e[0m\n"
}

print_subsection() {
    echo -e "\n\e[1;33m----- $1 -----\e[0m"
}

print_status() {
    echo -e "\e[0;32m[INFO]\e[0m $1"
}

print_warning() {
    echo -e "\e[0;33m[WARNING]\e[0m $1"
}

print_error() {
    echo -e "\e[0;31m[ERROR]\e[0m $1"
}

check_dependencies() {
    print_section "Checking dependencies"
    
    local missing_deps=0
    local deps=("docker" "git" "cmake" "make" "g++" "curl" "jq")
    
    for dep in "${deps[@]}"; do
        if ! command -v $dep &> /dev/null; then
            print_error "$dep is required but not installed"
            missing_deps=1
        else
            print_status "$dep found: $(command -v $dep)"
        fi
    done
    
    # Check for ROS 2 Humble
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        print_status "ROS 2 Humble found"
    else
        print_warning "ROS 2 Humble not found in /opt/ros/humble"
        print_warning "You may need to install ROS 2 Humble for local development"
    fi
    
    # Check for Terraform
    if command -v terraform &> /dev/null; then
        print_status "Terraform found: $(terraform --version | head -n 1)"
    else
        print_warning "Terraform not found - required for AKS deployment"
    fi
    
    # Check for kubectl
    if command -v kubectl &> /dev/null; then
        print_status "kubectl found: $(kubectl version --client 2>/dev/null | head -n 1 || kubectl version --client --short)"
    else
        print_warning "kubectl not found - required for Kubernetes management"
    fi
    
    # Check for Azure CLI
    if command -v az &> /dev/null; then
        print_status "Azure CLI found: $(az --version | head -n 1)"
    else
        print_warning "Azure CLI not found - required for AKS deployment"
    fi
    
    if [ $missing_deps -eq 1 ]; then
        print_error "Please install the missing dependencies before continuing"
        exit 1
    fi
}

setup_terraform_environment() {
    print_section "Setting up Terraform environment for AKS"
    
    if command -v terraform &> /dev/null; then
        print_status "Terraform is installed"
        
        # Create Terraform init script
        cat > "$REPO_ROOT/init_terraform.sh" << 'EOL'
#!/bin/bash
set -e

cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Initializing Terraform for AKS deployment"

# Check if Azure CLI is installed and logged in
if ! command -v az &> /dev/null; then
    echo "Azure CLI is not installed. Please install it first."
    echo "Visit: https://docs.microsoft.com/en-us/cli/azure/install-azure-cli"
    exit 1
fi

# Check if logged in to Azure
if ! az account show &> /dev/null; then
    echo "Not logged in to Azure. Please login first with 'az login'"
    exit 1
fi

# Create storage for Terraform state
echo "Creating Azure storage for Terraform state..."
RESOURCE_GROUP="tf-state-rg"
STORAGE_ACCOUNT="tfstatejetson$RANDOM"
CONTAINER_NAME="tfstate"

# Create resource group
az group create --name $RESOURCE_GROUP --location eastus

# Create storage account
az storage account create --resource-group $RESOURCE_GROUP --name $STORAGE_ACCOUNT --sku Standard_LRS --encryption-services blob

# Get storage account key
ACCOUNT_KEY=$(az storage account keys list --resource-group $RESOURCE_GROUP --account-name $STORAGE_ACCOUNT --query '[0].value' -o tsv)

# Create blob container
az storage container create --name $CONTAINER_NAME --account-name $STORAGE_ACCOUNT --account-key $ACCOUNT_KEY

# Update terraform/main.tf with the correct backend config
sed -i "s/storage_account_name = \"tfstatejetson\"/storage_account_name = \"$STORAGE_ACCOUNT\"/" terraform/main.tf

echo "Terraform state storage created:"
echo "  Resource Group: $RESOURCE_GROUP"
echo "  Storage Account: $STORAGE_ACCOUNT"
echo "  Container: $CONTAINER_NAME"

# Initialize Terraform
cd terraform
terraform init

echo "Terraform initialized successfully!"
echo "You can now run 'cd terraform && terraform apply' to provision AKS"
EOL
        chmod +x "$REPO_ROOT/init_terraform.sh"
        
        print_status "Created init_terraform.sh script for setting up Terraform environment"
    else
        print_warning "Terraform is not installed, skipping Terraform setup"
        print_status "Install Terraform to deploy to AKS"
    fi
}

main() {
    print_section "SIWVS Setup Script"
    
    # Check dependencies
    check_dependencies
    
    # Create missing files and directories
    create_missing_files
    
    # Copy model files to Gazebo directory
    copy_model_files
    
    # Build ROS 2 packages
    build_ros2_packages
    
    # Prepare Docker build environment
    prepare_docker_build
    
    # Setup Jetson environment if running on Jetson
    setup_jetson_environment
    
    # Setup Terraform environment for AKS
    setup_terraform_environment
    
    print_section "Setup completed successfully!"
    print_status "The following scripts have been created:"
    print_status "  - build_ros2.sh: Build ROS 2 packages locally"
    print_status "  - build_docker.sh: Build Docker images for deployment"
    if [ -f "$REPO_ROOT/init_terraform.sh" ]; then
        print_status "  - init_terraform.sh: Initialize Terraform for AKS deployment"
    fi
    
    print_status ""
    print_status "Next steps:"
    print_status "1. Build the ROS 2 packages: ./build_ros2.sh"
    print_status "2. Build Docker images: ./build_docker.sh"
    print_status "3. For Jetson deployment: sudo /tmp/k3s-install.sh"
    print_status "4. For AKS deployment: ./init_terraform.sh"
    
    print_section "Happy developing with SIWVS!"
}

# Execute main function
main

create_missing_files() {
    print_section "Creating missing files and directories"
    
    # Check and create the .gitignore file if empty
    if [ ! -s "$REPO_ROOT/.gitignore" ]; then
        print_status "Creating .gitignore file"
        cat > "$REPO_ROOT/.gitignore" << 'EOL'
# ROS 2 build artifacts
build/
install/
log/

# Python artifacts
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
*.egg-info/
.installed.cfg
*.egg

# Terraform artifacts
.terraform/
*.tfstate
*.tfstate.*
*.tfvars
.terraform.lock.hcl

# IDE artifacts
.idea/
.vscode/
*.swp
*.swo

# System files
.DS_Store
Thumbs.db

# Log files
*.log

# Docker artifacts
.docker/

# Compiled binaries
*.o
*.a
*.so
*.dll
*.dylib
*.exe

# ROS bags
*.bag
*.db3

# Custom
config.yaml
EOL
    fi
    
    # Create Gazebo model files if empty
    if [ ! -s "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf/model.config" ]; then
        print_status "Creating shelf model.config"
        mkdir -p "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf"
        cat > "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf/model.config" << 'EOL'
<?xml version="1.0"?>
<model>
  <name>Inventory Shelf</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>SIWVS Developer</name>
    <email>user@example.com</email>
  </author>

  <description>
    A shelf with integrated weight sensors for inventory tracking.
  </description>
</model>
EOL
    fi
    
    if [ ! -s "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf/model.sdf" ]; then
        print_status "Creating shelf model.sdf"
        cat > "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf/model.sdf" << 'EOL'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="inventory_shelf">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    
    <!-- Base of the shelf -->
    <link name="base">
      <collision name="base_collision">
        <geometry>
          <box>
            <size>1.2 0.6 0.05</size>
          </box>
        </geometry>
      </collision>
      <visual name="base_visual">
        <geometry>
          <box>
            <size>1.2 0.6 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Vertical supports -->
    <link name="left_support">
      <pose>-0.575 0 0.6 0 0 0</pose>
      <collision name="left_support_collision">
        <geometry>
          <box>
            <size>0.05 0.6 1.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="left_support_visual">
        <geometry>
          <box>
            <size>0.05 0.6 1.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="right_support">
      <pose>0.575 0 0.6 0 0 0</pose>
      <collision name="right_support_collision">
        <geometry>
          <box>
            <size>0.05 0.6 1.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="right_support_visual">
        <geometry>
          <box>
            <size>0.05 0.6 1.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Shelf layers -->
    <link name="shelf_1">
      <pose>0 0 0.3 0 0 0</pose>
      <collision name="shelf_1_collision">
        <geometry>
          <box>
            <size>1.2 0.6 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual name="shelf_1_visual">
        <geometry>
          <box>
            <size>1.2 0.6 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="shelf_2">
      <pose>0 0 0.6 0 0 0</pose>
      <collision name="shelf_2_collision">
        <geometry>
          <box>
            <size>1.2 0.6 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual name="shelf_2_visual">
        <geometry>
          <box>
            <size>1.2 0.6 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="shelf_3">
      <pose>0 0 0.9 0 0 0</pose>
      <collision name="shelf_3_collision">
        <geometry>
          <box>
            <size>1.2 0.6 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual name="shelf_3_visual">
        <geometry>
          <box>
            <size>1.2 0.6 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Weight sensors (3x3 grid, one for each position) -->
    <!-- Row 1 -->
    <link name="sensor_1_1">
      <pose>-0.35 0.15 0.315 0 0 0</pose>
      <collision name="sensor_1_1_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_1_1_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="sensor_1_2">
      <pose>0 0.15 0.315 0 0 0</pose>
      <collision name="sensor_1_2_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_1_2_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="sensor_1_3">
      <pose>0.35 0.15 0.315 0 0 0</pose>
      <collision name="sensor_1_3_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_1_3_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Row 2 -->
    <link name="sensor_2_1">
      <pose>-0.35 0.15 0.615 0 0 0</pose>
      <collision name="sensor_2_1_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_2_1_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="sensor_2_2">
      <pose>0 0.15 0.615 0 0 0</pose>
      <collision name="sensor_2_2_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_2_2_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="sensor_2_3">
      <pose>0.35 0.15 0.615 0 0 0</pose>
      <collision name="sensor_2_3_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_2_3_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Row 3 -->
    <link name="sensor_3_1">
      <pose>-0.35 0.15 0.915 0 0 0</pose>
      <collision name="sensor_3_1_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_3_1_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="sensor_3_2">
      <pose>0 0.15 0.915 0 0 0</pose>
      <collision name="sensor_3_2_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_3_2_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <link name="sensor_3_3">
      <pose>0.35 0.15 0.915 0 0 0</pose>
      <collision name="sensor_3_3_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
      </collision>
      <visual name="sensor_3_3_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Plugin for weight sensing -->
    <plugin name="weight_sensor_plugin" filename="libWeightSensorPlugin.so">
      <sensorNamespace>siwvs</sensorNamespace>
      <updateRate>30.0</updateRate>
      <noiseMean>0.0</noiseMean>
      <noiseStdDev>0.01</noiseStdDev>
    </plugin>
  </model>
</sdf>
EOL
    fi
    
    # Fix the shelf_inventory.world file if it's corrupt
    if [ -s "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world" ]; then
        if grep -q "#" "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world"; then
            print_status "Fixing corrupted shelf_inventory.world file"
            mv "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world" "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world.bak"
        fi
    fi
    
    if [ ! -s "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world" ] || [ -f "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world.bak" ]; then
        print_status "Creating shelf_inventory.world file"
        cat > "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/worlds/shelf_inventory.world" << 'EOL'
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="shelf_inventory_world">
    <!-- Physics settings optimized for small objects -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Inventory shelf with weight sensors -->
    <include>
      <uri>model://shelf</uri>
      <name>inventory_shelf</name>
      <pose>1.5 0 0 0 0 0</pose>
    </include>
    
    <!-- ROS interface -->
    <plugin name="gazebo_ros_interface" filename="libgazebo_ros_api_plugin.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </world>
</sdf>
EOL
    fi
    
    # Create Terraform providers.tf if empty
    if [ ! -s "$REPO_ROOT/terraform/providers.tf" ]; then
        print_status "Creating Terraform providers.tf"
        cat > "$REPO_ROOT/terraform/providers.tf" << 'EOL'
# Azure Provider configuration
provider "azurerm" {
  features {}

  # Uncomment to use specific subscription
  # subscription_id = var.subscription_id
  # tenant_id       = var.tenant_id
}

# Random provider for unique resource names
provider "random" {
}

# Time provider for resource rotation policies
provider "time" {
}
EOL
    fi
    
    # Create Kubernetes directory structure for k3s deployments
    print_status "Creating Kubernetes deployment directories"
    mkdir -p "$REPO_ROOT/k3s-siwvs/kubernetes/deployments"
    mkdir -p "$REPO_ROOT/k3s-siwvs/kubernetes/namespaces"
    mkdir -p "$REPO_ROOT/k3s-siwvs/kubernetes/services"
    mkdir -p "$REPO_ROOT/k3s-siwvs/kubernetes/storage-classes"
    
    # Create namespace definition
    if [ ! -s "$REPO_ROOT/k3s-siwvs/kubernetes/namespaces.yaml" ]; then
        print_status "Creating Kubernetes namespaces.yaml"
        cat > "$REPO_ROOT/k3s-siwvs/kubernetes/namespaces.yaml" << 'EOL'
apiVersion: v1
kind: Namespace
metadata:
  name: siwvs-system
  labels:
    name: siwvs-system
    part-of: siwvs
---
apiVersion: v1
kind: Namespace
metadata:
  name: siwvs-monitoring
  labels:
    name: siwvs-monitoring
    part-of: siwvs
EOL
    fi
    
    # Create SIWVS deployment file
    if [ ! -s "$REPO_ROOT/k3s-siwvs/kubernetes/deployments/siwvs-deployment.yaml" ]; then
        print_status "Creating SIWVS deployment YAML"
        cat > "$REPO_ROOT/k3s-siwvs/kubernetes/deployments/siwvs-deployment.yaml" << 'EOL'
apiVersion: apps/v1
kind: Deployment
metadata:
  name: siwvs-ros2-control
  namespace: siwvs-system
  labels:
    app: siwvs
    component: ros2-control
spec:
  replicas: 1
  selector:
    matchLabels:
      app: siwvs
      component: ros2-control
  template:
    metadata:
      labels:
        app: siwvs
        component: ros2-control
    spec:
      containers:
      - name: ros2-control
        image: ${ACR_NAME}/siwvs/ros2-control:latest
        imagePullPolicy: Always
        resources:
          limits:
            cpu: "1"
            memory: "1Gi"
          requests:
            cpu: "500m"
            memory: "512Mi"
        ports:
        - containerPort: 9090
          name: rosbridge
        volumeMounts:
        - name: config-volume
          mountPath: /config
        - name: ros2-data
          mountPath: /ros2_data
        env:
        - name: ROS_DOMAIN_ID
          value: "0"
        - name: SIWVS_SIMULATION
          value: "true"
      volumes:
      - name: config-volume
        configMap:
          name: siwvs-config
      - name: ros2-data
        persistentVolumeClaim:
          claimName: ros2-data-pvc
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: siwvs-gazebo-simulation
  namespace: siwvs-system
  labels:
    app: siwvs
    component: gazebo-simulation
spec:
  replicas: 1
  selector:
    matchLabels:
      app: siwvs
      component: gazebo-simulation
  template:
    metadata:
      labels:
        app: siwvs
        component: gazebo-simulation
    spec:
      containers:
      - name: gazebo-simulation
        image: ${ACR_NAME}/siwvs/gazebo-simulation:latest
        imagePullPolicy: Always
        resources:
          limits:
            cpu: "2"
            memory: "2Gi"
          requests:
            cpu: "1"
            memory: "1Gi"
        volumeMounts:
        - name: ros2-data
          mountPath: /ros2_data
        env:
        - name: ROS_DOMAIN_ID
          value: "0"
        - name: GAZEBO_MODEL_PATH
          value: "/opt/ros/humble/share/gazebo_ros/models:/models"
      volumes:
      - name: ros2-data
        persistentVolumeClaim:
          claimName: ros2-data-pvc
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: siwvs-weight-sensor-ml
  namespace: siwvs-system
  labels:
    app: siwvs
    component: weight-sensor-ml
spec:
  replicas: 1
  selector:
    matchLabels:
      app: siwvs
      component: weight-sensor-ml
  template:
    metadata:
      labels:
        app: siwvs
        component: weight-sensor-ml
    spec:
      containers:
      - name: weight-sensor-ml
        image: ${ACR_NAME}/siwvs/weight-sensor-ml:latest
        imagePullPolicy: Always
        resources:
          limits:
            cpu: "2"
            memory: "2Gi"
            nvidia.com/gpu: "1"
          requests:
            cpu: "500m"
            memory: "1Gi"
        volumeMounts:
        - name: ml-models
          mountPath: /models
        - name: ros2-data
          mountPath: /ros2_data
        env:
        - name: ROS_DOMAIN_ID
          value: "0"
        - name: CUDA_VISIBLE_DEVICES
          value: "0"
      volumes:
      - name: ml-models
        persistentVolumeClaim:
          claimName: ml-models-pvc
      - name: ros2-data
        persistentVolumeClaim:
          claimName: ros2-data-pvc
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: siwvs-config
  namespace: siwvs-system
data:
  sensor_config.yaml: |
    weight_threshold: 0.05
    calibration_factor: 1.0
    filtering: true
    filter_type: moving_average
    filter_window: 5
    units: kg
EOL
    fi
    
    # Create service definitions
    if [ ! -s "$REPO_ROOT/k3s-siwvs/kubernetes/services/ros2-control-service.yaml" ]; then
        print_status "Creating ROS 2 control service"
        cat > "$REPO_ROOT/k3s-siwvs/kubernetes/services/ros2-control-service.yaml" << 'EOL'
apiVersion: v1
kind: Service
metadata:
  name: ros2-control-service
  namespace: siwvs-system
  labels:
    app: siwvs
    component: ros2-control
spec:
  selector:
    app: siwvs
    component: ros2-control
  ports:
  - port: 9090
    targetPort: 9090
    protocol: TCP
    name: rosbridge
  type: ClusterIP
EOL
    fi
    
    # Create docker directory for building container images
    print_status "Creating Docker build directory structure"
    
    mkdir -p "$REPO_ROOT/k3s-siwvs/docker/ros2"
    mkdir -p "$REPO_ROOT/k3s-siwvs/docker/gazebo"
    mkdir -p "$REPO_ROOT/k3s-siwvs/docker/ml"
    
    # Create ROS 2 Dockerfile if missing
    if [ ! -s "$REPO_ROOT/k3s-siwvs/docker/ros2/Dockerfile.arm64" ]; then
        print_status "Creating ROS 2 Dockerfile for ARM64"
        cat > "$REPO_ROOT/k3s-siwvs/docker/ros2/Dockerfile.arm64" << 'EOL'
FROM arm64v8/ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    python3-yaml \
    ros-humble-rosbridge-server \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws

# Copy and build ROS packages
COPY k3s-siwvs/ros2/siwvs_msgs /ros2_ws/src/siwvs_msgs
COPY k3s-siwvs/ros2/siwvs_control /ros2_ws/src/siwvs_control
COPY k3s-siwvs/ros2/siwvs_bringup /ros2_ws/src/siwvs_bringup

# Build the ROS 2 packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Setup entrypoint
COPY k3s-siwvs/docker/ros2/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "siwvs_bringup", "control_only.launch.py"]
EOL
    fi
    
    # Create ROS 2 entrypoint script
    if [ ! -s "$REPO_ROOT/k3s-siwvs/docker/ros2/entrypoint.sh" ]; then
        print_status "Creating ROS 2 entrypoint script"
        cat > "$REPO_ROOT/k3s-siwvs/docker/ros2/entrypoint.sh" << 'EOL'
#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

# Execute command
exec "$@"
EOL
        chmod +x "$REPO_ROOT/k3s-siwvs/docker/ros2/entrypoint.sh"
    fi
    
    # Create Gazebo Dockerfile if missing
    if [ ! -s "$REPO_ROOT/k3s-siwvs/docker/gazebo/Dockerfile.arm64" ]; then
        print_status "Creating Gazebo Dockerfile for ARM64"
        cat > "$REPO_ROOT/k3s-siwvs/docker/gazebo/Dockerfile.arm64" << 'EOL'
FROM arm64v8/ros:humble

# Install Gazebo and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros-control \
    ros-humble-tf2-ros \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    python3-numpy \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws

# Copy and build ROS packages
COPY k3s-siwvs/ros2/siwvs_msgs /ros2_ws/src/siwvs_msgs
COPY k3s-siwvs/ros2/siwvs_control /ros2_ws/src/siwvs_control
COPY k3s-siwvs/ros2/siwvs_bringup /ros2_ws/src/siwvs_bringup
COPY k3s-siwvs/gazebo/plugins/weight_sensor_plugin /ros2_ws/src/weight_sensor_plugin

# Copy Gazebo models
COPY k3s-siwvs/gazebo/models /models

# Build the ROS 2 packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Setup entrypoint
COPY k3s-siwvs/docker/gazebo/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "siwvs_bringup", "gazebo_simulation.launch.py"]
EOL
    fi
    
    # Create Gazebo entrypoint script
    if [ ! -s "$REPO_ROOT/k3s-siwvs/docker/gazebo/entrypoint.sh" ]; then
        print_status "Creating Gazebo entrypoint script"
        cat > "$REPO_ROOT/k3s-siwvs/docker/gazebo/entrypoint.sh" << 'EOL'
#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

# Setup Gazebo environment
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/models
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_PLUGIN_PATH=/ros2_ws/install/weight_sensor_plugin/lib:$GAZEBO_PLUGIN_PATH

# Execute command
exec "$@"
EOL
        chmod +x "$REPO_ROOT/k3s-siwvs/docker/gazebo/entrypoint.sh"
    fi
    
    # Create ML Dockerfile if missing
    if [ ! -s "$REPO_ROOT/k3s-siwvs/docker/ml/Dockerfile.arm64" ]; then
        print_status "Creating ML Dockerfile for ARM64"
        cat > "$REPO_ROOT/k3s-siwvs/docker/ml/Dockerfile.arm64" << 'EOL'
FROM nvcr.io/nvidia/l4t-ml:r35.2.1-py3

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python ML libraries
RUN pip3 install --no-cache-dir \
    numpy==1.24.1 \
    scikit-learn==1.3.0 \
    onnx==1.14.0 \
    onnxruntime-gpu==1.15.1 \
    pandas==2.0.3

# Set up workspace
WORKDIR /ros2_ws

# Copy ROS packages
COPY k3s-siwvs/ros2/siwvs_msgs /ros2_ws/src/siwvs_msgs

# Create ML scripts directory
RUN mkdir -p /ml_scripts
COPY k3s-siwvs/ml/inference /ml_scripts

# Build the ROS 2 packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Setup entrypoint
COPY k3s-siwvs/docker/ml/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/ml_scripts/weight_inference.py"]
EOL
    fi
    
    # Create ML entrypoint script
    if [ ! -s "$REPO_ROOT/k3s-siwvs/docker/ml/entrypoint.sh" ]; then
        print_status "Creating ML entrypoint script"
        cat > "$REPO_ROOT/k3s-siwvs/docker/ml/entrypoint.sh" << 'EOL'
#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"

# Setup ML environment
export PYTHONPATH=$PYTHONPATH:/ml_scripts
export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}

# Execute command
exec "$@"
EOL
        chmod +x "$REPO_ROOT/k3s-siwvs/docker/ml/entrypoint.sh"
    fi
    
    # Create ML inference directory and sample script
    if [ ! -d "$REPO_ROOT/k3s-siwvs/ml/inference" ]; then
        print_status "Creating ML inference directory and sample script"
        mkdir -p "$REPO_ROOT/k3s-siwvs/ml/inference"
        cat > "$REPO_ROOT/k3s-siwvs/ml/inference/weight_inference.py" << 'EOL'
#!/usr/bin/env python3
"""
Weight Sensor ML Inference Node
This node subscribes to weight sensor data and performs ML inference to identify items.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import os
import time
import json
from threading import Lock
from std_msgs.msg import String
from siwvs_msgs.msg import WeightSensor

class WeightInferenceNode(Node):
    """ROS 2 node for performing ML inference on weight sensor data."""
    
    def __init__(self):
        super().__init__('weight_inference_node')
        
        # Configuration
        self.declare_parameter('model_path', '/models/weight_model.onnx')
        self.declare_parameter('use_gpu', True)
        
        self.model_path = self.get_parameter('model_path').value
        self.use_gpu = self.get_parameter('use_gpu').value
        
        # State
        self.sensors_data = {}
        self.lock = Lock()
        self.inference_running = False
        
        # Initialize ML model
        self.initialize_model()
        
        # Create subscribers for each sensor
        for row in range(1, 4):
            for col in range(1, 4):
                sensor_id = f'sensor_{row}_{col}'
                self.create_subscription(
                    WeightSensor,
                    f'/siwvs/weight/{sensor_id}',
                    self.weight_callback,
                    10
                )
        
        # Create publishers
        self.inference_pub = self.create_publisher(
            String,
            '/siwvs/inference/result',
            10
        )
        
        # Create timer for periodic inference
        self.timer = self.create_timer(1.0, self.inference_timer_callback)
        
        self.get_logger().info('Weight inference node initialized')
    
    def initialize_model(self):
        """Initialize the ML model."""
        try:
            # Here we're creating a mock model for the example
            # In a real implementation, you would load an ONNX model
            self.get_logger().info(f'Initializing ML model from {self.model_path}')
            
            # Mock model for demonstration
            self.item_classes = ['empty', 'small_box', 'medium_box', 'large_box', 'bottle', 'can']
            
            # Check if model file exists
            if os.path.exists(self.model_path):
                self.get_logger().info(f'Model file found at {self.model_path}')
                # In a real implementation: 
                # import onnxruntime as ort
                # providers = ['CUDAExecutionProvider'] if self.use_gpu else ['CPUExecutionProvider']
                # self.session = ort.InferenceSession(self.model_path, providers=providers)
            else:
                self.get_logger().warning(f'Model file not found at {self.model_path}, using mock model')
        
        except Exception as e:
            self.get_logger().error(f'Failed to initialize model: {e}')
    
    def weight_callback(self, msg):
        """Callback for weight sensor messages."""
        with self.lock:
            sensor_key = f'{msg.row}_{msg.column}'
            self.sensors_data[sensor_key] = {
                'weight': msg.calibrated_weight,
                'is_stable': msg.is_stable,
                'confidence': msg.confidence,
                'timestamp': self.get_clock().now().to_msg()
            }
    
    def inference_timer_callback(self):
        """Periodic callback to run inference."""
        if self.inference_running:
            return
        
        self.inference_running = True
        try:
            with self.lock:
                # Copy data to avoid modification during processing
                sensor_data = self.sensors_data.copy()
            
            # Only run inference if we have data and sensors are stable
            if sensor_data and all(data.get('is_stable', False) for data in sensor_data.values()):
                self.run_inference(sensor_data)
        finally:
            self.inference_running = False
    
    def run_inference(self, sensor_data):
        """Run inference on sensor data."""
        try:
            # Extract features from sensor data
            features = []
            for row in range(1, 4):
                for col in range(1, 4):
                    key = f'{row}_{col}'
                    if key in sensor_data:
                        features.append(sensor_data[key]['weight'])
                    else:
                        features.append(0.0)
            
            features = np.array(features, dtype=np.float32).reshape(1, -1)
            
            # In a real implementation, you would run inference with the ONNX model:
            # result = self.session.run(None, {'input': features})
            
            # Mock inference for the example
            time.sleep(0.1)  # Simulate processing time
            
            # Determine the item based on the weight pattern
            total_weight = sum(features[0])
            
            # Simple mock classification based on weight
            if total_weight < 0.1:
                item_class = 'empty'
                confidence = 0.95
            elif total_weight < 0.5:
                item_class = 'small_box'
                confidence = 0.85
            elif total_weight < 1.0:
                item_class = 'medium_box'
                confidence = 0.82
            elif total_weight < 2.0:
                item_class = 'large_box'
                confidence = 0.79
            elif features[0][4] > 0.3:  # Center position has significant weight
                item_class = 'bottle'
                confidence = 0.75
            else:
                item_class = 'can'
                confidence = 0.70
            
            # Create result message
            result = {
                'timestamp': self.get_clock().now().seconds_nanoseconds(),
                'class': item_class,
                'confidence': confidence,
                'weights': features[0].tolist()
            }
            
            # Publish result
            msg = String()
            msg.data = json.dumps(result)
            self.inference_pub.publish(msg)
            
            self.get_logger().info(f'Inference result: {item_class} (conf: {confidence:.2f})')
        
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WeightInferenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOL
    fi
    
    # Create ML training directory and sample script
    if [ ! -d "$REPO_ROOT/k3s-siwvs/ml/training" ]; then
        print_status "Creating ML training directory and sample script"
        mkdir -p "$REPO_ROOT/k3s-siwvs/ml/training"
        cat > "$REPO_ROOT/k3s-siwvs/ml/training/train_model.py" << 'EOL'
#!/usr/bin/env python3
"""
Weight Sensor ML Model Training Script
This script trains a model to identify items based on weight sensor patterns.
"""

import numpy as np
import pandas as pd
import argparse
import os
import pickle
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report
import onnxmltools

def load_data(input_file):
    """Load training data from CSV file."""
    print(f"Loading data from {input_file}")
    data = pd.read_csv(input_file)
    return data

def preprocess_data(data):
    """Preprocess the data for training."""
    print("Preprocessing data...")
    
    # Extract features (weight values) and labels (item class)
    X = data.iloc[:, 1:10].values  # 9 weight sensors
    y = data.iloc[:, 10].values    # item class
    
    # Split data into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )
    
    return X_train, X_test, y_train, y_test

def train_model(X_train, y_train):
    """Train the model on the data."""
    print("Training model...")
    
    # Create and train a Random Forest classifier
    model = RandomForestClassifier(
        n_estimators=100,
        max_depth=10,
        random_state=42
    )
    
    model.fit(X_train, y_train)
    
    return model

def evaluate_model(model, X_test, y_test):
    """Evaluate the model performance."""
    print("Evaluating model...")
    
    # Make predictions on the test set
    y_pred = model.predict(X_test)
    
    # Calculate accuracy
    accuracy = accuracy_score(y_test, y_pred)
    print(f"Accuracy: {accuracy:.4f}")
    
    # Print classification report
    print("Classification Report:")
    print(classification_report(y_test, y_pred))
    
    return accuracy

def convert_to_onnx(model, output_model):
    """Convert the model to ONNX format."""
    print(f"Converting model to ONNX and saving to {output_model}")
    
    # Create initial types for the ONNX model
    initial_types = [('input', onnxmltools.common.data_types.FloatTensorType([None, 9]))]
    
    # Convert the model to ONNX format
    onnx_model = onnxmltools.convert_sklearn(
        model, 
        initial_types=initial_types,
        target_opset=12
    )
    
    # Save the ONNX model
    onnxmltools.utils.save_model(onnx_model, output_model)

def create_sample_data(output_file):
    """Create sample training data if no input file is provided."""
    print(f"Creating sample training data and saving to {output_file}")
    
    # Define item classes
    classes = ['empty', 'small_box', 'medium_box', 'large_box', 'bottle', 'can']
    
    # Generate random samples
    n_samples = 1000
    data = []
    
    for _ in range(n_samples):
        # Randomly select a class
        item_class = np.random.choice(classes)
        
        # Generate weight patterns based on the class
        if item_class == 'empty':
            weights = np.random.normal(0.0, 0.05, 9)
            weights = np.maximum(0, weights)  # No negative weights
        elif item_class == 'small_box':
            weights = np.random.normal(0.3, 0.1, 9)
            weights = np.maximum(0, weights)
        elif item_class == 'medium_box':
            weights = np.random.normal(0.7, 0.15, 9)
            weights = np.maximum(0, weights)
        elif item_class == 'large_box':
            weights = np.random.normal(1.5, 0.2, 9)
            weights = np.maximum(0, weights)
        elif item_class == 'bottle':
            weights = np.zeros(9)
            # Bottles usually concentrated in one sensor
            center_idx = np.random.randint(0, 9)
            weights[center_idx] = np.random.normal(0.5, 0.1)
            weights = np.maximum(0, weights)
        else:  # can
            weights = np.zeros(9)
            # Cans are lighter
            center_idx = np.random.randint(0, 9)
            weights[center_idx] = np.random.normal(0.3, 0.05)
            weights = np.maximum(0, weights)
        
        # Add some noise
        weights += np.random.normal(0, 0.02, 9)
        weights = np.maximum(0, weights)
        
        # Add sample ID
        sample_id = _
        
        # Create sample
        sample = [sample_id] + weights.tolist() + [item_class]
        data.append(sample)
    
    # Create DataFrame
    columns = ['sample_id'] + [f'weight_{i+1}' for i in range(9)] + ['item_class']
    df = pd.DataFrame(data, columns=columns)
    
    # Save to CSV
    df.to_csv(output_file, index=False)
    
    return output_file

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Train a weight sensor ML model')
    parser.add_argument('--input-file', type=str, help='Path to input data CSV file')
    parser.add_argument('--output-model', type=str, default='weight_model.onnx', 
                        help='Path to save the trained model')
    parser.add_argument('--create-sample-data', action='store_true',
                        help='Create sample training data if no input file is provided')
    parser.add_argument('--sample-output', type=str, default='weight_data.csv',
                        help='Path to save the sample data if created')
    
    args = parser.parse_args()
    
    # Create sample data if requested
    if args.create_sample_data or not args.input_file:
        input_file = create_sample_data(args.sample_output)
    else:
        input_file = args.input_file
    
    # Load and preprocess the data
    data = load_data(input_file)
    X_train, X_test, y_train, y_test = preprocess_data(data)
    
    # Train the model
    model = train_model(X_train, y_train)
    
    # Evaluate the model
    evaluate_model(model, X_test, y_test)
    
    # Convert and save the model in ONNX format
    convert_to_onnx(model, args.output_model)
    
    print(f"Model training completed and saved to {args.output_model}")

if __name__ == '__main__':
    main()
EOL
    fi
    
    # Check if we need to create a non-existent launch file
    if [ ! -s "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/launch/control_only.launch.py" ]; then
        print_status "Creating control_only.launch.py"
        mkdir -p "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/launch"
        cat > "$REPO_ROOT/k3s-siwvs/ros2/siwvs_bringup/launch/control_only.launch.py" << 'EOL'
#!/usr/bin/env python3
"""
Launch file for SIWVS ROS 2 control nodes without Gazebo simulation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to packages
    siwvs_control_dir = get_package_share_directory('siwvs_control')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Weight sensor nodes
    weight_sensor_nodes = []
    
    # Create 3x3 grid of weight sensors (3 rows, 3 columns)
    for row in range(1, 4):
        for col in range(1, 4):
            sensor_id = f'sensor_{row}_{col}'
            weight_sensor_nodes.append(
                Node(
                    package='siwvs_control',
                    executable='weight_sensor_node.py',
                    name=f'weight_sensor_{row}_{col}',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'sensor_id': sensor_id,
                        'shelf_id': 'shelf_01',
                        'row': row,
                        'column': col,
                        'config_file': '/config/sensor_config.yaml'
                    }],
                    output='screen'
                )
            )
    
    # ROS Bridge server for web interface
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 9090
        }],
        output='screen'
    )
    
    # Create LaunchDescription
    ld = LaunchDescription()
    
    # Add launch configurations
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))
    
    # Add weight sensor nodes
    for node in weight_sensor_nodes:
        ld.add_action(node)
    
    # Add ROS Bridge
    ld.add_action(rosbridge_node)
    
    return ld
EOL
    fi
}

copy_model_files() {
    print_section "Copying model files to Gazebo directory"
    
    # Ensure Gazebo models directory exists
    mkdir -p "/home/$SUDO_USER/.gazebo/models"
    
    # Copy shelf model
    if [ -d "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf" ]; then
        print_status "Copying shelf model to Gazebo models directory"
        cp -r "$REPO_ROOT/k3s-siwvs/gazebo/models/shelf" "/home/$SUDO_USER/.gazebo/models/"
        chown -R $SUDO_USER:$SUDO_USER "/home/$SUDO_USER/.gazebo/models/shelf"
    fi
}

build_ros2_packages() {
    print_section "Building ROS 2 packages"
    
    # Source ROS 2 setup
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        print_status "Building using local ROS 2 installation"
        
        # Create the build script for the user to run
        cat > "$REPO_ROOT/build_ros2.sh" << 'EOL'
#!/bin/bash
cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/humble/setup.bash
cd k3s-siwvs/ros2
colcon build --symlink-install
echo "ROS 2 packages built successfully"
echo "Run the following to use them:"
echo "source k3s-siwvs/ros2/install/setup.bash"
EOL
        chmod +x "$REPO_ROOT/build_ros2.sh"
        
        print_status "Created build_ros2.sh script. Run it to build the ROS 2 packages."
    else
        print_warning "ROS 2 Humble not found, skipping local build"
        print_status "You will need to build packages in Docker or after installing ROS 2 Humble"
    fi
}

prepare_docker_build() {
    print_section "Setting up Docker build environment"
    
    # Check if Docker is installed and running
    if command -v docker &> /dev/null; then
        print_status "Docker is installed"
        
        # Check if Docker daemon is running
        if docker info &> /dev/null; then
            print_status "Docker daemon is running"
            
            # Create Docker build script
            cat > "$REPO_ROOT/build_docker.sh" << 'EOL'
#!/bin/bash
set -e

# Docker Build Script for SIWVS
REGISTRY=${1:-"localhost:5000"}
PUSH=${2:-"false"}

cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Building Docker images for SIWVS"
echo "Using registry: ${REGISTRY}"

# Build ROS 2 Control image
echo "Building ROS 2 Control image..."
docker build -t ${REGISTRY}/siwvs/ros2-control:latest -f k3s-siwvs/docker/ros2/Dockerfile.arm64 .

# Build Gazebo Simulation image
echo "Building Gazebo Simulation image..."
docker build -t ${REGISTRY}/siwvs/gazebo-simulation:latest -f k3s-siwvs/docker/gazebo/Dockerfile.arm64 .

# Build ML image
echo "Building ML image..."
docker build -t ${REGISTRY}/siwvs/weight-sensor-ml:latest -f k3s-siwvs/docker/ml/Dockerfile.arm64 .

# Push images if requested
if [ "$PUSH" = "true" ]; then
    echo "Pushing images to registry ${REGISTRY}..."
    docker push ${REGISTRY}/siwvs/ros2-control:latest
    docker push ${REGISTRY}/siwvs/gazebo-simulation:latest
    docker push ${REGISTRY}/siwvs/weight-sensor-ml:latest
fi

echo "Docker images built successfully"
EOL
            chmod +x "$REPO_ROOT/build_docker.sh"
            
            print_status "Created build_docker.sh script for building Docker images"
        else
            print_warning "Docker daemon is not running"
        fi
    else
        print_warning "Docker is not installed, skipping Docker setup"
    fi
}

setup_jetson_environment() {
    print_section "Setting up Jetson environment"
    
    if [ -f "/etc/nv_tegra_release" ]; then
        print_status "Detected Jetson platform"
        
        # Check if k3s is already installed
        if command -v k3s &> /dev/null; then
            print_status "k3s is already installed on this Jetson"
        else
            print_status "Installing prerequisites for Jetson Nano"
            
            # Only run this section if we have sudo privileges
            if [ "$(id -u)" -eq 0 ]; then
                apt-get update
                apt-get install -y --no-install-recommends \
                    curl \
                    jq \
                    open-iscsi \
                    nfs-common \
                    iptables
                
                # Create data directories
                mkdir -p /data/ml-models
                mkdir -p /data/ros2-data
                chmod 777 -R /data
                
                # Copy k3s installation script to make it executable
                cp "$REPO_ROOT/k3s-siwvs/setup/install.sh" /tmp/k3s-install.sh
                chmod +x /tmp/k3s-install.sh
                
                print_status "k3s installation script prepared at /tmp/k3s-install.sh"
                print_status "Run the script with sudo to install k3s on this Jetson"
            else
                print_warning "Not running as root, skipping k3s installation"
                print_status "Run the following as root to install k3s:"
                print_status "sudo $REPO_ROOT/k3s-siwvs/setup/install.sh"
            fi
        fi
        
        # Enable Docker support for NVIDIA container runtime
        if command -v docker &> /dev/null; then
            # Check if NVIDIA container runtime is installed
            if [ -f "/etc/docker/daemon.json" ]; then
                if grep -q "nvidia" "/etc/docker/daemon.json"; then
                    print_status "NVIDIA container runtime is already configured"
                else
                    print_warning "NVIDIA container runtime not configured in Docker"
                    if [ "$(id -u)" -eq 0 ]; then
                        # Backup existing daemon.json
                        cp /etc/docker/daemon.json /etc/docker/daemon.json.bak
                        
                        # Create new daemon.json with NVIDIA runtime
                        cat > /etc/docker/daemon.json << 'EOL'
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
EOL
                        systemctl restart docker
                        print_status "NVIDIA container runtime configured and Docker restarted"
                    else
                        print_warning "Not running as root, cannot configure NVIDIA container runtime"
                    fi
                fi
            else
                if [ "$(id -u)" -eq 0 ]; then
                    # Create daemon.json with NVIDIA runtime
                    mkdir -p /etc/docker
                    cat > /etc/docker/daemon.json << 'EOL'
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
EOL
                    if systemctl is-active docker &> /dev/null; then
                        systemctl restart docker
                    fi
                    print_status "NVIDIA container runtime configured"
                else
                    print_warning "Not running as root, cannot configure NVIDIA container runtime"
                fi
            fi
        fi
    else
        print_status "Not running on Jetson platform, skipping Jetson-specific setup"
    fi

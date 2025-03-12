#!/bin/bash
# k3s installation script for Jetson Nano with optimizations for AI/ML workloads

set -e

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root or with sudo"
    exit 1
fi

# System preparation
echo "Preparing system for k3s installation..."

# Update system
apt-get update
apt-get upgrade -y

# Install dependencies
apt-get install -y curl jq open-iscsi nfs-common

# Configure system for container workloads
echo "Configuring system for containerized workloads..."

# Configure kernel parameters for Kubernetes
cat > /etc/sysctl.d/99-kubernetes.conf << EOL
net.bridge.bridge-nf-call-iptables = 1
net.bridge.bridge-nf-call-ip6tables = 1
net.ipv4.ip_forward = 1
vm.swappiness = 0
vm.overcommit_memory = 1
kernel.panic = 10
kernel.panic_on_oops = 1
EOL

# Apply sysctl parameters
sysctl --system

# Configure memory and GPU settings for Jetson
if [ -f /etc/tegra-release ]; then
    echo "Detected Jetson platform, configuring GPU settings..."
    
    # Enable JetPack GPIO settings
    systemctl enable nvargus-daemon
    
    # Configure Jetson power mode for better performance
    nvpmodel -m 0
    jetson_clocks
    
    # Allocate more memory to GPU for ML workloads
    echo "vm.dirty_background_ratio = 5" >> /etc/sysctl.conf
    echo "vm.dirty_ratio = 10" >> /etc/sysctl.conf
    sysctl -p
fi

# Create k3s configuration directory
mkdir -p /etc/rancher/k3s

# Copy configuration file (should be adjacent to this script)
if [ -f ./config.yaml ]; then
    cp ./config.yaml /etc/rancher/k3s/config.yaml
else
    echo "Warning: config.yaml not found, using default configuration"
    
    # Create basic configuration
    cat > /etc/rancher/k3s/config.yaml << EOL
# k3s configuration for Jetson Nano with ML workloads
write-kubeconfig-mode: "0644"
disable:
  - traefik
  - servicelb
  - metrics-server
  - local-storage
  - coredns
kube-controller-manager-arg:
  - "node-monitor-period=60s"
  - "node-monitor-grace-period=150s"
kubelet-arg:
  - "image-gc-high-threshold=50"
  - "image-gc-low-threshold=40"
  - "eviction-hard=memory.available<500Mi,nodefs.available<10%"
  - "kube-reserved=cpu=100m,memory=100Mi,ephemeral-storage=1Gi"
  - "system-reserved=cpu=100m,memory=200Mi,ephemeral-storage=1Gi"
EOL
fi

# Install k3s - server mode for single-node deployment
echo "Installing k3s server..."
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="server" sh -

# Wait for k3s to be ready
echo "Waiting for k3s to start..."
sleep 30

# Install Helm for additional components
echo "Installing Helm..."
curl https://raw.githubusercontent.com/helm/helm/master/scripts/get-helm-3 | bash

# Configure kubectl
export KUBECONFIG=/etc/rancher/k3s/k3s.yaml
mkdir -p $HOME/.kube
cp /etc/rancher/k3s/k3s.yaml $HOME/.kube/config
chmod 600 $HOME/.kube/config

# Apply storage configuration
if [ -f ./storage-setup.yaml ]; then
    echo "Setting up storage configuration..."
    kubectl apply -f ./storage-setup.yaml
else
    echo "No storage configuration found, skipping storage setup"
fi

# Install custom container registries configuration
if [ -f ./registries.yaml ]; then
    echo "Setting up container registries..."
    cp ./registries.yaml /etc/rancher/k3s/
    systemctl restart k3s
fi

# Output information about the installed k3s
echo "k3s installed successfully!"
echo "k3s version: $(k3s --version)"
echo "Kubernetes version: $(kubectl version --short)"
echo "Node status:"
kubectl get nodes

echo "Installation complete! Your k3s cluster is ready for AI/ML workloads."
echo "Use kubectl to manage your cluster."

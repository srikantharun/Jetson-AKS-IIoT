variable "resource_group_name" {
  description = "Name of the resource group"
  type        = string
  default     = "jetson-aks-rg"
}

variable "location" {
  description = "Azure region to deploy resources"
  type        = string
  default     = "eastus"
}

variable "cluster_name" {
  description = "Name of the AKS cluster"
  type        = string
  default     = "jetson-aks-cluster"
}

variable "acr_name" {
  description = "Name of the Azure Container Registry"
  type        = string
  default     = "jetsonaksacr"
}

variable "environment" {
  description = "Environment (dev, test, prod)"
  type        = string
  default     = "dev"
}

variable "kubernetes_version" {
  description = "Kubernetes version"
  type        = string
  default     = "1.27.3"
}

# Jetson Nano comparable settings
variable "arm_node_count" {
  description = "Number of ARM nodes in the cluster"
  type        = number
  default     = 2
}

variable "arm_vm_size" {
  description = "VM size for ARM nodes (comparable to Jetson Nano specs)"
  type        = string
  default     = "Standard_D4ps_v5" # 4 vCPUs, 16GB memory, ARM64
}

variable "arm_os_disk_size_gb" {
  description = "OS disk size for ARM nodes (in GB)"
  type        = number
  default     = 32 # Comparable to Jetson Nano 32GB
}

variable "arm_max_pods" {
  description = "Maximum number of pods per node"
  type        = number
  default     = 50
}

# Network settings
variable "network_plugin" {
  description = "Network plugin for AKS cluster"
  type        = string
  default     = "kubenet"
}

variable "network_policy" {
  description = "Network policy for AKS cluster"
  type        = string
  default     = "calico"
}

# Create ARM64 node pool (comparable to Jetson Nano)
resource "azurerm_kubernetes_cluster_node_pool" "arm64" {
  name                  = "arm64pool"
  kubernetes_cluster_id = azurerm_kubernetes_cluster.aks.id
  vm_size               = var.arm_vm_size
  node_count            = var.arm_node_count
  
  # Set resource constraints similar to Jetson Nano
  os_disk_size_gb       = var.arm_os_disk_size_gb
  max_pods              = var.arm_max_pods
  
  # Enable auto-scaling
  enable_auto_scaling   = true
  min_count             = 1
  max_count             = 4
  
  # Use ARM64 architecture
  orchestrator_version  = var.kubernetes_version
  os_type               = "Linux"
  os_sku                = "Ubuntu"
  
  # Use spot instances to reduce costs (optional)
  # priority              = "Spot"
  # eviction_policy       = "Delete"
  # spot_max_price        = -1 # default to on-demand price
  
  node_labels = {
    "nodepool-type"     = "user"
    "environment"       = var.environment
    "nodepoolos"        = "linux"
    "app"               = "jetson-workload"
    "kubernetes.azure.com/scalesetpriority" = "regular"
    "architecture"      = "arm64"
  }
  
  node_taints = [
    "architecture=arm64:NoSchedule"
  ]
  
  tags = {
    Environment = var.environment
    NodeType    = "arm64"
    Project     = "Jetson-AKS-IIoT"
  }
  
  # Prevent parallel creation with cluster
  depends_on = [
    azurerm_kubernetes_cluster.aks
  ]
}

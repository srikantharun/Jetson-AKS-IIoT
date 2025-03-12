# Create AKS cluster optimized for Jetson Nano compatibility
resource "azurerm_kubernetes_cluster" "aks" {
  name                = var.cluster_name
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name
  dns_prefix          = var.cluster_name
  kubernetes_version  = var.kubernetes_version
  
  # Default node pool (system nodes, x86)
  default_node_pool {
    name                = "systempool"
    node_count          = 1
    vm_size             = "Standard_D2s_v3"
    os_disk_size_gb     = 30
    type                = "VirtualMachineScaleSets"
    enable_auto_scaling = true
    min_count           = 1
    max_count           = 3
    max_pods            = 30
    vnet_subnet_id      = null # Use default subnet
    
    node_labels = {
      "nodepool-type" = "system"
      "environment"   = var.environment
      "nodepoolos"    = "linux"
      "app"           = "system-apps"
    }
    
    tags = {
      Environment = var.environment
      NodeType    = "system"
    }
  }
  
  # Use managed identity
  identity {
    type = "SystemAssigned"
  }
  
  # Enable monitoring
  oms_agent {
    log_analytics_workspace_id = azurerm_log_analytics_workspace.aks.id
  }
  
  # Network configuration
  network_profile {
    network_plugin     = var.network_plugin
    network_policy     = var.network_policy
    load_balancer_sku  = "standard"
    outbound_type      = "loadBalancer"
    docker_bridge_cidr = "172.17.0.1/16"
    service_cidr       = "10.0.0.0/16"
    dns_service_ip     = "10.0.0.10"
  }
  
  # Enable RBAC
  azure_active_directory_role_based_access_control {
    managed = true
    azure_rbac_enabled = true
  }
  
  # Enable Auto-upgrade channel
  automatic_channel_upgrade = "stable"
  
  # Add container registry access
  # This creates a role assignment for AKS to access ACR
  depends_on = [
    azurerm_container_registry.acr
  ]
  
  tags = {
    Environment = var.environment
    Project     = "Jetson-AKS-IIoT"
  }
}

# Create role assignment for AKS to pull images from ACR
resource "azurerm_role_assignment" "aks_acr" {
  principal_id                     = azurerm_kubernetes_cluster.aks.kubelet_identity[0].object_id
  role_definition_name             = "AcrPull"
  scope                            = azurerm_container_registry.acr.id
  skip_service_principal_aad_check = true
}

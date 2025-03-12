# Create storage account for persistent data
resource "azurerm_storage_account" "aks_storage" {
  name                     = "${replace(var.cluster_name, "-", "")}sa"
  resource_group_name      = azurerm_resource_group.rg.name
  location                 = azurerm_resource_group.rg.location
  account_tier             = "Standard"
  account_replication_type = "LRS"
  
  # Enable blob and file storage
  blob_properties {
    versioning_enabled = true
    
    delete_retention_policy {
      days = 7
    }
  }
  
  # Enable file shares
  share_properties {
    retention_policy {
      days = 7
    }
  }
  
  tags = {
    Environment = var.environment
    Project     = "Jetson-AKS-IIoT"
  }
}

# Create file share for persistent storage (similar to what you'd use on Jetson)
resource "azurerm_storage_share" "aks_share" {
  name                 = "jetsonshare"
  storage_account_name = azurerm_storage_account.aks_storage.name
  quota                = 10 # 10 GB, similar to what you might allocate on Jetson
}

# Create Azure Managed Disk for simulating Jetson storage constraints
resource "azurerm_managed_disk" "aks_disk" {
  name                 = "${var.cluster_name}-managed-disk"
  location             = azurerm_resource_group.rg.location
  resource_group_name  = azurerm_resource_group.rg.name
  storage_account_type = "StandardSSD_LRS"
  create_option        = "Empty"
  disk_size_gb         = 32 # 32GB, similar to Jetson Nano
  
  tags = {
    Environment = var.environment
    Project     = "Jetson-AKS-IIoT"
  }
}

# Terraform for AKS Jetson-Compatible Environment

This directory contains Terraform scripts to provision an Azure Kubernetes Service (AKS) cluster that is designed to be compatible with Jetson Nano deployments.

## Features

- ARM64 node pool that mimics Jetson Nano specifications
- Resource constraints aligned with edge deployment
- Storage configuration similar to what would be used on Jetson
- Network settings optimized for container communication

## Usage

1. Initialize Terraform:
   ```bash
   terraform init
   ```

2. Apply the configuration:
   ```bash
   terraform apply
   ```

3. Connect to the cluster:
   ```bash
   az aks get-credentials --resource-group $(terraform output -raw resource_group_name) --name $(terraform output -raw kubernetes_cluster_name)
   ```

#!/bin/bash
# Script to set up and manage the Terraform environment for AKS deployment
# for Jetson-AKS-IIoT project

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TERRAFORM_DIR="${SCRIPT_DIR}"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to display help message
function show_help {
    echo -e "Usage: ${YELLOW}$0 [OPTIONS]${NC}"
    echo -e "Setup and manage Terraform environment for Jetson-AKS-IIoT project."
    echo
    echo -e "Options:"
    echo -e "  ${GREEN}-h, --help${NC}       Show this help message"
    echo -e "  ${GREEN}-c, --clean${NC}      Clean existing Terraform setup and start fresh"
    echo -e "  ${GREEN}-i, --init${NC}       Initialize Terraform environment"
    echo -e "  ${GREEN}-a, --apply${NC}      Apply Terraform configuration"
    echo -e "  ${GREEN}-d, --destroy${NC}    Destroy Terraform-managed infrastructure"
    echo -e "  ${GREEN}-v, --validate${NC}   Validate Terraform configuration"
    echo
}

# Function to check if Azure CLI is installed
function check_prerequisites {
    echo -e "${YELLOW}Checking prerequisites...${NC}"

    # Check if Azure CLI is installed
    if ! command -v az &> /dev/null; then
        echo -e "${RED}Azure CLI could not be found. Please install it first:${NC}"
        echo -e "See: https://docs.microsoft.com/en-us/cli/azure/install-azure-cli"
        exit 1
    fi

    # Check if Terraform is installed
    if ! command -v terraform &> /dev/null; then
        echo -e "${RED}Terraform could not be found. Please install it first:${NC}"
        echo -e "See: https://learn.hashicorp.com/tutorials/terraform/install-cli"
        exit 1
    fi

    # Check if user is logged in to Azure
    if ! az account show &> /dev/null; then
        echo -e "${YELLOW}You are not logged in to Azure. Attempting to log in...${NC}"
        az login
    fi

    echo -e "${GREEN}All prerequisites met.${NC}"
}

# Function to clean terraform directory
function clean_terraform {
    echo -e "${YELLOW}Cleaning Terraform environment...${NC}"
    
    # Ask for confirmation
    read -p "This will remove all Terraform state files. Are you sure? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Clean operation canceled.${NC}"
        return
    fi
    
    # Delete .terraform directory and local state files
    rm -rf "${TERRAFORM_DIR}/.terraform" "${TERRAFORM_DIR}/.terraform.lock.hcl" "${TERRAFORM_DIR}/terraform.tfstate" "${TERRAFORM_DIR}/terraform.tfstate.backup"
    
    echo -e "${GREEN}Terraform environment cleaned.${NC}"
}

# Function to initialize Terraform
function init_terraform {
    echo -e "${YELLOW}Initializing Terraform...${NC}"
    cd "${TERRAFORM_DIR}"
    
    # Create storage account for Terraform state if it doesn't exist
    echo -e "${YELLOW}Checking if Terraform state storage account exists...${NC}"
    
    if ! az group show --name "tf-state-rg" &> /dev/null; then
        echo -e "${YELLOW}Creating resource group for Terraform state...${NC}"
        az group create --name "tf-state-rg" --location "eastus"
    fi
    
    if ! az storage account show --name "tfstatejetson" --resource-group "tf-state-rg" &> /dev/null; then
        echo -e "${YELLOW}Creating storage account for Terraform state...${NC}"
        az storage account create --name "tfstatejetson" --resource-group "tf-state-rg" --location "eastus" --sku "Standard_LRS"
        
        # Create storage container
        echo -e "${YELLOW}Creating storage container for Terraform state...${NC}"
        az storage container create --name "tfstate" --account-name "tfstatejetson"
    fi
    
    # Initialize Terraform
    terraform init
    
    echo -e "${GREEN}Terraform initialized successfully.${NC}"
}

# Function to validate Terraform configuration
function validate_terraform {
    echo -e "${YELLOW}Validating Terraform configuration...${NC}"
    cd "${TERRAFORM_DIR}"
    terraform validate
    echo -e "${GREEN}Terraform configuration validated.${NC}"
}

# Function to apply Terraform configuration
function apply_terraform {
    echo -e "${YELLOW}Applying Terraform configuration...${NC}"
    cd "${TERRAFORM_DIR}"
    terraform plan -out=tfplan
    terraform apply tfplan
    
    echo -e "${GREEN}Terraform configuration applied successfully.${NC}"
    echo -e "${YELLOW}Getting AKS credentials...${NC}"
    
    # Get AKS credentials
    RESOURCE_GROUP=$(terraform output -raw resource_group_name)
    CLUSTER_NAME=$(terraform output -raw kubernetes_cluster_name)
    
    az aks get-credentials --resource-group "${RESOURCE_GROUP}" --name "${CLUSTER_NAME}" --overwrite-existing
    
    echo -e "${GREEN}AKS credentials fetched successfully.${NC}"
    echo -e "${YELLOW}ACR Login Server: $(terraform output -raw acr_login_server)${NC}"
}

# Function to destroy Terraform infrastructure
function destroy_terraform {
    echo -e "${YELLOW}Destroying Terraform infrastructure...${NC}"
    
    # Ask for confirmation
    read -p "This will destroy all resources created by Terraform. Are you sure? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Destroy operation canceled.${NC}"
        return
    fi
    
    cd "${TERRAFORM_DIR}"
    terraform destroy -auto-approve
    
    echo -e "${GREEN}Terraform infrastructure destroyed successfully.${NC}"
}

# Parse command line arguments
if [[ $# -eq 0 ]]; then
    show_help
    exit 0
fi

# Process options
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_help
            exit 0
            ;;
        -c|--clean)
            check_prerequisites
            clean_terraform
            shift
            ;;
        -i|--init)
            check_prerequisites
            init_terraform
            shift
            ;;
        -a|--apply)
            check_prerequisites
            apply_terraform
            shift
            ;;
        -d|--destroy)
            check_prerequisites
            destroy_terraform
            shift
            ;;
        -v|--validate)
            check_prerequisites
            validate_terraform
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

exit 0

.PHONY: all clean build deploy terraform-apply terraform-destroy docker-build

# Default target
all: build

# Clean build artifacts
clean:
	rm -rf build/ install/ log/
	cd terraform && terraform destroy -auto-approve

# Build ROS 2 packages
build:
	cd k3s-siwvs/ros2 && \
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install

# Build and push Docker images
docker-build:
	cd k3s-siwvs/docker/ros2 && docker buildx build --platform linux/arm64 -t $(REGISTRY)/siwvs/ros2-control:latest -f Dockerfile.arm64 --push .
	cd k3s-siwvs/docker/gazebo && docker buildx build --platform linux/arm64 -t $(REGISTRY)/siwvs/gazebo-simulation:latest -f Dockerfile.arm64 --push .
	cd k3s-siwvs/docker/ml && docker buildx build --platform linux/arm64 -t $(REGISTRY)/siwvs/weight-sensor-ml:latest -f Dockerfile.arm64 --push .

# Deploy infrastructure with Terraform
terraform-apply:
	cd terraform && terraform init && terraform apply -auto-approve

# Destroy infrastructure with Terraform
terraform-destroy:
	cd terraform && terraform destroy -auto-approve

# Deploy to k3s
deploy:
	export ACR_NAME=$(REGISTRY) && \
	envsubst < k3s-siwvs/kubernetes/deployments/siwvs-deployment.yaml | kubectl apply -f - && \
	kubectl apply -f k3s-siwvs/kubernetes/namespaces.yaml && \
	kubectl apply -f k3s-siwvs/kubernetes/storage-classes/ && \
	kubectl apply -f k3s-siwvs/kubernetes/services/

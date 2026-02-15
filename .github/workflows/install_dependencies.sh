#!/bin/bash
set -e

# Install basic build tools
sudo apt-get update
sudo apt-get install -y cmake build-essential gcc-14 g++-14 yq wget gnupg libeigen3-dev libopencv-dev

# Install OpenVINO
wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo gpg --dearmor --output /usr/share/keyrings/intel-openvino-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/intel-openvino-archive-keyring.gpg] https://apt.repos.intel.com/openvino ubuntu24 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt-get update
sudo apt-get install -y --no-install-recommends openvino-2025.2.0
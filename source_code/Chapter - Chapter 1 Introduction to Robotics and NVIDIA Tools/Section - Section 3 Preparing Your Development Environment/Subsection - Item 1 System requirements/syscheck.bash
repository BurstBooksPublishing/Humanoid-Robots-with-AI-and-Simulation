#!/usr/bin/env bash
# Check GPU presence and memory
if ! command -v nvidia-smi >/dev/null; then
  echo "nvidia-smi not found: install NVIDIA driver" >&2; exit 1
fi
echo "GPU summary:" 
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader

# Check CUDA (nvcc) if present
if command -v nvcc >/dev/null; then
  echo "nvcc version:"; nvcc --version
else
  echo "nvcc not found; verify CUDA toolkit installation" >&2
fi

# Check Docker + NVIDIA container tool
if command -v docker >/dev/null; then
  echo "Docker found"; docker --version
  # quick GPU test via container (requires network)
  docker run --rm --gpus all nvidia/cuda:11.8-base nvidia-smi || \
    echo "Docker GPU smoke test failed"
else
  echo "Docker not installed"
fi

# Check ROS2 CLI presence
if command -v ros2 >/dev/null; then
  echo "ROS2 CLI present:"; ros2 --version
else
  echo "ROS2 not detected; install ROS2 matching Ubuntu LTS"
fi

# Basic Python check
python3 -c "import sys; print('Python', sys.version.split()[0])" || true
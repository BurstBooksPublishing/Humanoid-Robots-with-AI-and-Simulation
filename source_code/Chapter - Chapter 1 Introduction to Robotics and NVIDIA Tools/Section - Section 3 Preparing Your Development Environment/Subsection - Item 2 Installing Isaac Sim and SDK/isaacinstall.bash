#!/usr/bin/env bash
set -e
# check GPU and driver
nvidia-smi || { echo "NVIDIA driver missing"; exit 1; }

# create conda env for reproducibility
conda create -y -n humanoid-sim python=3.9  # Isaac Sim often targets specific Python versions
conda activate humanoid-sim

# install minimal Python tooling for headless scripts (not the full app)
pip install --upgrade pip setuptools wheel

# optional: install ROS2 bridge client libs (example, replace with target ROS distro)
# pip install ros2cli                         # small helper tools for ROS integration

# if using the headless Isaac Sim archive, set this to extracted path
ISAAC_SIM_DIR=/opt/isaac-sim  # <-- update to your extracted installer path

if [ -d "$ISAAC_SIM_DIR" ]; then
  # launch headless simulation with a robot USD and user script
  "$ISAAC_SIM_DIR/isaac-sim.sh" --headless -- /path/to/your/humanoid_scene.usd \
    -r /path/to/robot_controller.py   # run a Python controller in the simulation
else
  echo "Isaac Sim directory not found. Use Omniverse Launcher or download archive."
  exit 2
fi
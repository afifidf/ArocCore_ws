#  AROC 2026 ROS2 WORKSPACE

A core workspace for aroc team based on robotis-op3 code.

## GETTING STARTED

### Should Be There First
- Ubuntu 24.04
- ROS2 (Jazzy)

### How to Build

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Clone repository
git clone https://github.com/afifidf/ArocCore_ws.git ~/aroc3
cd aroc3

# Install ROS2 depedencies
rosdep install --from-paths src --ignore-src -r -y

# Build and source workspace
colcon build
source install/setup.bash

#OPTIONAL for more faster build
#build for enough ram (8gb or more)
colcon build --parallel-workers 4

#4gb or less
colcon build --paraller-workers 2
```
## YOLO ENV

### Follow This

```bash
#intel booster
sudo apt install intel-opencl-icd

#build the virtual environment
sudo apt install python3.12-venv
python3 -m venv ~/yolo
source ~/yolo/bin/activate
pip install --upgrade pip

#install yolo and friends
pip install -r requirments.txt

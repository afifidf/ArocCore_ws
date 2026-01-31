#  AROC 2026 ROS2 WORKSPACE

A core workspace for aroc team based on robotis-op3 code.

## GETTING STARTED

### Should Be There First
- Ubuntu 24.04
- ROS2 (Jazzy)

### How to Build

```bash
# Clone repository
git clone <repository-url>
cd study_ws

# Install Python dependencies
pip install -r requirements.txt

# Install ROS2 dependencies
cd 03_ros2_integration
rosdep install --from-paths src --ignore-src -r -y
```

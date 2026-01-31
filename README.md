#  AROC 2026 ROS2 WORKSPACE

A core workspace for aroc team based on robotis-op3 code.

## GETTING STARTED

## 🔧 Requirements

### Software yang Dibutuhkan
- Python 3.8+
- ROS2 (Jazzy)
- OpenCV
- Ultralytics (YOLOv8)

### Install Dependencies

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

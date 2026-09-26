# ROS2-Based-6-DOF

A ROS 2 computer-vision pipeline for estimating the **6-DOF pose of a planar reference object** from a camera image.

The project combines:

- **ROS 2** for image transport and pose publishing
- **OpenCV SIFT** for feature detection and matching
- **RANSAC homography estimation** for locating the reference object in the scene
- **Perspective geometry + `solvePnPRansac`** for 3D pose estimation
- **YOLO** for object detection / confidence filtering
- **Quaternion conversion** for representing orientation
- **TF2** for broadcasting the estimated camera transform
- **RViz-compatible ROS messages** for visualization and downstream robotics applications

The resulting pose contains **three position components** and **three rotational degrees of freedom**, giving a full 6-DOF camera/object pose estimate.

---

## Overview

The system starts with an image or video stream and attempts to determine where a known planar object is located relative to the camera.

The core pipeline is:

```text
Reference Image + Scene Image
              │
              ▼
        SIFT Feature Detection
              │
              ▼
        SIFT Feature Matching
              │
              ▼
          Ratio Test
              │
              ▼
      RANSAC Homography
              │
              ▼
   Project Reference Corners
              │
              ▼
        solvePnPRansac
              │
              ▼
       Rotation + Translation
              │
              ▼
        Quaternion Pose
              │
              ▼
             ROS 2
        ┌─────┼──────┐
        ▼     ▼      ▼
   Detection3D  Pose  TF
      Array    Msg   Transform
```

The project is designed as a perception component that could be integrated into a larger robotics system where the camera needs to estimate its position and orientation relative to a known planar target.

---

# 🎯 What Is 6-DOF Pose?

A 6-DOF pose describes both the **position** and **orientation** of an object.

### Position — 3 DOF

```text
X → left / right
Y → up / down
Z → forward / backward
```

### Orientation — 3 DOF

```text
Roll  → rotation around X
Pitch → rotation around Y
Yaw   → rotation around Z
```

Together:

```text
6 DOF = 3 position + 3 orientation
```

The project estimates the translation vector:

```text
t = [x, y, z]
```

and the rotation vector:

```text
r = [rx, ry, rz]
```

The rotation vector is then converted into a quaternion before being published through ROS 2.

---

# 🧠 How the Pose Estimation Works

## 1. Reference Image

The system uses a known reference image:

```text
Scene.png
```

This image represents the planar target whose pose should be estimated.

The reference image is converted to grayscale before feature extraction.

---

## 2. SIFT Feature Detection

The system uses OpenCV's SIFT implementation:

```python
sift = cv2.SIFT_create()
```

SIFT detects distinctive keypoints and computes descriptors for both:

- the reference image
- the current camera frame

Conceptually:

```text
Reference Image                Scene Image
      │                              │
      ▼                              ▼
 SIFT Keypoints                  SIFT Keypoints
      │                              │
      ▼                              ▼
  Descriptors                    Descriptors
```

SIFT is useful here because the reference object may appear at different positions, scales, and orientations in the camera image.

---

## 3. Feature Matching

The project uses a brute-force matcher with the L2 distance:

```python
bf = cv2.BFMatcher(cv2.NORM_L2)
```

The descriptors are matched using:

```python
matches = bf.knnMatch(
    descriptorsRef,
    descriptorsScene,
    k=2
)
```

For each descriptor, the two nearest matches are obtained.

---

## 4. Lowe's Ratio Test

The project applies a ratio test to remove weak or ambiguous matches:

```python
goodMatches = [
    m for m, n in matches
    if m.distance < 0.75 * n.distance
]
```

This means a match is accepted only when the best match is sufficiently better than the second-best match.

At least four good matches are required:

```python
if len(goodMatches) < 4:
    return "Error: Not enough matches to compute homography.", None
```

Four point correspondences are the minimum needed to estimate a planar homography.

---

# 🔄 5. Homography Estimation

The coordinates of the matched keypoints are extracted from both images.

The project then computes a homography using RANSAC:

```python
M, mask = cv2.findHomography(
    ptsRef,
    ptsScene,
    cv2.RANSAC,
    5.0
)
```

The homography maps points from the reference-image plane into the corresponding locations in the scene image.

Conceptually:

```text
Reference Plane
      │
      │ Homography
      ▼
Camera Image
```

RANSAC helps reject incorrect feature matches, which is important because SIFT matching can contain outliers.

---

# 📐 6. Projecting the Reference Corners

Once the homography is known, the four corners of the reference image are projected into the scene.

```python
projected_points = cv2.perspectiveTransform(
    pts,
    M
)
```

This gives four 2D image coordinates corresponding to the corners of the known planar target.

The project therefore obtains:

```text
Reference Image Coordinates
          ↓
      Homography
          ↓
Scene Image Corner Coordinates
```

---

# 📏 7. Defining the 3D Target

The target is modeled as a square planar object with a side length of:

```text
0.6096 m
```

The four 3D points are:

```python
posterPoints3D = np.array([
    [0,      0,      0],
    [0.6096, 0,      0],
    [0.6096, 0.6096, 0],
    [0,      0.6096,  0]
], dtype=np.float32)
```

Because all four points lie on the same plane:

```text
Z = 0
```

The known physical dimensions allow the system to recover metric translation rather than only estimating a relative image transformation.

---

# 📷 8. Camera Calibration

Pose estimation requires camera intrinsics and lens distortion parameters.

The project currently uses:

```python
camera_mtx = np.array([
    [4.2854390e+03, 0.0000000e+00, 2.8507354e+03],
    [0.0000000e+00, 4.2845395e+03, 2.1335531e+03],
    [0.0000000e+00, 0.0000000e+00, 1.0000000e+00]
])
```

and:

```python
camera_dist = np.array([
    3.0210000e-01,
    -1.1210000e+00,
    0.0000000e+00,
    0.0000000e+00,
    0.0000000e+00
])
```

These values are specific to the camera calibration used during development.

**They should be replaced with calibration values from the camera actually being used.**

---

# 🧮 9. PnP Pose Estimation

The projected 2D points and known 3D points are passed into:

```python
cv2.solvePnPRansac(...)
```

using:

```python
cv2.SOLVEPNP_IPPE_SQUARE
```

This estimates:

```text
rvec → rotation
tvec → translation
```

The rotation and translation describe the camera/object relationship.

The translation vector is also used to calculate the Euclidean distance:

```python
distance_m = np.linalg.norm(tvec)
```

The pose is subsequently refined using:

```python
cv2.solvePnPRefineVVS(...)
```

This provides a refinement step after the initial PnP solution.

---

# 🔢 10. Quaternion Conversion

ROS 2 commonly represents orientation using quaternions.

The estimated rotation vector is converted using:

```python
q = quat.from_rotation_vector(
    self.rvec.flatten()
)
```

The quaternion components are then extracted and placed into ROS messages:

```text
w
x
y
z
```

This allows the estimated orientation to be published through ROS 2.

---

# 🤖 ROS 2 Architecture

The project contains two ROS 2 nodes:

```text
image_publisher.py
        │
        │ /camera/image_raw
        ▼
image_subscriber.py
        │
        ├── YOLO
        ├── SIFT
        ├── Homography
        ├── PnP
        └── Pose
             │
             ├── /detections_3d
             ├── /pose_stamped
             ├── /pose_maker
             ├── /camera_info
             └── TF: poster → camera
```

---

# 📡 `image_publisher.py`

`image_publisher.py` creates a ROS 2 node named:

```text
pose_publisher
```

It publishes images to:

```text
/camera/image_raw
```

The video is read using OpenCV:

```python
cv2.VideoCapture(video_path)
```

A timer runs every:

```text
0.1 seconds
```

which corresponds to approximately:

```text
10 Hz
```

Each frame is converted to a ROS 2 `sensor_msgs/Image` using `CvBridge`.

If the video reaches the end, the publisher resets the video position back to the first frame and continues looping.

### Published Topic

```text
/camera/image_raw
```

Message type:

```text
sensor_msgs/msg/Image
```

---

# 📥 `image_subscriber.py`

`image_subscriber.py` creates a ROS 2 node named:

```text
pose_subscriber
```

It subscribes to:

```text
/camera/image_raw
```

For every received image, it:

1. Converts the ROS image into an OpenCV image
2. Runs the YOLO model
3. Estimates the planar target pose
4. Selects the highest-confidence YOLO detection
5. Rejects detections below a confidence threshold
6. Converts the pose orientation into a quaternion
7. Publishes ROS pose messages
8. Publishes a visualization marker
9. Broadcasts a TF transform
10. Publishes camera calibration information

---

# 🧠 YOLO Integration

The subscriber loads a YOLO model from the ROS package share directory:

```python
model_path = os.path.join(
    get_package_share_directory('perception_projects'),
    'rs25_8_15_2.pt'
)

self.model = YOLO(model_path)
```

The model is run with:

```python
results = self.model.predict(
    scene_img,
    save=True,
    imgsz=320,
    conf=0.25
)
```

The system then searches for the highest-confidence detection.

A second confidence threshold is applied:

```python
threshold = 0.9
```

Therefore, the node only continues publishing the pose when the best detection has a confidence of at least:

```text
0.90
```

The YOLO result is used as a confidence gate while the geometric pose estimation is performed by the SIFT/homography/PnP pipeline.

---

# 📤 ROS 2 Outputs

## `/detections_3d`

Message type:

```text
vision_msgs/msg/Detection3DArray
```

The estimated translation and orientation are stored in the detection's pose.

The message includes:

```text
Position:
    x
    y
    z

Orientation:
    x
    y
    z
    w
```

The detected object's class name and YOLO confidence are also included.

---

## `/pose_stamped`

Message type:

```text
geometry_msgs/msg/PoseStamped
```

The estimated position and orientation are published as a standard ROS pose.

The current frame ID is:

```text
poster
```

---

## `/pose_maker`

Message type:

```text
visualization_msgs/msg/Marker
```

The node publishes an arrow marker that can be visualized in RViz.

The marker is useful for visualizing the estimated pose and orientation.

---

## `/camera_info`

Message type:

```text
sensor_msgs/msg/CameraInfo
```

The node publishes the camera calibration parameters used by the pose-estimation pipeline.

---

# 🔗 TF Broadcast

The subscriber also broadcasts a transform:

```text
parent frame:
poster

child frame:
camera
```

Conceptually:

```text
poster
  │
  │ estimated transform
  ▼
camera
```

The transform contains:

```text
Translation:
    x
    y
    z

Rotation:
    quaternion
```

This makes the pose available to other ROS 2 components through the TF2 system.

---

# 📁 Repository Structure

The repository currently contains three Python modules:

```text
ROS2-Based-6-DOF/
│
├── image_publisher.py
├── image_subscriber.py
└── pose_estimation_v2.py
```

## `image_publisher.py`

Reads a video with OpenCV and publishes frames to:

```text
/camera/image_raw
```

as ROS 2 image messages.

## `image_subscriber.py`

Main ROS 2 perception node.

It combines:

- ROS 2 subscriptions/publications
- OpenCV
- YOLO
- SIFT-based pose estimation
- PnP
- quaternion conversion
- TF2
- RViz visualization

## `pose_estimation_v2.py`

Contains the main geometric pose-estimation function:

```python
estimate_pose(ref_path, scene_img)
```

The function returns either an error or a dictionary containing:

```text
rvec
tvec
homography
distance_m
```

---

# ⚙️ Dependencies

The project uses the following major Python libraries:

```text
rclpy
opencv-python
cv_bridge
numpy
scipy / quaternion
ultralytics
vision_msgs
geometry_msgs
sensor_msgs
visualization_msgs
tf2_ros
ament_index_python
```

A ROS 2 installation is also required.

The code uses standard ROS 2 message packages including:

```text
sensor_msgs
vision_msgs
geometry_msgs
visualization_msgs
```

---

# 🛠️ Setup

## 1. Install ROS 2

Install a ROS 2 distribution compatible with the system on which you intend to run the project.

Then source ROS 2:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
```

Replace `<your_ros_distro>` with your installed distribution.

---

## 2. Install Python Dependencies

Install the Python packages used by the project.

For example:

```bash
pip install opencv-python numpy ultralytics numpy-quaternion
```

ROS-specific packages should be installed through the ROS package manager for your distribution when appropriate.

---

## 3. Configure File Paths

The current code contains development-machine-specific absolute paths.

For example, `image_publisher.py` contains a video path similar to:

```text
/home/tarun2006/ros2_ws/src/perception_projects/perception_projects/Eren.MOV
```

The subscriber also references:

```text
/home/tarun2006/ros2_ws/src/perception_projects/perception_projects/Scene.png
```

These paths **must be changed** for another machine.

The YOLO model is loaded using:

```python
get_package_share_directory('perception_projects')
```

so the model must exist in the corresponding package share directory.

---

# ▶️ Running the Pipeline

Because the repository currently contains the Python nodes but does not include a complete ROS 2 package structure (`package.xml`, `setup.py`, launch files, etc.), the scripts need to be placed into an appropriate ROS 2 Python package before using standard `ros2 run` commands.

Once the scripts are part of a ROS 2 package, the general workflow is:

### Terminal 1 — Start the image publisher

```bash
ros2 run <package_name> image_publisher
```

### Terminal 2 — Start the pose subscriber

```bash
ros2 run <package_name> image_subscriber
```

### Terminal 3 — Inspect topics

```bash
ros2 topic list
```

You should see topics including:

```text
/camera/image_raw
/detections_3d
/pose_stamped
/pose_maker
/camera_info
```

You can inspect the pose:

```bash
ros2 topic echo /pose_stamped
```

---

# 👁️ RViz Visualization

RViz can be used to visualize the ROS 2 outputs.

Start RViz with:

```bash
rviz2
```

Useful displays include:

```text
TF
Marker
Pose
Image
```

The TF tree should contain the relationship:

```text
poster → camera
```

The marker published on:

```text
/pose_maker
```

can be used to visualize the estimated orientation.

---

# 📐 Coordinate Frames

The project currently uses the following conceptual frames:

```text
poster
  │
  └── camera
```

The planar target is treated as the reference coordinate system.

The target plane is defined using:

```text
Z = 0
```

for all four reference points.

The estimated camera pose is represented relative to this coordinate system.

When integrating this pipeline into a larger robotics system, the frame conventions should be explicitly verified because changing parent/child frame definitions can change the interpretation of the reported translation and rotation.

---

# 🔬 Technical Summary

The project combines multiple perception techniques rather than relying on a single computer-vision algorithm.

### Object detection

YOLO provides a confidence-gated detection of the target.

### Feature-based localization

SIFT identifies visual features that can be matched between the reference and current images.

### Geometric estimation

A RANSAC homography determines how the planar reference image maps into the current camera image.

### Metric pose estimation

Known 3D coordinates of the planar target are combined with projected 2D image points and camera calibration through PnP.

### Robotics integration

The resulting pose is converted into ROS-compatible messages and TF transforms.

This creates an end-to-end perception pipeline:

```text
Visual Features
      ↓
2D Correspondences
      ↓
Planar Geometry
      ↓
3D Camera Pose
      ↓
ROS 2 Pose
      ↓
TF / RViz / Robotics Stack
```

---

# ⚠️ Current Limitations

This repository represents a research/prototype implementation and contains several pieces that should be improved before treating it as a reusable ROS 2 package.

### Hard-coded paths

The video and reference-image paths are currently absolute paths tied to the original development environment.

These should be replaced with:

- ROS parameters
- launch-file arguments
- package-share paths
- environment variables
- command-line arguments

### Camera calibration

The calibration values in the code are specific to the camera used during development.

The code itself contains a comment indicating that some values are not final.

A proper deployment should use calibration parameters obtained from the actual camera.

### ROS 2 package structure

The repository currently does not contain the standard ROS 2 package files such as:

```text
package.xml
setup.py
setup.cfg
resource/
launch/
```

Adding these would make the project easier to build and run with `colcon`.

### Pose accuracy

The final translation estimate depends heavily on:

- camera calibration
- reference-object dimensions
- feature matching quality
- homography accuracy
- target visibility
- image resolution
- camera viewpoint

Poor calibration or inaccurate reference dimensions can lead to significant errors in the estimated translation.

### YOLO and geometric estimation

The YOLO detection and SIFT/PnP estimation serve different roles.

YOLO provides a detection/confidence gate, while the actual 6-DOF geometric pose is calculated using feature correspondences and PnP.

This means that a high YOLO confidence does not automatically guarantee a highly accurate pose.

---

# 🚀 Future Improvements

Potential improvements include:

- Convert the scripts into a complete ROS 2 Python package
- Add `package.xml`
- Add `setup.py` / `setup.cfg`
- Add ROS 2 launch files
- Replace hard-coded paths with ROS parameters
- Add configurable camera calibration parameters
- Publish calibration through a proper `CameraInfo` configuration
- Add visualization of SIFT matches and homography in debugging mode
- Add pose-confidence / reprojection-error metrics
- Improve outlier rejection
- Evaluate multiple PnP algorithms
- Add temporal filtering to reduce pose jitter
- Add Kalman filtering for smoother tracking
- Add configurable YOLO confidence thresholds
- Support live camera input instead of only prerecorded video
- Improve TF frame naming and coordinate-frame documentation
- Add automated tests for pose estimation
- Benchmark pose accuracy against ground-truth measurements

---

# 🧪 Error Handling

The pose-estimation function checks several failure conditions.

### Images cannot be loaded

```text
Error: Could not read one or both images.
```

### No SIFT descriptors

```text
Error: No features detected.
```

### Too few feature matches

```text
Error: Not enough matches to compute homography.
```

### Homography failure

```text
Error: Homography computation failed.
```

### PnP failure

```text
Error: Pose estimation failed.
```

The ROS 2 subscriber logs pose-estimation failures and skips publishing the pose for that frame.

---

# 📊 Outputs

The estimated pose ultimately contains:

```text
Translation
───────────
x
y
z

Rotation
────────
x
y
z
w
```

where the rotational components are represented as a quaternion for ROS 2 compatibility.

The Euclidean distance from the camera to the estimated target position can also be calculated as:

```text
distance = √(x² + y² + z²)
```

---

# 👨‍💻 Author

**Tarun Shivakumar**

Computer Science & Engineering student interested in:

- Computer Vision
- Robotics
- Autonomous Systems
- Perception
- SLAM
- Sensor Fusion
- 3D Localization

---

# 📄 License

No open-source license is currently specified in this repository.

If this project is intended to be distributed or reused publicly, consider adding an appropriate license.

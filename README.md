# Monocular Multi-Modal Vision Framework

A ROS 2-based **monocular visual perception pipeline** for estimating camera motion and generating sparse 3D structure from image sequences.

The framework combines multiple visual-processing techniques:

- **Lucas-Kanade optical flow** for frame-to-frame feature tracking
- **Essential matrix estimation + RANSAC** for relative camera motion
- **SIFT feature matching** for robust feature correspondences
- **Stereo-style triangulation from consecutive monocular views** to recover 3D points
- **ROS 2 custom messages** for bundling consecutive image frames
- **TF2** for publishing camera motion
- **`nav_msgs/Path` and `PoseStamped`** for camera trajectory visualization
- **`sensor_msgs/PointCloud2`** for publishing reconstructed 3D points
- **Blur-aware frame sampling** to prepare video data for visual odometry / SfM

Despite the repository name containing "multi-modal," the current implementation is primarily **multi-method monocular vision**: multiple computer-vision techniques are applied to the same camera stream rather than combining physical sensors such as LiDAR, IMU, or stereo cameras.

---

## Overview

The system takes a sequence of images from a single camera and processes overlapping groups of three frames.

```text
Input Video
    │
    ▼
Frame Sampling
    │
    ├── Sample every Nth frame
    └── Remove blurry frames
    │
    ▼
Frame Sequence
    │
    ▼
Overlapping Image Triples
(1,2,3), (3,4,5), (5,6,7), ...
    │
    ▼
ROS 2 ImageTriple Message
    │
    ▼
┌─────────────────────────────────────┐
│        Monocular Perception         │
│                                     │
│  Frame A ──► Frame B ──► Frame C    │
│      │           │           │       │
│      ▼           ▼           ▼       │
│ Optical Flow / Essential Matrix     │
│      │           │                   │
│      ▼           ▼                   │
│ Relative R + t                       │
│                                      │
│ SIFT Feature Matching                │
│      │           │                   │
│      ▼           ▼                   │
│ Matched 2D Features                 │
│                                      │
│ Triangulation                        │
│      │                               │
│      ▼                               │
│ Sparse 3D Point Cloud                │
└─────────────────────────────────────┘
    │
    ├── Camera Pose
    ├── Camera Path
    ├── TF Transform
    └── 3D Point Cloud
```

---

# 🎯 Project Goal

The goal of the framework is to build a lightweight monocular perception pipeline capable of estimating:

1. **Relative camera rotation**
2. **Relative camera translation**
3. **Accumulated camera trajectory**
4. **Sparse 3D structure from tracked/matched image features**

The project is structured as a ROS 2 package so that the estimated pose and point cloud can be consumed by other robotics components.

---

# 🧠 Core Concepts

## Monocular Visual Odometry

Visual odometry estimates how a camera moves by analyzing changes between consecutive images.

Given:

```text
Image A → Image B
```

the system attempts to estimate:

```text
R = rotation
t = translation
```

The process is repeated over time:

```text
Frame 1 → Frame 2 → Frame 3 → Frame 4 → ...
     │          │          │
     ▼          ▼          ▼
   Motion     Motion     Motion
```

The individual relative motions can then be accumulated to estimate a camera trajectory.

---

# 🔍 Optical Flow

The project uses OpenCV's Lucas-Kanade optical flow implementation.

Feature points are first detected with:

```python
cv.goodFeaturesToTrack(...)
```

The Lucas-Kanade tracker then attempts to locate those same points in the next frame:

```python
cv.calcOpticalFlowPyrLK(...)
```

The implementation uses:

```python
feature_params = {
    "maxCorners": 100,
    "qualityLevel": 0.3,
    "minDistance": 7,
    "blockSize": 7
}
```

and:

```python
lk_params = {
    "winSize": (15, 15),
    "maxLevel": 2,
    "criteria": (
        cv.TERM_CRITERIA_EPS |
        cv.TERM_CRITERIA_COUNT,
        10,
        0.03
    )
}
```

The result is a set of corresponding points:

```text
Frame 1                    Frame 2

  •                          •
     •                    •
          •              •
   •                          •
        •                  •

       matched feature locations
```

---

# 📐 Essential Matrix

After tracking features between two frames, the project estimates the essential matrix:

```python
E, _ = cv.findEssentialMat(
    good_old,
    good_new,
    None,
    method=cv.RANSAC,
    prob=0.999,
    threshold=1.0
)
```

RANSAC is used to reduce the effect of incorrect feature correspondences.

The essential matrix encodes the epipolar geometry between the two camera views.

The relative camera pose is then recovered using:

```python
_, R, t, _ = cv.recoverPose(
    E,
    good_old,
    good_new,
    None
)
```

The result is:

```text
R → relative rotation
t → relative translation direction
```

### Important monocular limitation

For a monocular camera, the translation recovered from the essential matrix has an inherent **scale ambiguity**.

In other words, the system can estimate the direction and relative magnitude of camera motion, but it does not automatically know the real-world distance traveled without an external scale reference.

This is a fundamental limitation of monocular visual odometry.

---

# 🔗 Pose Accumulation

The ROS 2 subscriber maintains:

```python
self.R_global = np.eye(3)
self.t_global = np.zeros((3, 1))
```

For each new relative motion, the translation is transformed into the accumulated coordinate frame:

```python
self.t_global = np.add(
    self.t_global,
    np.matmul(self.R_global, t)
)
```

The global rotation is updated using:

```python
self.R_global = np.matmul(
    R,
    self.R_global
)
```

Conceptually:

```text
Relative Motion 1
      ↓
Global Pose 1
      ↓
Relative Motion 2
      ↓
Global Pose 2
      ↓
Relative Motion 3
      ↓
Global Pose 3
```

The accumulated pose is then converted into a quaternion for ROS 2.

---

# 🧩 SIFT Feature Matching

The project also implements a separate feature-matching pipeline using SIFT.

SIFT is initialized with:

```python
sift = cv.SIFT_create()
```

Descriptors are extracted from both frames:

```python
kp1, des1 = sift.detectAndCompute(gray1, None)
kp2, des2 = sift.detectAndCompute(gray2, None)
```

A brute-force matcher is then used:

```python
bf = cv.BFMatcher()
```

The two nearest matches are found using:

```python
bf.knnMatch(des1, des2, k=2)
```

---

# ✅ Lowe Ratio Test

Incorrect or ambiguous SIFT matches are filtered using the ratio test:

```python
good = [
    m for m, n in matches
    if m.distance < 0.75 * n.distance
]
```

This keeps a match when the best descriptor match is significantly better than the second-best candidate.

The remaining feature locations are returned as:

```text
points1
points2
```

These matched 2D coordinates are used for 3D reconstruction.

---

# 🌐 Triangulation

The project reconstructs 3D points from corresponding observations in two camera frames.

Projection matrices are constructed as:

```python
P1 = K [R1 | t1]
P2 = K [R2 | t2]
```

where:

```text
K  = camera intrinsic matrix
R  = camera rotation
t  = camera translation
```

OpenCV's triangulation function is then used:

```python
cv.triangulatePoints(
    proj_matrix1,
    proj_matrix2,
    points1,
    points2
)
```

The resulting homogeneous coordinates are converted into 3D coordinates:

```python
points3D = points4D[:3, :] / points4D[3, :]
```

The final result is a sparse point cloud:

```text
        •
   •         •
       •
 •             •
          •
```

These points are published as a ROS 2 `PointCloud2` message.

---

# 📸 Frame Sampling

Before running the ROS 2 pipeline, the repository includes a preprocessing tool:

```text
sample_frames.py
```

It extracts useful frames from a video and removes blurry frames.

This is particularly useful for:

- Visual odometry
- Structure-from-motion
- COLMAP preprocessing
- Reducing redundant frames
- Removing motion-blurred images

---

## Sampling Interval

The script can keep every Nth frame:

```bash
python sample_frames.py input.mov frames --interval 5
```

For example:

```text
Original video:

1 2 3 4 5 6 7 8 9 10 ...

interval = 5

↓
1 6 11 16 ...
```

---

# 🔬 Blur Detection

The project uses the **variance of the Laplacian** as a sharpness metric.

```python
gray = cv2.cvtColor(
    frame,
    cv2.COLOR_BGR2GRAY
)

score = cv2.Laplacian(
    gray,
    cv2.CV_64F
).var()
```

Higher values generally correspond to sharper images.

Low values can indicate:

- Motion blur
- Defocus
- Poor image quality

Frames below the configured threshold are discarded.

The default threshold is:

```text
100.0
```

---

# 🖼️ Output Frame Naming

Surviving frames are written sequentially:

```text
frame_0000.jpg
frame_0001.jpg
frame_0002.jpg
...
```

This ensures alphabetical ordering matches chronological ordering.

---

# 🔗 ROS 2 Architecture

The repository contains two ROS 2 packages:

```text
mono_perception_msgs
        │
        │ custom ImageTriple message
        ▼
mono_perception_pipeline
        │
        ├── image_publisher
        │
        └── image_subscriber
```

---

# 📦 `mono_perception_msgs`

This package defines the custom:

```text
ImageTriple
```

message.

The message contains:

```text
std_msgs/Header header

uint32 triple_index

sensor_msgs/Image image_a
sensor_msgs/Image image_b
sensor_msgs/Image image_c

string image_a_name
string image_b_name
string image_c_name
```

The purpose is to bundle three consecutive frames into a single ROS message.

---

# 🔄 Why Three Frames?

The publisher creates overlapping triples:

```text
(1, 2, 3)
(3, 4, 5)
(5, 6, 7)
(7, 8, 9)
...
```

Notice that each triple overlaps with the next one by one frame.

This allows the downstream node to process a short temporal window while avoiding independently timed image topics.

---

# 📡 `image_publisher`

The `image_publisher` node:

1. Reads images from a folder
2. Sorts them naturally
3. Groups them into overlapping triples
4. Converts them into ROS `Image` messages
5. Publishes `ImageTriple`
6. Stops automatically after the final triple

Default publish rate:

```text
2.0 triples / second
```

Default topic:

```text
/image_triples
```

Default frame ID:

```text
camera
```

The input directory is provided as a ROS parameter.

---

# 📥 `image_subscriber`

The `image_subscriber` node subscribes to:

```text
/image_triples
```

and performs the main perception processing.

For each `ImageTriple`, it:

1. Converts ROS images to OpenCV images
2. Computes optical flow for A → B
3. Computes optical flow for B → C
4. Estimates relative camera motion
5. Accumulates camera pose
6. Converts rotation matrices to quaternions
7. Publishes camera poses
8. Broadcasts TF transforms
9. Performs SIFT feature matching
10. Triangulates 3D points
11. Publishes a point cloud

---

# 📤 ROS 2 Topics

## `/image_triples`

Message:

```text
mono_perception_msgs/msg/ImageTriple
```

Contains three overlapping consecutive frames.

---

## `/camera/pose`

Message:

```text
geometry_msgs/msg/PoseStamped
```

Contains the accumulated camera position and orientation.

---

## `/camera/path`

Message:

```text
nav_msgs/msg/Path
```

Contains the estimated camera trajectory.

This can be visualized in RViz.

---

## `/camera/point_cloud`

Message:

```text
sensor_msgs/msg/PointCloud2
```

Contains the reconstructed 3D feature points.

---

# 🌍 TF

The subscriber broadcasts a transform:

```text
world
  │
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
    x
    y
    z
    w
```

This makes the estimated camera pose available through the ROS 2 TF system.

---

# 📁 Repository Structure

```text
Monocular-Multi-Modal-Vision-Framework/
│
├── mono_perception_msgs/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       └── ImageTriple.msg
│
├── mono_perception_pipeline/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   │
│   ├── launch/
│   │   └── image_publisher.launch.py
│   │
│   ├── mono_perception_pipeline/
│   │   ├── image_publisher.py
│   │   ├── image_subscriber.py
│   │   ├── optical_flow.py
│   │   ├── feature_matching.py
│   │   ├── triangulation.py
│   │   └── sample_frames.py
│   │
│   └── test/
│
├── build/
├── install/
├── log/
└── README.md
```

The repository also contains generated ROS 2 `build/`, `install/`, and `log/` directories from previous builds. These are generally not required in source control and can be regenerated with `colcon build`.

---

# 🛠️ Installation

## Requirements

The project requires:

- ROS 2
- Python 3
- OpenCV
- NumPy
- SciPy
- `cv_bridge`
- ROS 2 message packages
- `tf2_ros`
- `colcon`

The package is configured as an `ament_python` package.

---

## 1. Create a ROS 2 Workspace

If you do not already have a workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone or copy the repository into the `src` directory.

The source tree should ultimately look approximately like:

```text
~/ros2_ws/src/
├── mono_perception_msgs/
└── mono_perception_pipeline/
```

---

## 2. Install ROS Dependencies

From the workspace:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

This installs available ROS package dependencies.

You may also need the Python packages used directly by the vision modules:

```bash
pip install numpy scipy opencv-python
```

On ROS systems, using the distribution-provided OpenCV packages can be preferable to mixing system ROS Python packages with pip packages.

---

# 🔨 Build

From the workspace root:

```bash
cd ~/ros2_ws
colcon build
```

After building:

```bash
source install/setup.bash
```

For convenience:

```bash
source ~/ros2_ws/install/setup.bash
```

---

# ▶️ Running the Pipeline

## Step 1 — Prepare Frames

Use the frame sampler to convert a video into a clean image sequence.

For example:

```bash
python3 sample_frames.py input.mov frames
```

Or specify the sampling interval and blur threshold:

```bash
python3 sample_frames.py input.mov frames \
    --interval 5 \
    --blur-threshold 100
```

---

## Step 2 — Launch the Image Publisher

The repository includes:

```text
image_publisher.launch.py
```

Run:

```bash
ros2 launch mono_perception_pipeline image_publisher.launch.py \
    input_folder:=/path/to/frames
```

You can also configure the publish rate:

```bash
ros2 launch mono_perception_pipeline image_publisher.launch.py \
    input_folder:=/path/to/frames \
    publish_rate_hz:=2.0
```

The launch file supports:

```text
input_folder
publish_rate_hz
topic_name
frame_id
```

---

## Step 3 — Start the Perception Node

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run mono_perception_pipeline image_subscriber
```

The node subscribes to:

```text
/image_triples
```

and publishes:

```text
/camera/pose
/camera/path
/camera/point_cloud
```

---

# 👁️ RViz Visualization

Start RViz:

```bash
rviz2
```

Useful displays include:

```text
TF
Path
PointCloud2
```

Set the RViz fixed frame to:

```text
world
```

Then add:

```text
/camera/path
/camera/point_cloud
```

This allows the estimated camera trajectory and reconstructed 3D structure to be inspected visually.

---

# 🔎 Inspecting ROS Topics

List active topics:

```bash
ros2 topic list
```

Inspect the camera pose:

```bash
ros2 topic echo /camera/pose
```

Inspect the path:

```bash
ros2 topic echo /camera/path
```

Inspect TF:

```bash
ros2 run tf2_tools view_frames
```

Inspect the point cloud:

```bash
ros2 topic info /camera/point_cloud
```

---

# 🧮 Important Monocular Geometry Considerations

## Scale Ambiguity

The most important limitation of monocular visual odometry is scale.

From two monocular images, the essential matrix can recover:

```text
Rotation
Translation direction
```

but not absolute metric translation by itself.

For example, these two trajectories may produce the same image geometry:

```text
Camera moves 1 meter
Camera moves 10 meters
```

if the scene is scaled accordingly.

Therefore, the translation generated by:

```python
cv.recoverPose(...)
```

should not automatically be interpreted as meters.

To obtain metric scale, the system would need an additional source of scale such as:

- IMU
- Wheel odometry
- Known object dimensions
- Stereo baseline
- LiDAR
- GPS
- Depth sensor
- Ground-truth trajectory

---

# ⚠️ Current Implementation Limitations

This repository is a research/prototype implementation and has several areas that should be addressed before using it as a production visual-odometry system.

## Camera Intrinsics

The triangulation function expects:

```python
mtx
```

for the camera intrinsic matrix.

However, in the current `image_subscriber.py`, the member:

```python
self.mtx = None
```

is initialized but is not populated before being passed into `compute_point_cloud()`.

Therefore, the point-cloud portion requires a valid camera intrinsic matrix to be supplied before triangulation can work correctly.

A proper implementation should load calibration from:

- ROS `CameraInfo`
- a YAML calibration file
- a ROS parameter
- another calibrated-camera source

---

## Translation Scale

As described above, the translation returned by monocular essential-matrix recovery is scale ambiguous.

The current global trajectory therefore should not be interpreted as automatically metric.

---

## Pose Drift

Because relative poses are accumulated:

```text
Pose 1
  ↓
Pose 2
  ↓
Pose 3
  ↓
Pose 4
  ↓
...
```

small errors accumulate over time.

This produces drift.

Long sequences can therefore cause the estimated trajectory to gradually diverge from the real camera trajectory.

Potential solutions include:

- Loop closure
- Bundle adjustment
- Keyframe selection
- Pose graph optimization
- IMU fusion
- Stereo/depth measurements
- SLAM frameworks such as ORB-SLAM

---

## Point-Cloud Consistency

The current point-cloud generation uses the relative pose estimates and matched SIFT features.

For high-quality reconstruction, additional filtering would be useful, including:

- Reprojection-error filtering
- Depth consistency checks
- Cheirality checks
- Outlier rejection
- Temporal feature tracking
- Bundle adjustment

---

# 🔬 Why Use Both Optical Flow and SIFT?

The project intentionally contains two different feature-processing approaches.

### Optical Flow

Used for:

```text
Frame A → Frame B
Frame B → Frame C
```

to estimate short-term camera motion.

Optical flow is efficient because it tracks existing image features directly.

### SIFT

Used for more descriptor-based feature matching.

SIFT provides:

- Scale-invariant descriptors
- More explicit feature correspondences
- A useful basis for triangulation

The architecture therefore separates the responsibilities:

```text
Optical Flow
     ↓
Camera Motion

SIFT Matching
     ↓
Feature Correspondences
     ↓
3D Triangulation
```

This is the "multi-method" aspect of the current implementation.

---

# 🧪 Frame Quality Pipeline

The preprocessing stage provides a useful workflow for preparing video data:

```text
Raw Video
    ↓
Frame Sampling
    ↓
Laplacian Blur Score
    ↓
Remove Blurry Frames
    ↓
Sequential JPEG Frames
    ↓
ROS 2 ImageTriple Publisher
```

This reduces the number of poor-quality frames entering the visual-odometry pipeline.

---

# 🚀 Future Improvements

Potential improvements include:

### Camera Calibration

- Load `CameraInfo` automatically
- Add calibration YAML support
- Publish calibrated camera parameters
- Validate reprojection error

### Visual Odometry

- Add keyframe selection
- Improve optical-flow outlier rejection
- Track features over longer temporal windows
- Add forward-backward optical-flow validation
- Improve essential-matrix filtering

### Scale

- Fuse IMU measurements
- Add stereo/depth support
- Add known-scale landmarks
- Integrate wheel odometry
- Add GPS where appropriate

### SLAM

- Add loop closure
- Add pose-graph optimization
- Add bundle adjustment
- Improve long-term trajectory consistency

### 3D Reconstruction

- Add reprojection-error filtering
- Reject points behind the camera
- Add depth consistency checks
- Improve point-cloud density
- Add persistent landmark tracking

### ROS 2

- Add a complete launch system
- Add configurable camera calibration parameters
- Add RViz configuration
- Add diagnostic topics
- Add synchronized image/timestamp handling
- Add automated integration tests

---

# 🧱 Package Architecture

```text
                    ┌─────────────────────┐
                    │     Video File      │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │   sample_frames.py  │
                    │                     │
                    │ Sampling + Blur     │
                    │ Detection           │
                    └──────────┬──────────┘
                               │
                               ▼
                       Frame Directory
                               │
                               ▼
                    ┌─────────────────────┐
                    │  image_publisher    │
                    │                     │
                    │ Frames → ImageTriple│
                    └──────────┬──────────┘
                               │
                      /image_triples
                               │
                               ▼
                    ┌─────────────────────┐
                    │  image_subscriber   │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼─────────────────┐
              │                │                 │
              ▼                ▼                 ▼
       Optical Flow      SIFT Matching      ROS 2 Pose
              │                │                 │
              ▼                ▼                 ▼
       Essential Matrix    2D Points         TF / Path
              │                │
              ▼                ▼
          R + t           Triangulation
                               │
                               ▼
                         Point Cloud
```

---

# 📊 ROS 2 Interface Summary

| Topic | Message Type | Purpose |
|---|---|---|
| `/image_triples` | `mono_perception_msgs/ImageTriple` | Three overlapping input frames |
| `/camera/pose` | `geometry_msgs/PoseStamped` | Estimated camera pose |
| `/camera/path` | `nav_msgs/Path` | Accumulated camera trajectory |
| `/camera/point_cloud` | `sensor_msgs/PointCloud2` | Reconstructed 3D feature points |

TF:

```text
world → camera
```

---

# 📚 Main Technologies

| Technology | Purpose |
|---|---|
| ROS 2 | Robotics middleware |
| Python | Main implementation language |
| OpenCV | Computer vision |
| Lucas-Kanade Optical Flow | Feature tracking |
| SIFT | Feature description and matching |
| RANSAC | Outlier rejection |
| Essential Matrix | Relative camera motion |
| PnP-style geometry | Camera/scene geometry concepts |
| Triangulation | 3D reconstruction |
| NumPy | Numerical operations |
| SciPy | Rotation/quaternion handling |
| TF2 | Coordinate transforms |
| RViz | Visualization |
| Colcon | ROS 2 build system |

---

# 👨‍💻 Author

**Tarun Shivakumar**

Computer Science & Engineering student interested in:

- Computer Vision
- Robotics
- Autonomous Systems
- Visual Odometry
- SLAM
- Sensor Fusion
- 3D Reconstruction
- Perception Systems

---

# 📄 License

No open-source license is currently specified for this project.

If the repository is intended for public reuse or distribution, consider adding an appropriate open-source license.

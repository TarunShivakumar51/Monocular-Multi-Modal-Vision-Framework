# Monocular Multi-Modal Vision Framework

A ROS 2-based perception framework for processing monocular image sequences and preparing temporally consistent visual data for downstream computer-vision and visual-odometry pipelines.

The repository currently contains two ROS 2 packages:

- `mono_perception_pipeline` — Python ROS 2 nodes for image-sequence processing and publication.
- `mono_perception_msgs` — custom ROS 2 interfaces for bundling consecutive image frames.

## Overview

The framework is designed around a simple idea: instead of publishing individual camera frames independently, consecutive frames are grouped into a single message so downstream perception algorithms receive a consistent temporal window.

The current image publisher:

1. Reads images from a configurable folder.
2. Sorts the images in natural filename order.
3. Groups them into overlapping triples.
4. Publishes one triple at a configurable rate.
5. Automatically shuts down after the image sequence has been processed.

For example, a sequence:

```text
frame_1.jpg
frame_2.jpg
frame_3.jpg
frame_4.jpg
frame_5.jpg
frame_6.jpg
frame_7.jpg
```

is published as:

```text
(frame_1, frame_2, frame_3)
(frame_3, frame_4, frame_5)
(frame_5, frame_6, frame_7)
```

This overlapping structure preserves temporal continuity while limiting each message to three frames.

## Architecture

```text
Image Sequence
      │
      ▼
┌──────────────────────────┐
│  Image Triple Publisher  │
│  mono_perception_pipeline│
└────────────┬─────────────┘
             │
             │ /image_triples
             ▼
┌──────────────────────────┐
│   ImageTriple Message    │
│  mono_perception_msgs    │
│                          │
│  image_a                 │
│  image_b                 │
│  image_c                 │
└────────────┬─────────────┘
             │
             ▼
 Downstream Perception /
 Visual-Odometry Processing
```

## Repository Structure

```text
Monocular-Multi-Modal-Vision-Framework/
├── mono_perception_msgs/
│   ├── msg/
│   │   └── ImageTriple.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
├── mono_perception_pipeline/
│   ├── mono_perception_pipeline/
│   │   └── image_publisher.py
│   ├── launch/
│   ├── resource/
│   ├── setup.py
│   └── package.xml
│
├── install/
├── build/
├── log/
└── README.md
```

## Requirements

- ROS 2
- Python 3
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- OpenCV
- `mono_perception_msgs`

The ROS 2 packages declare their required dependencies through `package.xml`.

## Installation

Clone the repository into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/TarunShivakumar51/Monocular-Multi-Modal-Vision-Framework.git
```

Build the workspace:

```bash
cd ~/ros2_ws

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

## Running the Image Publisher

The main executable is:

```text
image_publisher
```

Run it with:

```bash
ros2 run mono_perception_pipeline image_publisher
```

The publisher requires an input image directory. A typical invocation is:

```bash
ros2 run mono_perception_pipeline image_publisher --ros-args   -p input_folder:=/path/to/images
```

### Available Parameters

| Parameter | Default | Description |
|---|---:|---|
| `input_folder` | `""` | Directory containing the input images |
| `publish_rate_hz` | `2.0` | Number of image triples published per second |
| `topic_name` | `image_triples` | ROS 2 topic used for the `ImageTriple` messages |
| `frame_id` | `camera` | TF frame ID assigned to published images |

Example:

```bash
ros2 run mono_perception_pipeline image_publisher --ros-args   -p input_folder:=/home/user/dataset/images   -p publish_rate_hz:=5.0   -p topic_name:=image_triples   -p frame_id:=camera
```

## Image Sequence Requirements

The publisher recognizes:

- `.jpg`
- `.jpeg`
- `.png`
- `.bmp`
- `.pgm`
- `.tif`
- `.tiff`

Images are naturally sorted, so filenames such as:

```text
frame_1.jpg
frame_2.jpg
frame_10.jpg
```

are ordered chronologically rather than lexicographically.

The intended input sequence contains at least three images. For a complete overlapping sequence, use an odd number of frames:

```text
3, 5, 7, 9, ...
```

Any trailing frames that cannot form a complete triple are not published.

## Custom Message

The `mono_perception_msgs` package defines:

```text
ImageTriple.msg
```

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

The three images are bundled into one ROS 2 message so a downstream visual-odometry or perception node can process a matched temporal window.

You can inspect the interface with:

```bash
ros2 interface show mono_perception_msgs/msg/ImageTriple
```

## Inspecting the Published Data

List active topics:

```bash
ros2 topic list
```

Check the publisher:

```bash
ros2 topic info /image_triples
```

Inspect the message structure:

```bash
ros2 interface show mono_perception_msgs/msg/ImageTriple
```

Echo the topic:

```bash
ros2 topic echo /image_triples
```

Because the message contains complete `sensor_msgs/Image` objects, `ros2 topic echo` can produce a large amount of output.

## Design Goals

The framework is intended to provide:

- **Temporal consistency** — consecutive frames are explicitly grouped together.
- **Modular ROS 2 communication** — perception components communicate through ROS 2 topics and custom interfaces.
- **Dataset-driven testing** — image sequences can be replayed without requiring a physical camera.
- **Reusable interfaces** — the `ImageTriple` message can be consumed by independent downstream perception nodes.
- **Configurable playback** — image source, publication rate, topic name, and frame ID are ROS 2 parameters.

## Development

After modifying the source code, rebuild with:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For faster iteration on Python nodes, `--symlink-install` allows changes to Python source files to be reflected without copying the package into the install space.

## Future Extensions

The framework can be extended with additional perception modules such as:

- Monocular visual odometry
- Feature detection and tracking
- Depth estimation
- Camera pose estimation
- Multi-modal feature fusion
- 3D reconstruction
- Sensor or camera calibration utilities
- Visualization and evaluation tools

## Author

**Tarun Shivakumar**

Computer Science & Engineering  
The Ohio State University

## License

A license has not yet been specified for this repository.

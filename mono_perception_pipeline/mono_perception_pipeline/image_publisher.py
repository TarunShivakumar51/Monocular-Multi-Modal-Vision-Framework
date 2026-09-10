"""Publishes overlapping triples of frames from a fixed image folder for
downstream VO.

ImageTriplePublisher reads every image file out of a configurable folder
once at startup, sorts them into chronological order, and publishes them
three at a time as mono_perception_msgs/ImageTriple messages -- one
triple per timer tick -- so a downstream visual-odometry node always
receives three consecutive frames bundled into a single message.
Consecutive triples overlap by one frame: the publisher advances two
frames per message, so the last frame of one triple is the first frame
of the next, e.g. (1,2,3), (3,4,5), (5,6,7), ... The source folder is
only ever read, never modified.

This is a run-once node over a fixed set of files: the input set is
expected to be sized so every triple is full (i.e. len(files) is odd
and >= 3, since each message after the first contributes 2 new frames).
After the last triple is published, the timer is cancelled and the node
shuts itself down cleanly.
"""

import os
import re

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from mono_perception_msgs.msg import ImageTriple

# Recognized still-image extensions, matched case-insensitively.
_IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.pgm', '.tif', '.tiff'}

_DIGIT_RUN = re.compile(r'(\d+)')


def _natural_sort_key(filename):
    """Split a filename into text/number chunks for natural ordering.

    Zero-padded names like frame_0000.jpg already sort correctly with a
    plain string sort, but this key also keeps non-padded names (e.g.
    frame_9.jpg before frame_10.jpg) in the right chronological order,
    so filename ordering is guaranteed defensively either way.
    """
    parts = _DIGIT_RUN.split(filename)
    return [int(part) if part.isdigit() else part.lower() for part in parts]


class ImageTriplePublisher(Node):
    """Reads a fixed image folder once and publishes frames as overlapping
    triples.

    Parameters:
        input_folder (string): Directory containing the image files to
            publish. Read once at startup; never written to or cleared.
        publish_rate_hz (double): One triple is published per timer
            tick, at this rate. Defaults to 2.0 Hz (one triple every
            0.5 s).
        topic_name (string): Topic to publish ImageTriple messages on.
        frame_id (string): frame_id stamped on every published Image
            and on the ImageTriple header.
    """

    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('input_folder', '')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('topic_name', 'image_triples')
        self.declare_parameter('frame_id', 'camera')

        input_folder = self.get_parameter('input_folder') \
            .get_parameter_value().string_value
        publish_rate_hz = self.get_parameter('publish_rate_hz') \
            .get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name') \
            .get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id') \
            .get_parameter_value().string_value

        if publish_rate_hz <= 0.0:
            self.get_logger().warn(
                f"publish_rate_hz={publish_rate_hz} is not positive; "
                "defaulting to 2.0 Hz.")
            publish_rate_hz = 2.0

        self._bridge = CvBridge()
        self._input_folder = input_folder
        self._triples = self._build_triples(input_folder)
        self._triple_index = 0

        self._publisher = self.create_publisher(ImageTriple, topic_name, 10)

        if not self._triples:
            self.get_logger().error(
                f"No usable image triples found in '{input_folder}'. "
                "Nothing to publish; shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Found {len(self._triples)} triple(s) in '{input_folder}'; "
            f"publishing at {publish_rate_hz:.2f} Hz on '{topic_name}'.")

        period_s = 1.0 / publish_rate_hz
        self._timer = self.create_timer(period_s, self._publish_next_triple)

    def _build_triples(self, input_folder):
        """Return sorted image filenames from input_folder, grouped into
        overlapping triples.

        Each element is a 3-tuple of filenames. Triples overlap by one
        frame: triple i is filenames[2*i : 2*i + 3], so the last frame
        of one triple is the first frame of the next. Any trailing
        frames that can't fill a full triple are dropped -- the input
        set is expected to be sized so this never happens. Returns an
        empty list if the folder is missing, empty, or contains fewer
        than 3 recognized image files.
        """
        if not input_folder or not os.path.isdir(input_folder):
            self.get_logger().error(
                f"input_folder '{input_folder}' does not exist or was "
                "not set.")
            return []

        filenames = sorted(
            (f for f in os.listdir(input_folder)
             if os.path.splitext(f)[1].lower() in _IMAGE_EXTENSIONS
             and os.path.isfile(os.path.join(input_folder, f))),
            key=_natural_sort_key)

        triples = [tuple(filenames[i:i + 3])
                   for i in range(0, len(filenames) - 2, 2)]

        n_used = len(filenames)
        n_dropped = n_used - (2 * len(triples) + 1) if triples else n_used
        if n_dropped > 0:
            self.get_logger().warn(
                f"{n_dropped} trailing frame(s) in '{input_folder}' "
                "could not fill a full triple and will not be "
                "published.")

        return triples

    def _load_image_msg(self, filename):
        """Read one image file from disk and convert it to sensor_msgs/Image."""
        path = os.path.join(self._input_folder, filename)
        cv_image = cv2.imread(path, cv2.IMREAD_COLOR)
        if cv_image is None:
            raise IOError(f"Failed to read image file: {path}")

        msg = self._bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        return msg

    def _publish_next_triple(self):
        """Publish the next 3 consecutive frames as one ImageTriple."""
        filenames = self._triples[self._triple_index]

        msg = ImageTriple()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.triple_index = self._triple_index

        msg.image_a = self._load_image_msg(filenames[0])
        msg.image_a_name = filenames[0]
        msg.image_b = self._load_image_msg(filenames[1])
        msg.image_b_name = filenames[1]
        msg.image_c = self._load_image_msg(filenames[2])
        msg.image_c_name = filenames[2]

        self.get_logger().info(
            f"Publishing triple {self._triple_index}: "
            f"{filenames[0]} + {filenames[1]} + {filenames[2]}")

        self._publisher.publish(msg)
        self._triple_index += 1

        if self._triple_index >= len(self._triples):
            self._timer.cancel()
            self.get_logger().info(
                f"Finished publishing all {len(self._triples)} "
                f"message(s) from '{self._input_folder}'. Shutting "
                "down.")
            rclpy.shutdown()


def main(args=None):
    """Entry point: spin ImageTriplePublisher until its fixed set is sent."""
    rclpy.init(args=args)
    node = ImageTriplePublisher()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

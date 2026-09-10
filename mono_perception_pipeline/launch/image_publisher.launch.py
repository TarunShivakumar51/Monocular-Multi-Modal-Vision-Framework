"""Launch file for the ImageTriplePublisher node.

Exposes input_folder, publish_rate_hz, topic_name, and frame_id as
launch arguments so the node can be pointed at any frame folder without
editing code, e.g.:

    ros2 launch mono_perception_pipeline image_publisher.launch.py \\
        input_folder:=/data/frames publish_rate_hz:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the LaunchDescription for image_publisher."""
    input_folder_arg = DeclareLaunchArgument(
        'input_folder',
        default_value='',
        description='Folder containing the image files to publish '
                     '(required, must exist).')
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='2.0',
        description='Triples published per second (default: 2.0, i.e. '
                     'one triple every 0.5 s).')
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='image_triples',
        description='Topic to publish mono_perception_msgs/ImageTriple '
                     'messages on.')
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera',
        description='frame_id stamped on every published image.')

    image_publisher_node = Node(
        package='mono_perception_pipeline',
        executable='image_publisher',
        name='image_publisher',
        output='screen',
        parameters=[{
            'input_folder': LaunchConfiguration('input_folder'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'topic_name': LaunchConfiguration('topic_name'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    return LaunchDescription([
        input_folder_arg,
        publish_rate_arg,
        topic_name_arg,
        frame_id_arg,
        image_publisher_node,
    ])

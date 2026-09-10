import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tarun2006/mm_ros2_ws/src/install/mono_perception_pipeline'

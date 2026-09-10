# Bundles three consecutive frames from a fixed image-folder sequence
# into a single message, so a downstream visual-odometry node always
# receives a matched frame triple together instead of three
# independently-timed Image messages.
#
# Consecutive triples overlap by one frame: the publisher advances two
# frames per message, so image_c of one triple is image_a of the next
# (e.g. (1,2,3), (3,4,5), (5,6,7), ...). Every published message always
# carries three real frames -- there is no partial/trailing case.

std_msgs/Header header

# Index of this message within the publish sequence, starting at 0.
uint32 triple_index

sensor_msgs/Image image_a
sensor_msgs/Image image_b
sensor_msgs/Image image_c

# Source filenames, for logging/debugging and matching against ground
# truth.
string image_a_name
string image_b_name
string image_c_name

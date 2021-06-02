# The standard ROS message contains a header.
# There can be multiple people in a frame.
# Hence we created an array of a person.
# We should name this array as people.
# However, for me, while coding persons
# make much more sense as it resembles
# the data structure array.
std_msgs/Header header
Person[] persons
uint32 image_w
uint32 image_h
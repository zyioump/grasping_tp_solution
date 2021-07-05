# This message stores the detected objects

# The list of object names.
std_msgs/String[] object_names

# All the objects bounding boxes coordinates.
grasping_tp/BoundingBoxCoord[] objects_bb

# The source cloud used.
sensor_msgs/PointCloud2 cloud

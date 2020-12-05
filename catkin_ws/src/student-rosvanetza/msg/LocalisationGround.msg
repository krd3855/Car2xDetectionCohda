Header header

# data fields included in this message are relative to ENU ground plane

float64 yaw                         # radian (angle between body longitudinal axis and North)
float64 pitch                       # radian
float64 roll                        # radian

# velocity vector
geometry_msgs/Vector3 velocity      # meter per second

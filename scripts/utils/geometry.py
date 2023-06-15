
from math import *


# Rotate a point by an angle w.r.t the origin
# @param point: a list of (x, y)
# @param angle: in radians
def rotate(point, angle):
    x, y = point
    return [x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle)]

# Covert pose to box
# @param pose: a list of (x, y, yaw), in meters and radians, assuming the
#              pose is at the tail of the box
def pose_to_box(pose, length, width, clearance=0):
    corners = [
        [length + clearance, width/2 + clearance],
        [length + clearance, -width/2 - clearance],
        [0 - clearance, -width/2 - clearance],
        [0 - clearance, width/2 + clearance]
    ]
    for i in range(len(corners)):
        corners[i] = rotate(corners[i], pose[2])
        corners[i][0] += pose[0]
        corners[i][1] += pose[1]
    return corners


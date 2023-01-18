#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import geometry_msgs
import math
import yaml
import rospkg


# eğim

def wayPoints():
    

    r = rospkg.RosPack()
    wp_path = r.get_path("full_coverage_path_planner")
    filepath = "{}/src/waypoints.yaml".format(wp_path)

    with open(filepath) as stream:
        try:
            wp_dict = yaml.safe_load(stream)
            if wp_dict is None:
                wp_dict = {}
        except:
            wp_dict = {}
    
    return wp_dict


def findRotAngle(x1, y1, x2, y2):

    slope = (y2-y1)/(x2-x1)
    if (y2 >= y1) and (x2 >= x1):   # sola, eğim kadar dönecek
        rot_angle = math.atan(slope)
    elif (y2 >= y1) and (x2 < x1):    # sola, 180 - eğim kadar dönecek
        rot_angle = math.pi - math.atan(slope)
    elif (y2 < y1) and (x2 >= x1):    # sağa, eğim kadar dönecek
        rot_angle = -1*(math.atan(slope))
    elif (y2 < y1) and (x2 < x1):    # sağa, 180 - eğim kadar dönecek
        rot_angle = -1*(math.pi - math.atan(slope))

    return rot_angle


def main():

    print("---main---")
    rospy.init_node('produce_path')
    r = rospy.Rate(10)

    path_pub = rospy.Publisher('/produce_path', Path, queue_size=10)
    path = Path()
    path.header.frame_id = "map"
    

    wp_dict = wayPoints()

    for i in range(len(wp_dict)-1):
        x1 = wp_dict[i]['x']
        y1 = wp_dict[i]['y']
        x2 = wp_dict[i+1]['x']
        y2 = wp_dict[i+1]['y']

        yaw = findRotAngle(x1, y1, x2, y2)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x2
        pose.pose.position.y = y2
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        path.poses.append(pose)

    while not rospy.is_shutdown():
        print("---publish path---")
        path_pub.publish(path)
        r.sleep()


if __name__ == '__main__':
    main()

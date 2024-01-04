#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class Rectangle:
    def __init__(self, min_point, max_point):
        self.min_point = min_point
        self.max_point = max_point

class pose_check:

    def __init__(self, rectangles):
        self.sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callback)
        self.rectangles = rectangles

        rospy.loginfo("Number of Rectangles: {}".format(len(self.rectangles)))
        for idx, rectangle in enumerate(self.rectangles):
            rospy.loginfo("Rectangle {}: Min={}, Max={}".format(idx + 1, rectangle.min_point, rectangle.max_point))

    def callback(self, msg):
        position = msg.pose.pose.position
        for i, rectangle in enumerate(self.rectangles):
            if (rectangle.min_point.x <= position.x <= rectangle.max_point.x and rectangle.min_point.y <= position.y <= rectangle.max_point.y):
                rospy.loginfo("Position is within Rectangle {}: x={}, y={}".format(i + 1, position.x, position.y))
                return
        rospy.loginfo("Position is outside all rectangles: x={}, y={}".format(position.x, position.y))


def main():

    # Define multiple rectangles
    rectangles = [
        Rectangle(min_point=Point(x=-6.0, y=0.0, z=0.0), max_point=Point(x=0.0, y=7.0, z=0.0))
    ]

    check_frequency = 10.0

    rospy.init_node('pose_check', anonymous=True)
    lf = pose_check(rectangles)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    main()

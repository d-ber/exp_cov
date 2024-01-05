#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import os
import json

class Rectangle:

    def __init__(self, min_point=None, max_point=None, center_x=None, center_y=None, width=None, height=None):
        if min_point is not None and max_point is not None:
            self.min_point = min_point
            self.max_point = max_point
            self.calculate_center_dimensions()
        elif center_x is not None and center_y is not None and width is not None and height is not None:
            self.center_x = center_x
            self.center_y = center_y
            self.width = width
            self.height = height
            self.calculate_min_max_points()
        else:
            raise ValueError("Invalid parameters. Please provide either min_point and max_point or center_x, center_y, width, and height.")

    def calculate_min_max_points(self):
        half_width = self.width / 2
        half_height = self.height / 2
        self.min_point = (self.center_x - half_width, self.center_y - half_height)
        self.max_point = (self.center_x + half_width, self.center_y + half_height)

    def calculate_center_dimensions(self):
        self.center_x = (self.min_point[0] + self.max_point[0]) / 2
        self.center_y = (self.min_point[1] + self.max_point[1]) / 2
        self.width = abs(self.max_point[0] - self.min_point[0])
        self.height = abs(self.max_point[1] - self.min_point[1])

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

def read_rectangles(json_file_path):
    rectangles = []

    # Read JSON data from file
    try:
        with open(json_file_path, 'r') as json_file:
            json_data = json.load(json_file)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON at '{json_file_path}': {e}")
        return rectangles

    # Create Rectangle objects from JSON data
    for rect in json_data:
        rectangle = Rectangle(
            center_x=info["center"][0],
            center_y=info["center"][1],
            width=info["width"],
            height=info["height"]
        )
        rectangles.append(rectangle)

    return rectangles


def main():

    # Read rectangles from json file
    json_file_path = os.path.join(os.getcwd(), "rectangles.json")
    rectangles = read_rectangles(json_file_path)
    if len(rectangles) == 0:
        print("No Rectangles Given!")
        
    rospy.init_node('pose_check', anonymous=True)
    _ = pose_check(rectangles)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    main()

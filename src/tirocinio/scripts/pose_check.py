#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import geometry_msgs.msg as geo
import subprocess as sp
import json
import argparse
import scipy.stats as st

class Rectangle:
    def __init__(self, min_point=None, max_point=None, center=None, width=None, height=None):
        if min_point is not None and max_point is not None:
            # Using the original constructor with min_point and max_point
            self.min_point = min_point
            self.max_point = max_point
            self.calculate_center_dimensions()
        elif center is not None and width is not None and height is not None:
            # Using the new constructor with center_x, center_y, width, and height
            self.center = center
            self.width = width
            self.height = height
            self.calculate_min_max_points()
        else:
            raise ValueError("Invalid parameters. Please provide either min_point and max_point or center_x, center_y, width, and height.")

    def calculate_min_max_points(self):
        half_width = self.width / 2
        half_height = self.height / 2
        self.min_point = geo.Point(self.center.x - half_width, self.center.y - half_height, 0)
        self.max_point = geo.Point(self.center.x + half_width, self.center.y + half_height, 0)

    def calculate_center_dimensions(self):
        self.center = geo.Point((self.min_point.x + self.max_point.x) / 2, (self.min_point.y + self.max_point.y) / 2, 0)
        self.width = abs(self.max_point.x - self.min_point.x)
        self.height = abs(self.max_point.y - self.min_point.y)

class pose_check:

    def __init__(self, rectangles, disturb_prop):
        self.pub = rospy.Publisher('/disturb', Odometry, queue_size=10)
        self.sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callback)
        self.rectangles = rectangles
        self.inside = -1
        self.disturb_prop = disturb_prop

        rospy.loginfo("Number of Rectangles: {}".format(len(self.rectangles)))
        for idx, rectangle in enumerate(self.rectangles):
            rospy.loginfo("Rectangle {}: \nMin =\n{}, \nMax =\n{}".format(idx + 1, rectangle.min_point, rectangle.max_point))
    def callback(self, msg):
        position = msg.pose.pose.position
        for i, rectangle in enumerate(self.rectangles):
            if (rectangle.min_point.x <= position.x <= rectangle.max_point.x and rectangle.min_point.y <= position.y <= rectangle.max_point.y):
                rospy.logdebug("Position is within Rectangle {}: x={}, y={}".format(i + 1, position.x, position.y))
                if self.inside != i or st.bernoulli.rvs(self.disturb_prop): # if inside, for each message received disturb with prob 0.003
                    #TODO: aumenta il logging, e.g.: logga che disturba per entrata in rett o per permanenza in rett
                    self.pub.publish(msg)
                    self.inside = i
                return
        rospy.logdebug("Position is outside all rectangles: x={}, y={}".format(position.x, position.y))
        self.inside = -1

def read_rectangles(json_file_path):
    rectangles = []

    # Read JSON data from file
    try:
        with open(json_file_path, 'r') as json_file:
            json_data = json.load(json_file)
            # Create Rectangle objects from JSON data
            for rect in json_data:
                rectangle = Rectangle(
                    center = geo.Point(rect["center"]["x"], rect["center"]["y"], rect["center"]["z"]),
                    width=rect["width"],
                    height=rect["height"]
                )
                rectangles.append(rectangle)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON at '{json_file_path}': {e}")
    except Exception as e:
        print(f"Error reading JSON at '{json_file_path}': {e}")
    finally:
        return rectangles

def parse_args():
    parser = argparse.ArgumentParser(description='Check if a robot is within some given areas, described by rectangles.')
    parser.add_argument('--path', required=True, help="Path of the rectangles json file.")
    return parser.parse_known_args()[0]

def main():

    args = parse_args()
    json_file_path = args.path

    rectangles = read_rectangles(json_file_path)
    disturb_prop = 0.003
        
    rospy.init_node('pose_check', anonymous=True)
    _ = pose_check(rectangles, disturb_prop)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    main()

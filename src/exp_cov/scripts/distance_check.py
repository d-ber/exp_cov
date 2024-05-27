#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math
from signal import signal, SIGINT

class Distance_check:

    def __init__(self):
        self.sub = rospy.Subscriber("base_pose_ground_truth", Odometry, self.callback)
        self.total_distance = 0
        self.previous_x = 0
        self.previous_y = 0
        self.first_run = True

    def callback(self, data):
        if self.first_run:
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
            self.first_run = False
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        d_increment = math.dist((self.previous_x, self.previous_y), (x,y))
        self.total_distance = self.total_distance + d_increment
        #print(f"Total distance traveled is {self.total_distance}")
        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y

def print_final_dist():
    print(f"Final distance is {dist.total_distance}")

def handler(a, b):
    print_final_dist()

if __name__ == '__main__':
    rospy.init_node('move_turtlebot', anonymous=True)
    dist = Distance_check()
    # spin() simply keeps python from exiting until this node is stopped

    signal(SIGINT, handler)
    rospy.on_shutdown(print_final_dist)
    rospy.spin()
    
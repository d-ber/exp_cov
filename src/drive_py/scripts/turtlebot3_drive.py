#!/usr/bin/env python
#CC BY-NC license
import rospy
from std_msgs.msg import String
from  geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

DEG2RAD=(math.pi / 180.0)
RAD2DEG=(180.0 / math.pi)

CENTER=0
LEFT=1
RIGHT=2

LINEAR_VELOCITY=0.3
ANGULAR_VELOCITY=1.5

GET_TB3_DIRECTION=0
TB3_DRIVE_FORWARD=1
TB3_RIGHT_TURN=2
TB3_LEFT_TURN=3

class Turtlebot3Drive():
     
  def odomMsgCallBack(self, msg):
    siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
    self.tb3_pose = math.atan2(siny, cosy)

  def laserScanMsgCallBack(self, msg):
    scan_angle = [0, 30, 330]

    for num in range(3):
      if msg.ranges[scan_angle[num]] == float('inf'):
        self.scan_data[num] = msg.range_max
      else:
       self.scan_data[num] = msg.ranges[scan_angle[num]]

  def __init__(self):
    cmd_vel_topic_name = "/cmd_vel" # todo: parametrizza nel file launch
    # inizializza variabili
    self.escape_range       = 30.0 * DEG2RAD
    self.check_forward_dist = 0.7
    self.check_side_dist    = 0.6
    self.scan_data          = [0.0, 0.0, 0.0]
    self.tb3_pose = 0.0
    self.prev_tb3_pose = 0.0
    self.turtlebot3_state_num = 0
    #inizializza publisher
    self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size=10)
    #inizializza subscriber
    self.laser_scan_sub  = rospy.Subscriber("scan", LaserScan, callback=self.laserScanMsgCallBack, queue_size=10)
    self.odom_sub  = rospy.Subscriber("odom", Odometry, callback=self.odomMsgCallBack, queue_size=10)

  def __del__(self):
    self.updatecommandVelocity(0, 0)


  def updatecommandVelocity(self, linear, angular):
    cmd_vel = Twist()

    cmd_vel.linear.x  = linear
    cmd_vel.angular.z = angular

    self.cmd_vel_pub.publish(cmd_vel)

  def controlLoop(self):
    if self.turtlebot3_state_num == GET_TB3_DIRECTION:
      if (self.scan_data[CENTER] > self.check_forward_dist):
        if (self.scan_data[LEFT] < self.check_side_dist):
          self.prev_tb3_pose = self.tb3_pose
          self.turtlebot3_state_num = TB3_RIGHT_TURN
        elif (self.scan_data[RIGHT] < self.check_side_dist):
          self.prev_tb3_pose = self.tb3_pose
          self.turtlebot3_state_num = TB3_LEFT_TURN
        else:
          self.turtlebot3_state_num = TB3_DRIVE_FORWARD
      if self.scan_data[CENTER] < self.check_forward_dist:
        self.prev_tb3_pose = self.tb3_pose
        self.turtlebot3_state_num = TB3_RIGHT_TURN

    elif self.turtlebot3_state_num == TB3_DRIVE_FORWARD:
      self.updatecommandVelocity(LINEAR_VELOCITY, 0)
      self.turtlebot3_state_num = GET_TB3_DIRECTION

    elif self.turtlebot3_state_num == TB3_RIGHT_TURN:
      if (abs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range):
        self.turtlebot3_state_num = GET_TB3_DIRECTION
      else:
        self.updatecommandVelocity(0, -1 * ANGULAR_VELOCITY)

    elif self.turtlebot3_state_num == TB3_LEFT_TURN:
      if (abs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range):
        self.turtlebot3_state_num = GET_TB3_DIRECTION
      else:
        self.updatecommandVelocity(0, ANGULAR_VELOCITY)

    else:
      self.turtlebot3_state_num = GET_TB3_DIRECTION

    return True
  


  
def turtlebot3_drive():
  turtlebot3_drive = Turtlebot3Drive()
  rospy.init_node('turtlebot3_drive', anonymous=True)
  rate = rospy.Rate(125) # 125 hz

  while not rospy.is_shutdown(): 
    turtlebot3_drive.controlLoop()
    rospy.spin()
    rate.sleep()




if __name__ == '__main__':
    try:
        turtlebot3_drive()
    except rospy.ROSInterruptException:
        pass

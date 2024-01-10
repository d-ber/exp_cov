#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class lidar_maxrange_filter:

    def __init__(self):
        #rospy.loginfo("inizializzo")
        self.pub = rospy.Publisher('scan_maxrange_filtered', LaserScan, queue_size=10)
        self.sub = rospy.Subscriber("scan_hokuyo", LaserScan, self.callback)

    def callback(self, scan):
        #rospy.loginfo("CALLBACK")
        filtered = scan 
        ranges = list(filtered.ranges)
        #for i in range(len(ranges)):
        #    if math.isnan(ranges[i]):
        #        #rospy.loginfo("corretto valore")
        #        ranges[i] = 4.0
        #filtered.ranges = tuple(ranges)
        filtered.range_max = 3.0
        self.pub.publish(filtered)

    
def main():

    rospy.init_node('lidar_maxrange_filter', anonymous=True)
    lf = lidar_maxrange_filter()
    #rospy.loginfo("sono nel main")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    main()

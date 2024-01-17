import scipy.stats as st
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import rospy

class disturb:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.sub = rospy.Subscriber("/disturb", Odometry, self.callback)
            # Disturb random values (in pixels)
        mean_movement = 0
        std_movement = 0.05
        self.norm_movement = st.norm(loc=mean_movement, scale=std_movement)
        mean_rotation = 0
        std_rotation = 1.5
        self.norm_rotation = st.norm(loc=mean_rotation, scale=std_rotation)
        rospy.loginfo("Started disturb node.")
    
    def callback(self, msg):
        rospy.logdebug("Disturb odometry message received, about to disturb.")
        reply = Pose2D()
        reply.x = msg.pose.pose.position.x + self.norm_movement.rvs()
        reply.y = msg.pose.pose.position.y + self.norm_movement.rvs()
        (_, _, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        reply.theta = yaw + self.norm_rotation.rvs()

        self.pub.publish(reply)
        rospy.loginfo("Robot disturbed.")

def main():
            
    rospy.init_node('disturb', anonymous=True)
    _ = disturb()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
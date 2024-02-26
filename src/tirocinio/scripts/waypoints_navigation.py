import rospy
import argparse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class waypoint_sender():

    def __init__(self, waypoints):
        self.pose_seq = list()
        x, y, z, w = quaternion_from_euler(0, 0, 0) # quaternion_from_euler(roll, pitch, yaw)
        for waypoint in waypoints:
            self.pose_seq.append(Pose(waypoint, Quaternion(x, y, z, w)))
        rospy.loginfo(f"Wyapoint sender started. Will send up to {len(self.pose_seq)} waypoints.")
        self.goal_cnt = 0
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Waypoint sender connected to move base server.")
        rospy.loginfo("Starting waypoint navigation.")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo(f"Goal pose {self.goal_cnt+1} is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo(f"Feedback for goal {self.goal_cnt}: {feedback}")
        rospy.loginfo(f"Feedback for goal pose {self.goal_cnt} received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        '''
        Reference for terminal status values: https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html
        uint8 PENDING         = 0   # The goal has yet to be processed by the action server
        uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
        uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                    #   and has since completed its execution (Terminal State)
        uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
        uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                    #    to some failure (Terminal State)
        uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                    #    because the goal was unattainable or invalid (Terminal State)
        uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                    #    and has not yet completed execution
        uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                    #    but the action server has not yet confirmed that the goal is canceled
        uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                    #    and was successfully cancelled (Terminal State)
        uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                    #    sent over the wire by an action server
        '''
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo(f"Goal pose {self.goal_cnt} received a cancel request after it started executing, completed execution.")
        elif status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal pose {self.goal_cnt} reached. Result is {result}") 
            if self.goal_cnt < len(self.pose_seq):
                self.movebase_client()
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return
        elif status == GoalStatus.ABORTED:
            rospy.loginfo(f"Goal pose {self.goal_cnt} was aborted by the Action Server")
            rospy.signal_shutdown(f"Goal pose {self.goal_cnt} aborted, shutting down!")
            return
        elif status == GoalStatus.REJECTED:
            rospy.loginfo(f"Goal pose {self.goal_cnt} has been rejected by the Action Server")
            rospy.signal_shutdown(f"Goal pose {self.goal_cnt} rejected, shutting down!")
            return
        elif status == GoalStatus.RECALLED:
            rospy.loginfo(f"Goal pose {self.goal_cnt} received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = self.goal_cnt
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo(f"Sending goal pose {self.goal_cnt} to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

def parse_args():
    parser = argparse.ArgumentParser(description='Send a list of waypoints to move_base.')
    parser.add_argument('-p', '--path', required=True, help="Path to the waypoints csv file.")
    return parser.parse_known_args()[0]

def read_waypoints(file_path):
    waypoints = list()
    with open(file_path, "r") as file:
        pos = [l.strip().split(",") for l in file.readlines()]
        [waypoints.append(Point(float(x.strip()), float(y.strip()), 0)) for (x, y) in pos] # on the ground, so height 0
    return waypoints

def main():

    args = parse_args()
    csv_file_path = args.path

    waypoints = read_waypoints(csv_file_path)
        
    rospy.init_node('waypoint_sender', anonymous=True)
    _ = waypoint_sender(waypoints)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")


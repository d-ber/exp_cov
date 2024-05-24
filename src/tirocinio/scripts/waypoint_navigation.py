import rospy
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from actionlib.action_client import ActionClient, CommState, get_name_of_constant
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan

class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2

SimpleGoalState.to_string = classmethod(get_name_of_constant)

class waypoint_sender():

    def __init__(self, waypoints):
        self.pose_seq = list()
        x, y, z, w = quaternion_from_euler(0, 0, 0) # quaternion_from_euler(roll, pitch, yaw)
        for waypoint in waypoints:
            self.pose_seq.append(Pose(waypoint, Quaternion(x, y, z, w)))
        rospy.loginfo(f"Waypoint sender started. Will send up to {len(self.pose_seq)} waypoints.")
        self.goal_cnt = 0
        #Create action client
        self.action_client = ActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.action_client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.wait_for_service('/move_base/make_plan')
        self.get_plan = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        self.odom_srv = rospy.Subscriber("/odom", Odometry, callback=self.save_odom)
        rospy.loginfo("Waypoint sender connected to move base server.")
        rospy.loginfo("Starting waypoint navigation.")
        self.MAX_WAIT = 120
        self.PLAN_CHECK_WAIT = 15
        self.PLAN_CHECK_MAX_RETRIES = 3
        self.simple_state = SimpleGoalState.DONE
        self.send_goal()
        rospy.spin()

    def save_odom(self, msg):
        self.odom = msg.pose.pose

    def active_cb(self):
        rospy.loginfo(f"Goal pose {self.goal_cnt} is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo(f"Feedback for goal {self.goal_cnt}: {feedback}")
        rospy.loginfo(f"Feedback for goal pose {self.goal_cnt} received")
        if abs(rospy.Time.now().secs - self.goal_start) >= self.MAX_WAIT:
            if self.goal_cnt < len(self.pose_seq):
                rospy.loginfo(f"Timeout reached for current goal. Skipping to next one.")
                self.goal_cnt += 1
                self.send_goal()
            else:
                rospy.loginfo(f"Timeout reached for last goal. Waypoint navigation ended.")
                rospy.signal_shutdown("Waypoint navigation ended.")
                return
        elif abs(rospy.Time.now().secs - self.last_plan_check) > self.PLAN_CHECK_WAIT:
            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = rospy.Time.now() 
            start.pose = self.odom
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now() 
            goal.pose = self.pose_seq[self.goal_cnt]
            tolerance = 0.5
            no_plan = True
            try:
                plan = self.get_plan(start, goal, tolerance)
                #rospy.loginfo(f"plan:\n{plan}")
                no_plan = len(plan.plan.poses) == 0
            except Exception as e:
                no_plan = True
                rospy.loginfo(f"Plan check {self.plan_check_retries} out of {self.PLAN_CHECK_MAX_RETRIES} failed with exception.")
                rospy.loginfo(f"start:\n{start}\ngoal:\n{goal}\ntolerance:\n{tolerance}\nexception:\n{e}")
            finally:
                if no_plan:
                    rospy.loginfo(f"Plan check {self.plan_check_retries} out of {self.PLAN_CHECK_MAX_RETRIES} failed without plan.")
                    self.plan_check_retries += 1
                else:
                    rospy.loginfo(f"Plan check {self.plan_check_retries} out of {self.PLAN_CHECK_MAX_RETRIES} ok.")
                    self.plan_check_retries = 1
                self.last_plan_check = rospy.get_rostime().secs

    def handle_transition(self, gh):
        if gh != self.gh:
            rospy.logerr("Got a transition callback on a goal handle that we're not tracking")
            return

        comm_state = gh.get_comm_state()

        rospy.loginfo(f"Transitioning with comm_state '{CommState.to_string(comm_state)}', simple_state '{SimpleGoalState.to_string(self.simple_state)}'")

        error_msg = "Received comm state %s when in simple state %s with SimpleActionClient in NS %s" % \
            (CommState.to_string(comm_state), SimpleGoalState.to_string(self.simple_state), rospy.resolve_name(self.action_client.ns))

        if comm_state == CommState.ACTIVE:
            if self.simple_state == SimpleGoalState.PENDING:
                self.simple_state = SimpleGoalState.ACTIVE
                self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                self.simple_state = SimpleGoalState.ACTIVE
                self.active_cb()
        elif comm_state == CommState.RECALLING:
            if self.simple_state != SimpleGoalState.PENDING:
                rospy.logerr(error_msg)
        elif comm_state == CommState.PREEMPTING:
            if self.simple_state == SimpleGoalState.PENDING:
                self.simple_state = SimpleGoalState.ACTIVE
                self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.DONE:
            if self.simple_state in [SimpleGoalState.PENDING, SimpleGoalState.ACTIVE]:
                self.done_cb(gh.get_goal_status(), gh.get_result())
                self.simple_state = SimpleGoalState.DONE
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr("SimpleActionClient received DONE twice")

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
            if self.goal_cnt < len(self.pose_seq):
                rospy.loginfo(f"Goal pose {self.goal_cnt-1} received a cancel request after it started executing, completed execution.")
            else:
                rospy.loginfo("Final goal preempted!")
                rospy.signal_shutdown("Final goal preempted!")
                return
        elif status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal pose {self.goal_cnt-1} reached.") 
            if self.goal_cnt < len(self.pose_seq):
                self.send_goal()
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return
        elif status == GoalStatus.ABORTED:
            rospy.loginfo(f"Goal pose {self.goal_cnt-1} was aborted by the Action Server. Skipping it")
            if self.goal_cnt < len(self.pose_seq):
                self.send_goal()
            else:
                rospy.loginfo("Final goal pose aborted.")
                rospy.signal_shutdown("Final goal pose aborted.")
            return
        elif status == GoalStatus.REJECTED:
            if self.goal_cnt < len(self.pose_seq):
                rospy.loginfo(f"Goal pose {self.goal_cnt-1} has been rejected by the Action Server")
                rospy.signal_shutdown(f"Goal pose {self.goal_cnt-1} rejected, shutting down!")
            else:
                rospy.loginfo("Final goal pose rejected.")
                rospy.signal_shutdown("Final goal pose rejected.")
            return
        elif status == GoalStatus.RECALLED:
            if self.goal_cnt < len(self.pose_seq):
                rospy.loginfo(f"Goal pose {self.goal_cnt-1} received a cancel request before it started executing, successfully cancelled!")
            else:
                rospy.loginfo("Final goal pose recalled.")
                rospy.signal_shutdown("Final goal pose recalled.")
                return

    def handle_feedback(self, gh, feedback):
        if not self.gh:
            # this is not actually an error - there can be a small window in which old feedback
            # can be received between the time this variable is reset and a new goal is
            # sent and confirmed
            return
        if gh != self.gh:
            rospy.logerr("Got a feedback callback on a goal handle that we're not tracking. %s vs %s" %
                         (self.gh.comm_state_machine.action_goal.goal_id.id, gh.comm_state_machine.action_goal.goal_id.id))
            return
        self.feedback_cb(feedback)

    def get_current_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = self.goal_cnt
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        return goal

    def send_goal(self):
        goal = self.get_current_goal()
        rospy.loginfo(f"Sending goal pose {self.goal_cnt} to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))       
        self.gh = None
        self.simple_state = SimpleGoalState.PENDING
        self.gh = self.action_client.send_goal(goal, self.handle_transition, self.handle_feedback)
        self.goal_start = rospy.get_rostime().secs
        self.last_plan_check = self.goal_start
        self.plan_check_retries = 1

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


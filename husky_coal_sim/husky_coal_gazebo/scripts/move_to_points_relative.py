#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf2_ros
from math import radians
from tf.transformations import quaternion_from_euler

class MoveToPoints:
    def __init__(self):
        rospy.init_node('move_to_points_relative')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        self.points = rospy.get_param('/move_to_points_relative/p_seq') #Gets the sequence of nodes
  
        self.initial_position = self.get_initial_position()

        # Send goals
        self.send_goals(self.points)
        
        rospy.loginfo("All goals processed, shutting down node...")
        rospy.signal_shutdown("Finished moving to all points")

    def get_initial_position(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1)  # Give some time for the transform to be available
        try:
            trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
            return (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('Failed to get initial position: %s' % str(e))
            return None

    def send_goals(self, points):
        if len(points) % 3 != 0:
            rospy.logerr("p_seq length should be divisible by 3")
            return

        initial_x, initial_y, initial_z = self.initial_position

        for i in range(0, len(points), 3):
            x, y, yaw_deg = points[i], points[i+1], points[i+2]
            yaw_rad = radians(yaw_deg)

            #Sets the goal to be relative to the initial position
            #Change these lines to make it absolute position. 
            goalX = initial_x + x
            goalY = initial_y + y

            #sets orientation
            quaternion = quaternion_from_euler(0, 0, yaw_rad)

            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = goalX
            pose.pose.position.y = goalY
            pose.pose.position.z = initial_z  #Assumes z
            pose.pose.orientation = Quaternion(*quaternion)

            # Send goal and wait for result
            goal = MoveBaseGoal()
            goal.target_pose = pose
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_state()
            rospy.loginfo(f"Goal sent to ({goalX}, {goalY}, {yaw_deg} degrees): Result {result}")

if __name__ == '__main__':
    try:
        mover = MoveToPoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")


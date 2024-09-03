#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class RobotButler:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_butler', anonymous=True)

        # Publisher to send goals to the move_base node
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Subscriber to receive orders from the /order topic
        self.order_sub = rospy.Subscriber('/order', String, self.order_callback)

        # Waypoints (coordinates should be set according to your Gazebo environment)
        self.waypoints = {
            'kitchen': self.create_waypoint(4.66, -4.06, 0.0),
            'table1': self.create_waypoint(4.31, 4.45, 0.0),
            'table2': self.create_waypoint(-2.31,-3.31, 0.0),
            'table3': self.create_waypoint(-4.45, 3.64, 0.0),
        }

        # Track the current order and state
        self.current_order = None
        self.state = 'IDLE'  # States: 'IDLE', 'NAVIGATING_TO_KITCHEN', 'NAVIGATING_TO_TABLE'

        rospy.loginfo("Robot Butler Initialized")

    def create_waypoint(self, x, y, yaw, frame_id='map'):
        """Creates and returns a PoseStamped message with given coordinates."""
        waypoint = PoseStamped()
        waypoint.header.frame_id = frame_id
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.orientation.w = 1.0  # Assumes a flat environment, only setting w component
        return waypoint

    def order_callback(self, msg):
        """Callback function for the /order topic."""
        order = msg.data.strip()
        rospy.loginfo(f"Received order: {order}")

        if order == "kitchen":
            self.current_order = None
            self.navigate_to('kitchen')
            self.state = 'NAVIGATING_TO_KITCHEN'
        elif order in ['table1', 'table2', 'table3']:
            self.current_order = order
            self.navigate_to('kitchen')
            self.state = 'NAVIGATING_TO_KITCHEN'
        else:
            rospy.logwarn(f"Unknown order: {order}")

    def navigate_to(self, location):
        """Publishes the goal to the move_base node."""
        if location in self.waypoints:
            goal = self.waypoints[location]
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            rospy.loginfo(f"Navigating to {location}")
        else:
            rospy.logwarn(f"Waypoint {location} not found")

    def update_state(self):
        """Checks if the robot has reached its goal and updates the state accordingly."""
        if self.state == 'NAVIGATING_TO_KITCHEN':
            if self.has_reached_goal(self.waypoints['kitchen']):
                if self.current_order:
                    rospy.loginfo(f"Reached the kitchen, now proceeding to {self.current_order}.")
                    self.navigate_to(self.current_order)
                    self.state = 'NAVIGATING_TO_TABLE'
                else:
                    rospy.loginfo("Reached the kitchen.")
                    self.state = 'IDLE'
        elif self.state == 'NAVIGATING_TO_TABLE':
            if self.has_reached_goal(self.waypoints[self.current_order]):
                rospy.loginfo(f"Order delivered to {self.current_order}")
                self.state = 'IDLE'

    def has_reached_goal(self, waypoint):
        """Placeholder function to check if the robot has reached the goal."""
        # TODO: Implement logic to check if the robot has reached the goal
        return True  # Replace with actual checking logic

    def run(self):
        """Main loop of the robot butler."""
        rate = rospy.Rate(10)  # 10Hz update rate
        while not rospy.is_shutdown():
            self.update_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        robot_butler = RobotButler()
        robot_butler.run()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import AiapaecRelay
import curses


class RelayClientNode(Node):
    def __init__(self):
        super().__init__("relay_client")
        self.relay_client_ = ActionClient(
            self, 
            AiapaecRelay, 
            "aiapaec_relay")

    def send_goal(self, status, period):
        # Wait for the server
        self.relay_client_.wait_for_server()

        # Create a goal
        goal = AiapaecRelay.Goal()
        goal.status = status
        goal.period = period

        # Send the goal
        self.get_logger().info(f"Sending goal with status {status}")
        self.relay_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Result: {result.status_result}")

    def goal_feedback_callback(self, feedback_msg):
        status = feedback_msg.feedback.status_feedback
        self.get_logger().info(f"Got feedback: {status}")


def get_user_input(stdscr, node):
    # Clear screen
    stdscr.clear()
    stdscr.refresh()

    # Instructions
    stdscr.addstr("Press 'q' to quit, '0' to turn off relay, '1' to turn on relay.\n")
    stdscr.refresh()

    while True:
        # Read a single key press
        key = stdscr.getch()

        if key == ord('q'):
            break  # Exit the loop if 'q' is pressed

        elif key == ord('0'):
            node.send_goal(0, 0.01)  # Send 'off' state
            stdscr.addstr("Relay turned off\n")
        
        elif key == ord('1'):
            node.send_goal(1, 0.01)  # Send 'on' state
            stdscr.addstr("Relay turned on\n")
        
        # Refresh the screen to show feedback
        stdscr.refresh()


def main(args=None):
    rclpy.init(args=args)
    
    node = RelayClientNode()

    # Start curses mode for key input
    curses.wrapper(get_user_input, node)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from robot_action_interfaces.srv import GetRobotPose
from ddg_multi_robot_srvs.srv import GetMultiPlan
from robot_action_interfaces.action import StateMachine, DockUndock, Navigate, MissionControl
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from copy import deepcopy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time
from threading import Event


class MissionControlActionServerSingle(Node):

    def __init__(self):
        super().__init__('state_machine_action_server')
        self.get_logger().info("Starting State Machine Action Server")

        action_server_name = "MissionControl"
        self.get_logger().info(f"Mission Control Action Server Name is {action_server_name}")
        robot1_state_machine_name = "StateMachine"
        self.get_logger().info(f"Robot1 StateMachine Server being used for client is {robot1_state_machine_name}")

        self.callback_group = ReentrantCallbackGroup()
        # Construct the action server
        self._action_server = ActionServer(
            self,
            MissionControl,
            action_server_name,
            self.execute_callback,
            callback_group=self.callback_group)

        # Global Planner Service Client
        self._global_planner_client = self.create_client(GetMultiPlan,
                                                         '/multi_robot_planner/get_plan',
                                                         callback_group=self.callback_group)
        self.get_plan_request = GetMultiPlan.Request()

        while not self._global_planner_client.wait_for_service(timeout_sec=100.0):
            self.get_logger().warn("DDG Multi Robot Planner Service not available, waiting...")

        # Construct the action client (node and name should be same as defined in action server)
        self._robot_1_state_machine_client = ActionClient(self, StateMachine, robot1_state_machine_name)

        self.re_init_goal_states()
        self._mission_control_success = False

        self._robot_1_input_feedback_pose = None
        self._robot_1_input_feedback_state = None

        self._robot_1_action_complete = Event()
        self._robot_1_action_complete.clear()

    #! Temporary Pose Subscribers ########################
        robot1_map_pose_topic = "/map_pose"
        self._robot_1_latest_pose = None
        self._robot_2_latest_pose = None

        self._robot_1_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            robot1_map_pose_topic,  # Topic on which pose is being relayed
            self._robot_1_pose_callback,
            10  # Adjust the queue size as needed
        )

        # Hardcode robot_2_latest_pose as we're not reading robot2 in single robot case
        pose = PoseWithCovarianceStamped()
        self._robot_2_latest_pose = pose
    #! ######################################################

    def stop_planner_client_thread(self):
        self.shutdown_event.set()
        self.planner_client_thread.join()

    def get_waypoints_from_planner(self, future):
        if future.done(): #and get_plan_future.result().success:
            self.get_logger().info(f"received the plan \n {future.result().plan}")
            self.combined_waypoints = future.result().plan
        self._robot_1_action_complete.set()


    def _robot_1_pose_callback(self, msg):
        self._robot_1_latest_pose = msg

    def re_init_goal_states(self):
        self._robot_1_mission_success = False
        self._goal_accepted = None
        self._goal_reached = None
        self.combined_waypoints = None


    def get_final_result(self, success_status):
        result = MissionControl.Result()
        result.success = success_status
        return result


    def publish_mission_control_feedback(self):
        # publish feedback to high level action servers
        output_feedback_msg = MissionControl.Feedback()
        output_feedback_msg.pose_feedback = [self._robot_1_input_feedback_pose, self._robot_2_latest_pose]
        output_feedback_msg.state_feedback = [self._robot_1_input_feedback_state, "Robot 2 not used"]
        self._mission_control_goal_handle.publish_feedback(output_feedback_msg)
        pass


    ########## Send Goals to Individual Robots ################################################


    def robot_1_send_goal(self, robot_1_goal_package):
        self.get_logger().info('Calling Robot 1 Action Server...')

        try:
            self._robot_1_state_machine_client.wait_for_server(timeout_sec=5)
        except:
            self.get_logger().error('Timeout: Action server not available, waited for 5 seconds')
            return

        self._send_goal_future = self._robot_1_state_machine_client.send_goal_async(
                                        robot_1_goal_package,
                                        feedback_callback=self.robot_1_client_feedback_callback)
        self._send_goal_future.add_done_callback(self.robot_1_client_goal_response_callback)

    ########### Robot1 Functions ##########################################################


    def robot_1_client_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Robot 1 Goal rejected :(')
            return

        self.get_logger().info('Robot 1 Goal accepted :)')
        self._goal_accepted = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot_1_client_get_result_callback)


    def robot_1_client_get_result_callback(self, future):
        result = future.result().result
        result_string = str(result.success)
        self.get_logger().info(f'Robot 1 Result: {result_string}')
        if(result_string == "True"):
            self.get_logger().info(f"Robot 1 Goal Reached")
            self._robot_1_mission_success = True
            self._robot_1_action_complete.set()
        else:
            self._robot_1_mission_success = False
            self.get_logger().error(f"Robot 1 Goal Not Reached!")
            self._robot_1_action_complete.set()

        return


    def robot_1_client_feedback_callback(self, input_feedback_msg):
        # get feedback from low level action server
        self._robot_1_input_feedback_pose = input_feedback_msg.feedback.pose_feedback
        self._robot_1_input_feedback_state = input_feedback_msg.feedback.state_feedback
        input_feedback_pose_x = str(round(self._robot_1_input_feedback_pose.pose.pose.position.x, 2))
        input_feedback_pose_y = str(round(self._robot_1_input_feedback_pose.pose.pose.position.y, 2))
        self.get_logger().info(f'Received feedback: robot_1 pos x={input_feedback_pose_x},\
                                 robot_1 pos y = {input_feedback_pose_y}')
        self.get_logger().info(f'Received feedback: robot_1 state={self._robot_1_input_feedback_state}')

        self.publish_mission_control_feedback()


    ############### MAIN LOOP START ################################################
    def execute_callback(self, goal_handle):
        """
        Each Robot Task will be split into Undocking -> Navigation -> Docking
        """
        self.re_init_goal_states()
        self._mission_control_goal_handle = goal_handle

        # INPUT FROM FLEET MANAGEMENT
        dock_ids = goal_handle.request.robot_specific_dock_ids
        self.get_logger().info(f'Input dock IDs are {dock_ids}')

        # DEFINE END GOAL POINTS FOR FROM GLOBAL PLANNER
        end_goal_robot1 = PoseStamped()
        end_goal_robot2 = PoseStamped()

        end_goal_robot1.header.stamp = self.get_clock().now().to_msg()
        end_goal_robot2.header.stamp = self.get_clock().now().to_msg()

        end_goal_robot1.pose.position.x = 12.0
        end_goal_robot1.pose.position.y = -2.1
        end_goal_robot2.pose.position.x = 0.92
        end_goal_robot2.pose.position.y = -4.0

        # DEFINE START GOAL POINTS FOR FROM GLOBAL PLANNER

        start_goal_robot1 = PoseStamped()
        start_goal_robot2 = PoseStamped()

        start_goal_robot1.header = self._robot_1_latest_pose.header
        start_goal_robot1.pose = self._robot_1_latest_pose.pose.pose
        start_goal_robot2.header = self._robot_2_latest_pose.header
        start_goal_robot2.pose = self._robot_2_latest_pose.pose.pose
        start_goal_robot2.pose.position.x = 0.0
        start_goal_robot2.pose.position.x = -4.0

        self.get_plan_request.start = [start_goal_robot1, start_goal_robot2]
        self.get_plan_request.goal = [end_goal_robot1, end_goal_robot2]

        # get plan from planner
        self._robot_1_action_complete.clear()
        get_plan_future = self._global_planner_client.call_async(self.get_plan_request)
        get_plan_future.add_done_callback(self.get_waypoints_from_planner)
        self._robot_1_action_complete.wait()
        if self.combined_waypoints is None:
            goal_handle.succeed()
            return self.get_final_result(False)

        robot_1_goal_package = StateMachine.Goal()
        robot_1_goal_package.start_dock_id = 1 #! HARDCODED FOR NOW
        robot_1_goal_package.end_dock_id = 2 #! HARDCODED FOR NOW
        robot_1_goal_package.goals = self.combined_waypoints[0].poses

        ######### Give Goals to both robots and wait ###########
        self._robot_1_action_complete.clear()
        self.robot_1_send_goal(robot_1_goal_package)
        self._robot_1_action_complete.wait()
        self.get_logger().info('Got result')

        goal_handle.succeed()

        if self._robot_1_mission_success:
            self.get_logger().info("Returning Success")
            return self.get_final_result(True)
        else:
            return self.get_final_result(False)


def MissionControlServerSingle(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)
    executor = MultiThreadedExecutor()
    # start the MotionActionServer
    mission_control_action_server = MissionControlActionServerSingle()
    rclpy.spin(mission_control_action_server, executor)

if __name__ == '__main__':
    MissionControlServerSingle()
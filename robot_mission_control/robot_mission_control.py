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
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from threading import Event, Lock

class MissionControlActionServer(Node):

    def __init__(self):
        super().__init__('state_machine_action_server')
        self.get_logger().info("Starting State Machine Action Server")

        self.declare_parameter('robot1_namespace_param', 'robot1')
        robot1_namespace = self.get_parameter('robot1_namespace_param').get_parameter_value().string_value
        self.get_logger().info(f"robot1 namespace is {robot1_namespace}")

        self.declare_parameter('robot2_namespace_param', 'robot2')
        robot2_namespace = self.get_parameter('robot2_namespace_param').get_parameter_value().string_value
        self.get_logger().info(f"robot2 namespace is {robot2_namespace}")

        action_server_name = "MissionControl"
        self.get_logger().info(f"State Machine Action Server Name is {action_server_name}")
        robot1_state_machine_name = robot1_namespace + "/StateMachine"
        robot2_state_machine_name = robot2_namespace + "/StateMachine"
        self.get_logger().info(f"Robot1 StateMachine Server being used for client is {robot1_state_machine_name}")
        self.get_logger().info(f"Robot2 StateMachine Server being used for client is {robot2_state_machine_name}")

        self.declare_parameter(
            "mission_params",
            os.path.join(
                get_package_share_directory("robot_mission_control"),
                "config",
                "params.yaml",
            ),
        )
        param_file_name = (
            self.get_parameter("mission_params").get_parameter_value().string_value
        )

        self.mission_params = yaml.safe_load(open(param_file_name))

        self.callback_group = ReentrantCallbackGroup()
        # Construct the action server
        self._action_server = ActionServer(
            self,
            MissionControl,
            action_server_name,
            execute_callback=self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group)

        # Global Planner Service Client
        self._global_planner_client = self.create_client(GetMultiPlan,
                                                         '/multi_robot_planner/get_plan',
                                                         callback_group=self.callback_group)
        self.get_plan_request = GetMultiPlan.Request()

        while not self._global_planner_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("DDG Multi Robot Planner Service not available, waiting...")

        # Construct the action client (node and name should be same as defined in action server)
        self._robot_1_state_machine_client = ActionClient(self, StateMachine, robot1_state_machine_name)
        self._robot_2_state_machine_client = ActionClient(self, StateMachine, robot2_state_machine_name)

        self.re_init_goal_states()

        self._robot_1_action_complete = Event()
        self._robot_2_action_complete = Event()
        self._get_waypoints_complete = Event()
        self._robot_1_action_complete.clear()
        self._robot_2_action_complete.clear()
        self._get_waypoints_complete.clear()
        self._goal_lock = Lock()
        self._mission_control_goal_handle = None

    ######### Pose Subscribers ########################
        robot1_map_pose_topic = robot1_namespace + "/map_pose_mirror"
        robot2_map_pose_topic = robot2_namespace + "/map_pose_mirror"
        self._robot_1_latest_pose = None
        self._robot_2_latest_pose = None

        self._robot_1_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            robot1_map_pose_topic,  # Topic on which pose is being relayed
            self._robot_1_pose_callback,
            10  # Adjust the queue size as needed
        )
        self._robot_2_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            robot2_map_pose_topic,  # Topic on which pose is being relayed
            self._robot_2_pose_callback,
            10  # Adjust the queue size as needed
        )

    def _robot_1_pose_callback(self, msg):
        self._robot_1_latest_pose = msg

    def _robot_2_pose_callback(self, msg):
        self._robot_2_latest_pose = msg
    ####################################################

    def re_init_goal_states(self):
        self._robot_1_mission_success = False
        self._robot_2_mission_success = False
        self._goal_accepted = None
        self._goal_reached = None
        self.combined_waypoints = None
        self._robot_1_input_feedback_pose = None
        self._robot_1_input_feedback_state = None
        self._robot_2_input_feedback_pose = None
        self._robot_2_input_feedback_state = None


    def get_final_result(self, success_status : int, status_text=None):
        result = MissionControl.Result()
        result.status_code = success_status
        if status_text is not None:
            result.text = status_text
        return result

    def get_waypoints_from_planner(self, future):
        if future.done():
            self.get_logger().info(f"received the plan \n {future.result().plan}")
            self.combined_waypoints = future.result().plan
        self._get_waypoints_complete.set()

    def publish_mission_control_feedback(self):
        # check if both robot1 and robot1 feedback has arrived:
        if self._robot_1_input_feedback_pose and self._robot_1_input_feedback_state \
            and self._robot_2_input_feedback_pose and self._robot_2_input_feedback_state:
            # publish feedback to high level action servers
            output_feedback_msg = MissionControl.Feedback()
            output_feedback_msg.pose_feedback = [self._robot_1_input_feedback_pose, self._robot_2_input_feedback_pose]
            output_feedback_msg.state_feedback = [self._robot_1_input_feedback_state, self._robot_2_input_feedback_state]
            if self._mission_control_goal_handle is not None:
                self._mission_control_goal_handle.publish_feedback(output_feedback_msg)


    def euclidean_distance(self, pos1_object, pos2_object):

        # accessor function
        def get_pose(pos_object):
            if isinstance(pos_object, PoseStamped):
                return pos_object.pose
            elif isinstance(pos_object, PoseWithCovarianceStamped):
                return pos_object.pose.pose
            else:
                raise ValueError("Input pos_object is not a valid type (Pose or PoseWithCovariance)")

        # Extract the positions from the poses
        pos1 = get_pose(pos1_object)
        pos2 = get_pose(pos2_object)

        # Calculate the Euclidean distance
        dx = pos1.position.x - pos2.position.x
        dy = pos1.position.y - pos2.position.y

        distance = math.sqrt(dx**2 + dy**2)
        return distance

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


    def robot_2_send_goal(self, robot_2_goal_package):
        self.get_logger().info('Calling Robot 2 Action Server...')

        try:
            self._robot_2_state_machine_client.wait_for_server(timeout_sec=5)
        except:
            self.get_logger().error('Timeout: Action server not available, waited for 5 seconds')
            return

        self._send_goal_future = self._robot_2_state_machine_client.send_goal_async(
                                        robot_2_goal_package,
                                        feedback_callback=self.robot_2_client_feedback_callback)
        self._send_goal_future.add_done_callback(self.robot_2_client_goal_response_callback)


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
        # self.get_logger().info(f'Received feedback: robot_1 pos x={input_feedback_pose_x},\
        #                          robot_1 pos y = {input_feedback_pose_y}')
        # self.get_logger().info(f'Received feedback: robot_1 state={self._robot_1_input_feedback_state}')

        self.publish_mission_control_feedback()

    ########### Robot2 Functions ##########################################################

    def robot_2_client_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Robot 2 Goal rejected :(')
            return

        self.get_logger().info('Robot 2 Goal accepted :)')
        self._goal_accepted = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot_2_client_get_result_callback)

    def robot_2_client_get_result_callback(self, future):
        result = future.result().result
        result_string = str(result.success)
        self.get_logger().info(f'Robot 2 Result: {result_string}')
        if(result_string == "True"):
            self.get_logger().info(f"Robot 2 Goal Reached")
            self._robot_2_mission_success = True
            self._robot_2_action_complete.set()
        else:
            self._robot_2_mission_success = False
            self.get_logger().error(f"Robot 2 Goal Not Reached!")
            self._robot_2_action_complete.set()

        return

    def robot_2_client_feedback_callback(self, input_feedback_msg):
        # get feedback from low level action server
        self._robot_2_input_feedback_pose = input_feedback_msg.feedback.pose_feedback
        self._robot_2_input_feedback_state = input_feedback_msg.feedback.state_feedback
        input_feedback_pose_x = str(round(self._robot_2_input_feedback_pose.pose.pose.position.x, 2))
        input_feedback_pose_y = str(round(self._robot_2_input_feedback_pose.pose.pose.position.y, 2))
        # self.get_logger().info(f'Received feedback: robot_2 pos x={input_feedback_pose_x},\
        #                          robot_2 pos y = {input_feedback_pose_y}')
        # self.get_logger().info(f'Received feedback: robot_2 state={self._robot_2_input_feedback_state}')

        self.publish_mission_control_feedback()

    ############## Handle only one goal at a time #################################

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._mission_control_goal_handle is not None and self._mission_control_goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._mission_control_goal_handle.abort()
                time.sleep(0.5)

            print("Setting new mission_control goal handle")
            self._mission_control_goal_handle = goal_handle

        goal_handle.execute()

    ############### MAIN LOOP START ################################################
    def execute_callback(self, goal_handle):
        """
        Each Robot Task will be split into Undocking -> Navigation -> Docking
        """
        self.re_init_goal_states()

        # INPUT FROM FLEET MANAGEMENT
        dock_ids = goal_handle.request.robot_specific_dock_ids
        undock_flags = goal_handle.request.robot_specific_undock_flags
        self.get_logger().info(f'Input dock IDs are {dock_ids}')

        robot_1_start_dock_id = dock_ids[0]
        robot_1_goal_dock_id = dock_ids[1]
        robot_2_start_dock_id = dock_ids[2]
        robot_2_goal_dock_id = dock_ids[3]
        robot_1_undock_flag = undock_flags[0]
        robot_2_undock_flag = undock_flags[1]
        self.get_logger().info(f"navigating robot_1 to dock ID {robot_1_goal_dock_id}")
        self.get_logger().info(f"navigating robot_2 to dock ID {robot_2_goal_dock_id}")
        self.get_logger().info(f"robot_1 undock flag : {robot_1_undock_flag}")
        self.get_logger().info(f"robot_2 undock flag : {robot_2_undock_flag}")

        # DEFINE END GOAL POINTS FOR FROM GLOBAL PLANNER
        end_goal_robot1 = PoseStamped()
        end_goal_robot2 = PoseStamped()

        end_goal_robot1.header.stamp = self.get_clock().now().to_msg()
        end_goal_robot1.pose.position.x = self.mission_params[robot_1_goal_dock_id]["position"]["x"]
        end_goal_robot1.pose.position.y = self.mission_params[robot_1_goal_dock_id]["position"]["y"]
        end_goal_robot1.pose.orientation.x = self.mission_params[robot_1_goal_dock_id]["orientation"]["x"]
        end_goal_robot1.pose.orientation.y = self.mission_params[robot_1_goal_dock_id]["orientation"]["y"]
        end_goal_robot1.pose.orientation.z = self.mission_params[robot_1_goal_dock_id]["orientation"]["z"]
        end_goal_robot1.pose.orientation.w = self.mission_params[robot_1_goal_dock_id]["orientation"]["w"]
        robot1_dock_lateral_bias = self.mission_params[robot_1_goal_dock_id]["dock_bias"]["lateral"]
        robot1_dock_forward_bias = self.mission_params[robot_1_goal_dock_id]["dock_bias"]["forward"]

        end_goal_robot2.header.stamp = self.get_clock().now().to_msg()
        end_goal_robot2.pose.position.x = self.mission_params[robot_2_goal_dock_id]["position"]["x"]
        end_goal_robot2.pose.position.y = self.mission_params[robot_2_goal_dock_id]["position"]["y"]
        end_goal_robot2.pose.orientation.x = self.mission_params[robot_2_goal_dock_id]["orientation"]["x"]
        end_goal_robot2.pose.orientation.y = self.mission_params[robot_2_goal_dock_id]["orientation"]["y"]
        end_goal_robot2.pose.orientation.z = self.mission_params[robot_2_goal_dock_id]["orientation"]["z"]
        end_goal_robot2.pose.orientation.w = self.mission_params[robot_2_goal_dock_id]["orientation"]["w"]
        robot2_dock_lateral_bias = self.mission_params[robot_2_goal_dock_id]["dock_bias"]["lateral"]
        robot2_dock_forward_bias = self.mission_params[robot_2_goal_dock_id]["dock_bias"]["forward"]

        # DEFINE START GOAL POINTS FOR FROM GLOBAL PLANNER

        # check to make sure we have current robot pose (robot 1)
        assert self._robot_1_latest_pose is not None
        start_goal_robot1 = PoseStamped()
        start_goal_robot1.header = self._robot_1_latest_pose.header
        start_goal_robot1.pose = self._robot_1_latest_pose.pose.pose

        # check to make sure we have current robot pose (robot 2)
        assert self._robot_2_latest_pose is not None
        start_goal_robot2 = PoseStamped()
        start_goal_robot2.header = self._robot_2_latest_pose.header
        start_goal_robot2.pose = self._robot_2_latest_pose.pose.pose

        if (robot_1_start_dock_id == 100) and (robot_1_goal_dock_id == 100):
            end_goal_robot1 = start_goal_robot1

        if (robot_2_start_dock_id == 100) and (robot_2_goal_dock_id == 100):
            end_goal_robot2 = start_goal_robot2

        # at this stage, we handled input cases: (A,B), (A,A), and (100,100)
        # check for conflicting goal poses
        goal_pose_distances = self.euclidean_distance(end_goal_robot1, end_goal_robot2)
        self.get_logger().info(f"euclidean distance is {goal_pose_distances}")
        if goal_pose_distances < 1.0 and (robot_1_goal_dock_id != 0 and robot_2_goal_dock_id != 0): # metres
            self.get_logger().info(f"Rejecting Goal due to conflicting end goal poses")
            goal_handle.abort()
            return self.get_final_result(400, "conflicting end goal poses")

        self.get_plan_request.start = [start_goal_robot1, start_goal_robot2]
        self.get_plan_request.goal = [end_goal_robot1, end_goal_robot2]

        # Ask Global Planner for Waypoints only if dock_IDs are not 0s
        if robot_1_goal_dock_id != 0 and robot_2_goal_dock_id != 0:
            self._get_waypoints_complete.clear()
            get_plan_future = self._global_planner_client.call_async(self.get_plan_request)
            get_plan_future.add_done_callback(self.get_waypoints_from_planner)
            self._get_waypoints_complete.wait()
            if self.combined_waypoints is None:
                goal_handle.succeed()
                return self.get_final_result(500, "no plan from global planner")
        else:
            self.combined_waypoints = [Path(), Path()]

        robot_1_goal_package = StateMachine.Goal()
        robot_1_goal_package.start_dock_id = robot_1_start_dock_id
        robot_1_goal_package.end_dock_id = robot_1_goal_dock_id
        robot_1_goal_package.goals = self.combined_waypoints[0].poses
        robot_1_goal_package.dock_lateral_bias = robot1_dock_lateral_bias
        robot_1_goal_package.dock_forward_bias = robot1_dock_forward_bias
        robot_1_goal_package.undock_flag = bool(robot_1_undock_flag)

        robot_2_goal_package = StateMachine.Goal()
        robot_2_goal_package.start_dock_id = robot_2_start_dock_id
        robot_2_goal_package.end_dock_id = robot_2_goal_dock_id
        robot_2_goal_package.goals = self.combined_waypoints[1].poses
        robot_2_goal_package.dock_lateral_bias = robot2_dock_lateral_bias
        robot_2_goal_package.dock_forward_bias = robot2_dock_forward_bias
        robot_2_goal_package.undock_flag = bool(robot_2_undock_flag)

        self.get_logger().info(f" robot_1_goal_package.undock_flag {robot_1_goal_package.undock_flag}")
        self.get_logger().info(f" robot_2_goal_package.undock_flag {robot_2_goal_package.undock_flag}")

        ######### Give Goals to both robots and wait ###########
        self.re_init_goal_states()
        self._robot_1_action_complete.clear()
        self._robot_2_action_complete.clear()
        robot1_goal_sent = False
        robot2_goal_sent = False

        ##### Conditions to check if user wants to control both robots or just one ########
        self._robot_1_mission_success = False
        self._robot_2_mission_success = False
        # if len(robot_1_goal_package.goals) > 2 or robot_1_goal_dock_id == 0:
        if robot_1_goal_dock_id != 100:
            self.robot_1_send_goal(robot_1_goal_package)
            self.get_logger().info("Starting Mission for Robot 1")
            robot1_goal_sent = True

        # if len(robot_2_goal_package.goals) > 2 or robot_2_goal_dock_id == 0:
        if robot_2_goal_dock_id != 100:
            self.robot_2_send_goal(robot_2_goal_package)
            self.get_logger().info("Starting Mission for Robot 2")
            robot2_goal_sent = True

        # wait for each robot to finish its mission
        while True:
            if not goal_handle.is_active:
                self.get_logger().info('GOAL ABORTED')
                return self.get_final_result(500, "mission aborted midway")

            # check for aborted or cancelled goal handle
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('GOAL CANCELLED, Cancelling low level actions')
                if robot1_goal_sent:
                    robot_1_goal_package.start_dock_id = 0
                    robot_1_goal_package.end_dock_id = 0
                    self.robot_1_send_goal(robot_1_goal_package)
                if robot2_goal_sent:
                    robot_2_goal_package.start_dock_id = 0
                    robot_2_goal_package.end_dock_id = 0
                    self.robot_2_send_goal(robot_2_goal_package)
                time.sleep(2) # wait for low level action serves to cancel
                return self.get_final_result(200, "cancelled successfully")

            condition_1 = self._robot_1_action_complete.is_set() if robot1_goal_sent else True
            condition_2 = self._robot_2_action_complete.is_set() if robot2_goal_sent else True

            if condition_1 and condition_2:
                break

            time.sleep(0.1)

        self.get_logger().info('Mission Complete')

        goal_handle.succeed()

        if robot1_goal_sent and robot2_goal_sent:
            if self._robot_1_mission_success and self._robot_2_mission_success:
                self.get_logger().info("Returning Success")
                return self.get_final_result(200, "success")
            else:
                return self.get_final_result(500, "mission failed midway")

        elif robot1_goal_sent and (not robot2_goal_sent):
            if self._robot_1_mission_success:
                self.get_logger().info("Returning Success")
                return self.get_final_result(200, "success")
            else:
                return self.get_final_result(500, "mission failed midway")

        elif (not robot1_goal_sent) and robot2_goal_sent:
            if self._robot_2_mission_success:
                self.get_logger().info("Returning Success")
                return self.get_final_result(200, "success")
            else:
                return self.get_final_result(500, "mission failed midway")
        else:
            return self.get_final_result(200, "success")

def MissionControlServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)
    executor = MultiThreadedExecutor()
    # start the MotionActionServer
    mission_control_action_server = MissionControlActionServer()
    rclpy.spin(mission_control_action_server, executor)

if __name__ == '__main__':
    MissionControlServer()
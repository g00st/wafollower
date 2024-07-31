import os
import rclpy
from rclpy.node import Node
import launch_testing
import pytest
import unittest
from time import sleep, time
import logging
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory
from launch_testing.asserts import assertExitCodes
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray
from miniprj_action_interfaces.action import Start

TURTLEBOT3_GAZEBO_PACKAGE = 'turtlebot3_gazebo'
MINIPR_PACKAGE = 'jg_213776_miniprj'
MINIPR_ACTION_PACKAGE='miniprj_action_interfaces'
MINIPR_LAUNCH_FILE = 'start_launch.py'
ACTION_FILE = 'Start.action'



class TestPackageExistence(unittest.TestCase):
    def test_packages_exist(self):
        # Verify that the required packages exist
        packages = [TURTLEBOT3_GAZEBO_PACKAGE, MINIPR_PACKAGE,MINIPR_ACTION_PACKAGE]
        for pkg in packages:
            pkg_path = get_package_share_directory(pkg)
            self.assertTrue(os.path.exists(pkg_path), f"Package {pkg} does not exist.")

    def test_files_exist(self):
        # Verify that the required files exist
        package_path = get_package_share_directory(MINIPR_PACKAGE)
        launch_file = os.path.join(package_path, 'launch', MINIPR_LAUNCH_FILE)
        self.assertTrue(os.path.exists(launch_file), f"Launch file {launch_file} does not exist.")

    def test_start_action_exists(self):
        package_path = get_package_share_directory(MINIPR_ACTION_PACKAGE)
        action_file_path = os.path.join(package_path, 'action', ACTION_FILE)
        self.assertTrue(os.path.exists(action_file_path), f"Action file {action_file_path} does not exist.")

        try:
            global Start
            from miniprj_action_interfaces.action import Start
        except ImportError:
            Start = None
            self.assertTrue(False, "Action interface not found")


@pytest.mark.launch_test
def generate_test_description():
    # Paths to the required launch files
    turtlebot3_gazebo_path = get_package_share_directory('minipr_test')
    package_path = get_package_share_directory(MINIPR_PACKAGE)
    launch_file = os.path.join(package_path, 'launch', MINIPR_LAUNCH_FILE)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_path, '/launch/testworld_launch.py']),
            launch_arguments={'x_pose': '0', 'y_pose': '0'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
        ),
        ReadyToTest(),
    ])


@pytest.mark.launch_test
class TestStudentNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_minipr(self):
        self.assertIsNotNone(self.node)

        # Test if action Server exists
        self.ActionClient = ActionClient(self.node, Start, 'start')
        available = self.ActionClient.wait_for_server(timeout_sec=10.0)
        self.assertTrue(available, "Action server not available within timeout period.")

        # Create a subscription to the model states topic
        self.position_subscription = self.node.create_subscription(
            PoseArray,
            '/model_states',
            self.model_state_callback,
            10,
        )

        # Send start Action
        self.Goal = Start.Goal()
        self.Goal.start_driving = True
        send_goal_future = self.ActionClient.send_goal_async(self.Goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        # Ensure goal was accepted
        if send_goal_future.result() is None:
            self.fail("Failed to send goal")

        # Initialize timeout
        start_time = time()
        timeout = 200  
        last_pos = {'x': 0, 'y': 0}
        self.position =  {'x': 0, 'y': 0}
        delta = 0.1
        inactive_counter = 20

        # Get result future for the goal
        result_future = send_goal_future.result().get_result_async()

        while True:
            # Spin once with a timeout to process callbacks
            rclpy.spin_once(self.node, timeout_sec=1.0)

            # Check elapsed time
            elapsed_time = time() - start_time
            if elapsed_time > timeout:
                self.assertTrue(False, "Test timed out after 20 seconds")
            
            # Update the position and check for inactivity
            if self.position:
                if (abs(last_pos['x'] - self.position['x']) < delta and
                    abs(last_pos['y'] - self.position['y']) < delta):
                    inactive_counter -= 1
                else:
                    inactive_counter = 40  # Reset the counter if movement is detected

                if inactive_counter <= 0:
                    self.assertTrue(False, "Robot was inactive for too long")
                last_pos = self.position

            #
                if self.position['x'] > 2.2:
                    self.assertTrue(True, "gap passed")
                    return 
            sleep(2)  # Adjust this sleep time as necessary


    def model_state_callback(self, msg):
        # Check if the burger model is in the message
        if 'burger' in msg.name:
            index = msg.name.index('burger')
            pose = msg.pose[index]
            position = pose.position
            # Update self.position with the burger's coordinates
            self.position = {
                'x': position.x,
                'y': position.y
            }


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(
        TestPackageExistence,
        TestStudentNode))
    return suite

if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    runner.run(suite())

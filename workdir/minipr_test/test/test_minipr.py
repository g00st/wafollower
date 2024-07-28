import os
import rclpy
from rclpy.node import Node
import launch_testing
import pytest
import unittest
from time import sleep
import logging
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory
from launch_testing.asserts import assertExitCodes
from rclpy.action import ActionClient


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
    turtlebot3_gazebo_path = get_package_share_directory(TURTLEBOT3_GAZEBO_PACKAGE)
    package_path = get_package_share_directory(MINIPR_PACKAGE)
    launch_file = os.path.join(package_path, 'launch', MINIPR_LAUNCH_FILE)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_path, '/launch/turtlebot3_house.launch.py']),
            launch_arguments={'x_pose': '-1.4', 'y_pose': '3.5'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
        ),
        ReadyToTest(),
    ])

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
        #self.timer = 60
        #sleep(self.timer)
        self.assertIsNotNone(self.node)
        #---------------------------------------------------------------------------------------------------------
        #test if action Server exists

        self.ActionClient = ActionClient(self.node, Start, 'start')
        available = self.ActionClient.wait_for_server(timeout_sec=10.0)
        self.assertTrue(available, "Action server not available within timeout period.")
        
        #----------------------------------------------------------------------------------------------------------
        #send start Action
        self.Goal = Start.



        # Additional assertion to demonstrate functionality
        self.assertTrue(True)
        

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestPackageExistence))
    suite.addTest(unittest.makeSuite(TestStudentNode))
    return suite

if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    runner.run(suite())

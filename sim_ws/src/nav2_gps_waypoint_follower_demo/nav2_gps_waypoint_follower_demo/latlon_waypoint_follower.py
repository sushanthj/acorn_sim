#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import time
import math
from pathlib import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from geographic_msgs.msg import GeoPose
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Load waypoints from a YAML file in config/demo_waypoints.yaml
        yaml_path = Path(get_package_share_directory('nav2_gps_waypoint_follower_demo')) / \
            'config' / 'demo_waypoints.yaml'
        self.waypoints = self.load_waypoints(yaml_path)

        # Create action client
        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Initialize result future
        self._get_result_future = None

        # Start the waypoint following loop
        self.follow_waypoints()

    def latLonYaw2Geopose(self, latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
        """
        Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
        """
        geopose = GeoPose()
        geopose.position.latitude = latitude
        geopose.position.longitude = longitude
        geopose.orientation = self.quaternion_from_euler(0.0, 0.0, yaw)
        return geopose

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

    def load_waypoints(self, filename):
        waypoints = []
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
            for wp in data['waypoints']:
                geopose = self.latLonYaw2Geopose(wp['latitude'], wp['longitude'], wp['yaw'])
                waypoints.append(geopose)
        return waypoints

    def geopose_to_pose_stamped(self, geopose: GeoPose) -> PoseStamped:
        """
        Converts GeoPose to PoseStamped
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'  # Assuming 'map' frame for the navigation
        pose_stamped.header.stamp = Time().to_msg()

        # Converting GeoPoint to position
        pose_stamped.pose.position.x = geopose.position.latitude  # This assumes latitude is converted to x
        pose_stamped.pose.position.y = geopose.position.longitude  # This assumes longitude is converted to y
        pose_stamped.pose.position.z = geopose.position.altitude

        # Orientation remains the same
        pose_stamped.pose.orientation = geopose.orientation

        return pose_stamped

    def follow_waypoints(self):
        rclpy.spin_once(self)
        self.get_logger().info("Waiting for action server to be available...")
        self._navigate_to_pose_client.wait_for_server()

        while rclpy.ok():
            for waypoint in self.waypoints:
                pose_stamped = self.geopose_to_pose_stamped(waypoint)
                self.navigate_to_pose(pose_stamped)
                self.get_logger().info(f"Navigating to waypoint: {pose_stamped}")
                while not self.goal_completed():
                    rclpy.spin_once(self)
                    time.sleep(1)

    def navigate_to_pose(self, pose_stamped: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self._send_goal_future = self._navigate_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self._get_result_future = None
        else:
            self.get_logger().info('Goal accepted')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def goal_completed(self):
        if self._get_result_future is None:
            return True
        return self._get_result_future.done()

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import sys
import time
from rclpy.time import Time

from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        geopose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            geopose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return geopose_wps

class GpsWpCommander():
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator()
        self.wp_parser = YamlWaypointParser(wps_file_path)

    def geopose_to_pose_stamped(self, geopose: GeoPose) -> PoseStamped:
        """
        Converts GeoPose to PoseStamped
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'  # Assuming map frame for the navigation
        pose_stamped.header.stamp = Time().to_msg()

        # Converting GeoPoint to position
        pose_stamped.pose.position.x = geopose.position.latitude  # Use appropriate transformation if necessary
        pose_stamped.pose.position.y = geopose.position.longitude  # Use appropriate transformation if necessary
        pose_stamped.pose.position.z = geopose.position.altitude

        # Orientation remains the same
        pose_stamped.pose.orientation = geopose.orientation

        return pose_stamped

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wps = self.wp_parser.get_wps()
        for geopose in wps:
            pose_stamped = self.geopose_to_pose_stamped(geopose)
            self.navigator.goToPose(pose_stamped)

            while not self.navigator.isTaskComplete():
                time.sleep(0.1)

            result = self.navigator.getResult()
            if result.code != BasicNavigator.TaskResult.SUCCEEDED:
                self.get_logger().error("Failed to reach waypoint, exiting.")
                break

        print("Waypoint following completed successfully")

def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()

if __name__ == "__main__":
    main()

import argparse

import rclpy

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node


class GazeboObjectSpawner(Node):
    """
    This class allows spawning objects in a Gazebo simulation environment using the Gazebo SpawnEntity service, see: http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/SpawnModel.html.
    Note that the XML/URDF has to be passed as a string i.e. read from file or elsewhere beforehand, could be read using `read_model_xml`
    
    ### Example
    ```ros2 run gazebo_satellite_terrain spawn_object my_model "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></visual></link></model></sdf>"```
    """

    def __init__(self, node_name: str = "GazeboSpawner") -> None:
        super().__init__(node_name)
        self.node_name = node_name
        self.client = None
        

    def init_client(self) -> None:
        self.client = self.create_client(SpawnEntity, 'spawn_entity') # http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/SpawnModel.html

        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info(f'{self.node_name}: Gazebo SpawnEntity service not available, trying again ...')


    def read_model_xml(self, urdf_path: str) -> str:
        """ """
        with open(urdf_path, "r") as g:
            return g.read()


    def spawn_object_gazebo(self, name: str, xml: str, pose: Pose = Pose(), verbose: bool = False) -> bool:
        """
        Spawn an object into Gazebo given xml and a pose
        ### Parameters
            name    : str,                    The name of the object to be spawned.
            xml     : str,                    The XML description of the object to be spawned.
            pose    : geometry_msgs.msg.Pose, Position and orientation of object
            verbose : bool (False)            Whether to print success 
        ### Returns
            success : bool                    Did we manage to spawn the object
        """
        # Create a spawner node 
        if not self.client:
            self.init_client()

        request = SpawnEntity.Request(
            name = name,
            xml = xml,
            initial_pose = pose
        )
        future = self.client.call_async(request)

        # Call the spawn 'server' to create the object
        while rclpy.ok():
            rclpy.spin_once(self)

            if not future.done():
                continue

            try:
                future.result()
                if verbose:
                    self.get_logger().info(f"{self.node_name}: Successfully spawned object: '{name}'")
                return True
            except Exception as e:
                self.get_logger().info(f"{self.node_name}: Service call to spawn object failed: {e}")
                return False


def main(args=None):
    rclpy.init(args=args)

    # Argument parsing
    parser = argparse.ArgumentParser(description = 'Arguments for making dataset')
    parser.add_argument('--name', type = str, help = 'Name of the object in gazebo')
    parser.add_argument('--xml', type = str, help = "The full XML of the object")
    parser.add_argument('--verbose', type = bool, default = False, help = "The full XML of the object")
    args, unknown = parser.parse_known_args(args=args)
    
    # Spawn object
    spawner = GazeboObjectSpawner()
    spawner.spawn_object_gazebo(name = args.name, xml = args.xml, verbose = args.verbose)

    spawner.destroy_node()
    rclpy.shutdown()


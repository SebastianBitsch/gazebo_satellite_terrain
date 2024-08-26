import os
import argparse

import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node

from gazebo_satellite_terrain.spawn_tile import GazeboSatelliteTileSpawner
from gazebo_satellite_terrain.objects import Map
from gazebo_satellite_terrain.coord import Coord
from gazebo_satellite_terrain.utils import generate_spiral_modes


class GazeboTerrainManager(Node):
    """¨
    TODO: Add a detailed explaination of what the file structure etc of the resources dir looks like.
    """

    def __init__(self, odom_topic: str, approx_world_origin: tuple, zoom: int, tile_size_pixels: int, map_size: int, prefetch_terrain: bool, prespawn_terrain: bool, node_name: str = "TerrainManager") -> None:
        """ """
        super().__init__(node_name)
        self.node_name = node_name
        
        self.map = Map(
            size = map_size,
            zoom = zoom,
            tile_size_pixels = tile_size_pixels,
            origin_approx = approx_world_origin
        )
        self.get_logger().info(str(self.map))
        
        self.spawner = GazeboSatelliteTileSpawner(self.map.tile_size_meters)

        self.create_materials_file(self.map)
        self.init_terrain(self.map, prefetch_terrain, prespawn_terrain)

        # Subscribe to the odometry message
        self.odom_subscription = self.create_subscription(
            msg_type = Odometry,
            topic = odom_topic,
            callback = self.odometry_callback,
            qos_profile = 10
        )


    def init_terrain(self, map: Map, prefetch_terrain: bool, prespawn_terrain: bool) -> None:
        if prefetch_terrain:
            self.download_map(map)
        if prespawn_terrain:
            self.spawn_map(map)


    def download_map(self, map: Map) -> None:
        """ Download the entire map """
        self.get_logger().info(f"{self.node_name}: Downloading (or getting cache of) {len(map.tiles)} tiles, please wait")
        for i in generate_spiral_modes(map.size):
            self.spawner.download_tile(map.tiles[i])

        self.get_logger().info(f"{self.node_name}: Done fetching tiles")


    def spawn_map(self, map: Map) -> None:
        """ Spawn the entire map """
        self.get_logger().info(f"{self.node_name}: Spawning {len(map.tiles)} tiles, please wait")
        for i in generate_spiral_modes(map.size):
            self.spawner.spawn_tile(map.tiles[i])

        self.get_logger().info(f"{self.node_name}: Done spawning tiles")


    def odometry_callback(self, odometry: Odometry) -> None:
        """
        Get the position of the object we are tracking to make sure there is terrain spawned underneath it,
        Wont matter if we have prespawned terrain at startup
        """
        pos = Coord(odometry.pose.pose.position.x, odometry.pose.pose.position.y)
        tile = self.map.tile_at_pos(pos)

        if tile is None:
            self.get_logger().info(f"{self.node_name}: Drone is outside bounds of map")
            return

        # If we are entering a new tile, download and spawn terrain at that location
        if not tile.is_downloaded:
            self.get_logger().info(f"{self.node_name}: Downloading tile at: {tile.pos_coords}")
            self.spawner.download_tile(tile)

        if not tile.is_spawned:
            self.get_logger().info(f"{self.node_name}: Spawning tile at: {tile.pos_coords}")
            self.spawner.spawn_tile(tile)


    def create_materials_file(self, map: Map) -> None:
        """
        This is a quirk with Gazebo, we have to create all our materials for every model we 'might' need up front. In our case we have to make an entry in the material file for each
        of our satellite images, specifying just the name and path.
        We write a line for each possible tile that we can match to a texture name later on when we want to spawn it.
        """
        materials_file_path = os.path.join(os.environ.get("MODEL_DIR"), "materials", "scripts", "ground.material")
        
        # Create the material file
        if not os.path.exists(materials_file_path):
            os.makedirs(os.path.dirname(materials_file_path))
            open(materials_file_path, 'a').close()

        materials = [
            f"material {tile.name} {{ technique {{ pass {{ texture_unit {{ texture {tile.name}.jpeg \n filtering none \n tex_address_mode clamp}} }} }} }}\n" for tile in map.tiles
        ]

        # Write to file
        with open(materials_file_path, 'w') as file:
            file.writelines(materials)


def main(args=None):
    rclpy.init(args=args)

    # Argument parsing
    parser = argparse.ArgumentParser(description='Arguments for making dataset')
    parser.add_argument('--odom_topic',         type=str, help='What topic should we be listening to and spawning tiles around')
    parser.add_argument('--approx_world_lat',   type=float, help='Approximate world origin, meaning that we cant guarantee that the origin will be exactly that. The reason for this is that we want the world origin to align with the tiles defined by OpenStreetMap, so we shift the world origin to the closest center of a tile.')
    parser.add_argument('--approx_world_lon',   type=float, help='Approximate world origin, meaning that we cant guarantee that the origin will be exactly that. The reason for this is that we want the world origin to align with the tiles defined by OpenStreetMap, so we shift the world origin to the closest center of a tile.')
    parser.add_argument('--zoom',               type=int, help="The Open Street Map tile zoom level")
    parser.add_argument('--tile_size_pixels',   type=int, default=512, help='How large are the downloaded tiles')
    parser.add_argument('--map_size',           type=int, help='How many tiles across is the map')
    parser.add_argument('--prefetch_terrain',   type=bool, default=True, help='Download all the tiles in the map at startup?')
    parser.add_argument('--prespawn_terrain',   type=bool, default=True, help='Spawn all the tiles in the map at startup?')
    args, unknown = parser.parse_known_args(args=args)
    
    # Spawn object
    spawner = GazeboTerrainManager(
        odom_topic          = args.odom_topic,
        approx_world_origin = (args.approx_world_lat, args.approx_world_lon),
        zoom                = args.zoom,
        tile_size_pixels    = args.tile_size_pixels,
        map_size            = args.map_size,
        prefetch_terrain    = args.prefetch_terrain,
        prespawn_terrain    = args.prespawn_terrain
    )
    rclpy.spin(spawner)

    # Destroy the node explicitly
    spawner.destroy_node()
    rclpy.shutdown()

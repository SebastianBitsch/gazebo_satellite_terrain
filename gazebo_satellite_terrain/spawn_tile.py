import os
import requests

import xml.etree.ElementTree as ET
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point as ROSPoint

from gazebo_satellite_terrain.spawn_object import GazeboObjectSpawner
from gazebo_satellite_terrain.objects import Tile


class GazeboSatelliteTileSpawner(GazeboObjectSpawner):
    """
    """

    def __init__(self, tile_size_meters: float, node_name: str = "TileSpawner") -> None:
        super().__init__(node_name)

        self.model_path    = os.path.join(os.environ.get("MODEL_DIR"), 'model.sdf')
        self.textures_dir  = os.path.join(os.environ.get("MODEL_DIR"), "materials", "textures")
        os.makedirs(self.textures_dir, exist_ok=True)

        self.set_tile_model_size(new_size = tile_size_meters)


    def spawn_tile(self, tile: Tile) -> None:
        """
        Spawn a tile at its position in Gazebo
        """
        tile.is_spawned = True
        
        pose = Pose()
        pose.position = ROSPoint(
            x =   float(tile.pos_meters.x), 
            y = - float(tile.pos_meters.y), # TODO: Find out why we need a minus here
            z = 0.0
        )
        
        self.set_texture_name(texture_name = tile.name)
        self.spawn_object_gazebo(name = tile.name, xml = self.read_model_xml(self.model_path), pose = pose)


    def download_tile(self, tile: Tile, verbose: bool = False) -> None:
        """
        Download a single tile from the mapbox static tile API (See: https://docs.mapbox.com/api/maps/static-tiles/) 
        Set your API-key, username and style in a .env file. Also not this could be used to download 'styled' terrain.
        (See: https://docs.mapbox.com/api/maps/styles/) tiles, but here we use a style that is completely plain satellite images

        TODO: Refactor / generalize to support downloading tiles from other providers
        """
        tile.texture_path = f'{self.textures_dir}/{tile.name}.jpeg'

        if os.path.isfile(tile.texture_path):
            tile.is_downloaded = True
            if verbose:
                self.get_logger().info(f"{self.node_name}: Tile '{tile.name}' has already been downloaded.")
            return

        # Make the GET request
        username = os.environ.get("MAPBOX_USERNAME")
        style_id = os.environ.get("MAPBOX_STYLENAME")
        response = requests.get(
            f"https://api.mapbox.com/styles/v1/{username}/{style_id}/tiles/{tile.size_pixels}/{tile.zoom}/{tile.pos_coords.x}/{tile.pos_coords.y}",
            params = {
                "access_token": os.environ.get("MAPBOX_APIKEY")
            }
        )

        # Handle the response
        if response.status_code == 200:
            tile.is_downloaded = True

            with open(tile.texture_path, "wb") as f:
                f.write(response.content)

        else:
            self.get_logger().info(f"{self.node_name}: Error downloading image: {response.status_code}: {response.reason}")


    def set_tile_model_size(self, new_size: float) -> None:
        """
        The tile size (in meters) is the same for all tiles but will be different depending on the starting latitude, so we set this once when we initilaize the terrain.
        Update the `model.sdf` to set the size of a tile in Gazebo
        """

        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.parse(self.model_path, parser=parser)
        
        # Update all the size elements
        for size_element in tree.getroot().findall(".//size"):
            size_element.text = f"{new_size} {new_size}"

        # Save the updated file
        tree.write(self.model_path, encoding='utf-8', xml_declaration=True) 


    def set_texture_name(self, texture_name: str) -> None:
        """
        Every time we spawn a tile we need to update what texture we are pointing to in the `model.sdf`, that way we keep spawning the same model but differently textured.
        Will be a name 'tile_x_y'. If the texture doesn't exist a error texture is shown
        """
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.parse(self.model_path, parser=parser)
        
        # Update the texture
        for size_element in tree.getroot().findall(".//name"):
            size_element.text = texture_name

        # Save the updated file
        tree.write(self.model_path, encoding='utf-8', xml_declaration=True) 

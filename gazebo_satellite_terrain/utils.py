import os
import math

import yaml
import numpy as np

from geopy.point import Point as GPSPoint
from gazebo_satellite_terrain.coord import Coord

class DotDict(dict):
    """
    a dictionary that supports dot notation 
    as well as dictionary access notation 
    usage: d = DotDict() or d = DotDict({'val1':'first'})
    set attributes: d.val2 = 'second' or d['val2'] = 'second'
    get attributes: d.val2 or d['val2']

    Shamelessly stolen from: https://stackoverflow.com/a/13520518
    """
    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

    def __init__(self, dct):
        for key, value in dct.items():
            if hasattr(value, 'keys'):
                value = DotDict(value)
            self[key] = value


def load_config(path: str) -> DotDict:
    with open(path, "r") as f:
        dct = yaml.safe_load(f)
        return DotDict(dct)


def load_env(path: str) -> None:
    """
    Function for loading .env file for secret API keys etc. and write them to the global enviroment
    I have the .env file located in the top ws dir, but could be anywhere.
    """
    with open(path, 'r') as fh:
        vars_dict = dict(
            tuple(line.replace('\n', '').split('=')) for line in fh.readlines() if not line.startswith('#')
        )
        os.environ.update(vars_dict)


def deg2tile(gps_point: GPSPoint, zoom: int) -> Coord:
    """
    Convert a GPS coordinate and zoom level to a tile coordinate.
    See: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Implementations
    and: https://docs.mapbox.com/help/glossary/zoom-level/
    and: https://wiki.openstreetmap.org/wiki/Zoom_levels
    """
    lat_rad = math.radians(gps_point.latitude)
    n = 1 << zoom
    xtile = int((gps_point.longitude + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return Coord(xtile, ytile)


def tile2deg(pos: Coord, zoom: int) -> GPSPoint:
    """
    Convert a tile to a GPS coordinate
    Adding +0.5 to the tile.x and tile.y because we want the center not top left corner
    Again see: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Implementations
    """
    n = 1 << zoom
    lon_deg = (pos.x + 0.5) / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * (pos.y + 0.5) / n)))
    lat_deg = math.degrees(lat_rad)

    return GPSPoint(lat_deg, lon_deg)


def meters_per_pixel(pixels_per_tile: int, lat: float, zoom: int) -> float:
    """ Calculate the number of meters a pixel fills, varies with zoom and latitude """
    EARTH_CIRCUMFERENCE_METERS = 40_075_016.686 # Per OpenStreetMap: https://wiki.openstreetmap.org/wiki/Zoom_levels
    meters_per_tile = EARTH_CIRCUMFERENCE_METERS * math.cos(math.radians(lat)) / math.pow(2, zoom)
    return meters_per_tile / pixels_per_tile



def generate_spiral_modes(n: int) -> np.ndarray:
    """
    Generate the indices of an n x n grid in a spiral order starting from the center.

    ### Parameters
        n : int, The size of the grid (n x n).

    ### Returns
        indices: list, A list of indices in spiral order starting from the center of the grid.
    """
    grid = np.arange(n*n).reshape(n, n)
    
    # Center point
    center = (n // 2, n // 2)
    
    # Directions: right, down, left, up
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    # Initialize variables
    current_position = center
    current_direction = 0
    steps = 1
    indices = [grid[center]]
    
    while len(indices) < n * n:
        for _ in range(2):
            for _ in range(steps):
                r, c = current_position
                dr, dc = directions[current_direction]
                r += dr
                c += dc
                if 0 <= r < n and 0 <= c < n:
                    indices.append(grid[r, c])
                    current_position = (r, c)
                else:
                    return indices
            current_direction = (current_direction + 1) % 4
        steps += 1
    
    return indices

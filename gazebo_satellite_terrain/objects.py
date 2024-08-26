from dataclasses import dataclass

from geopy.point import Point as GPSPoint
from geopy.distance import distance

from gazebo_satellite_terrain.coord import Coord
from gazebo_satellite_terrain.utils import deg2tile, tile2deg, meters_per_pixel


@dataclass
class Tile:
    """
    A tile represents a OSM tile that is a square texture defined on a (x,y,zoom) grid, the zoom level determines
    how large an area the tile covers. 
    The position of the tile can be represented by a (x,y,zoom) tuple or a GPS coordinate in the center of the tile
    or a tuple of how many meters east (x) and north (y) the tile is from the origin of the map.
    """
    pos_coords: Coord   # The position in the OSM GPS grid - (12768, 74120) for instance
    pos_meters: Coord   # The position of the tile (meters east and north) in the world in relation to a center of the center tile on the map
    zoom: int           # The zoom level of the OSM grid the pos is defined in

    size_meters: float
    size_pixels: int
    
    is_downloaded: bool = False
    is_spawned: bool    = False
    texture_path: str | None = None

    @property
    def name(self):
        return f"tile_{self.pos_coords.x}_{self.pos_coords.y}"


class Map():
    """ """
    
    def __init__(self, size: int, zoom: int, tile_size_pixels: int, origin_approx: tuple) -> None:
        """
        The map object represent the entire grid of tiles our world consists of.

        It is defined by an GPS origin and how large the grid of tiles should be and provides functionality for 
        querying tiles at a GPS, tile or world (meters east and north of origin) coordinate.

        Working with GPS coords is a bit of a mess of conversions - there is however no way around it:
            gazebo/rviz(world) -> gps -> tile -> map/grid
        """

        if size % 2 == 0:
            raise ValueError(f"Mapsize should be an uneven number, got '{size}'")
        
        self.size = size
        self.zoom = zoom

        self.origin_approx  = Coord(origin_approx[0], origin_approx[1])
        self.origin_gps     = tile2deg(deg2tile(GPSPoint(origin_approx), self.zoom), self.zoom)
        self.origin_coord   = deg2tile(self.origin_gps, self.zoom)

        self.tile_size_pixels = tile_size_pixels
        self.tile_size_meters = self.tile_size_pixels * meters_per_pixel(self.tile_size_pixels, self.origin_gps.latitude, self.zoom)

        # Create the tiles with the correct globel tile ids
        self.tiles = [
            Tile(
                pos_coords     = self.origin_coord + Coord(x,y) - self.half_size,
                pos_meters     = ((self.origin_coord + Coord(x,y) - self.half_size) - self.origin_coord) * self.tile_size_meters, # TODO: i am sure this can be written nicer
                zoom        = self.zoom,
                size_meters = self.tile_size_meters,
                size_pixels = self.tile_size_pixels,
            ) for x in range(size) for y in range(size)
        ]

    def __repr__(self) -> str:
        return (
            "\n--- Map ---\n"
            f"\torigin (gps):   {self.origin_gps}\n"
            f"\torigin (tile):  {self.origin_coord}\n"
            f"\tzoom level:     {self.zoom}\n"
            f"\tmap size (tile):{self.size} x {self.size}\n"
            f"\tmap size (m):   {self.size * self.tile_size_meters:.1f} x {self.size * self.tile_size_meters:.1f}\n"
            f"\ttile size (px): {self.tile_size_pixels} x {self.tile_size_pixels}\n"
            f"\ttile size (m):  {self.tile_size_meters:.1f} x {self.tile_size_meters:.1f}\n"
        )

    @property
    def half_size(self) -> int:
        return self.size // 2


    def tile_at_pos(self, pos: Coord, throws: bool = False) -> Tile | None:
        """ Return the tile at a number of meteres in x and y direction relative to the world origin"""
        # Move east (+x direction)
        east = distance(meters=abs(pos.x)).destination(point=self.origin_gps, bearing=90 if 0 < pos.x else 270)
        # Move north (+y direction)
        final = distance(meters=abs(pos.y)).destination(point=east, bearing=0 if 0 < pos.y else 180)
        
        return self.tile_at_gps(final, throws)


    def tile_at_gps(self, gps: GPSPoint, throws: bool = False) -> Tile | None:
        """ Return the tile at a gps coordinate """
        global_tile_pos = deg2tile(gps, zoom = self.zoom)
        return self.tile_at_index(global_tile_pos, throws)


    def tile_at_index(self, pos: Coord, throws: bool = False) -> Tile | None:
        """ Take a tile in global OSM frame and return the tile from our map"""
        relative_pos = self.origin_coord - pos + self.half_size  # center to our map and 'roll' so 0,0 is the center of our map

        idx = relative_pos.x + (relative_pos.y * self.size)     # flatten 2D coordinate to a single list idx

        if idx < 0 or len(self.tiles) <= idx:
            if throws:
                raise IndexError(f"Tile at {pos} is out of bounds for map of size {self.size} with center {self.origin_coord}")
            return None

        return self.tiles[idx]
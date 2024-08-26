# gazebo-satellite-terrain
ROS2 package for spawning and managing satellite image tiles in Gazebo to form a cohesive terrain.

This whole project is an exercise in Gazebo and ROS2 quirks which is why the code looks somewhat convoluted in some places.

### Installation
Package only tested on Mac M1 using [RoboStack](https://robostack.github.io) and on Ubuntu 24.04.

- Update the `config.yaml` with map size, tile size, zoom, etc.
- Create a `.env` file with the mapbox parameters:
    ```text
    MAPBOX_APIKEY=...
    MAPBOX_USERNAME=...
    MAPBOX_STYLENAME=...
    ```

**Instructions**
1. TODO
2. ...
3. ...

**Usage**
- To launch the simulation:
    ```
    ros2 launch gazebo_satellite_terrain launch_sim.launch.py
    ```

#### Future work
- Only [MapBox](https://www.mapbox.com) API is currently supported, would be nice to support other backends
- Rectuangular maps are currently not supported on N x N maps, could easily be implemented
- Comprehensive list of dependencies written to `CMakeList.txt` and `package.xml` should be compiled to make install easier
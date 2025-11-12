# Maps Directory

This directory stores the maps created during mapping mode.

## Map Files

Maps are saved in two formats:
- `.pgm` - Image file containing the occupancy grid
- `.yaml` - Metadata file describing the map parameters

## Auto-generated Maps

When you save a map, it will be stored here with a timestamp:
- `my_map_<timestamp>.pgm`
- `my_map_<timestamp>.yaml`

## Usage

To use a saved map in navigation mode, specify it in the launch file:
```bash
ros2 launch navigation_control navigation.launch.py map_file:=/path/to/map.yaml
```

# Global Paths Directory
This directory contains all mock global path CSV files, as well as a path_builder.py module.

## path_builder.py Module

The path_builder.py module can be used two different ways:

1.  The module serves a basic flask application which allows for easy creation and modification of mock global paths. To launch the GUI, run the following command from the root of the local_pathfinding repository:

`python3 global_paths/path_builder/path_builder.py`

2. The module can also be run from the command line to quickly plot or interpolate a path from a csv file. To do so, run the following command from the root of the local_pathfinding repository:

`python3 global_paths/path_builder/path_builder.py --file_path <path_to_csv_file>` (to plot)

`python3 global_paths/path_builder/path_builder.py --file_path <path_to_csv_file> --interpolate` (to interpolate)

This will plot the specified path on a 3d globe and will not launch the GUI.

### GUI Usage

The Path Builder supports the following operations:

* Path creation and editing on a 2D OpenStreetMap Map
* Path exporting
* Path importing
* Interpolation between waypoints
* Clear path function
* Purge generated path files
* 3D visualization of path

Note: Selecting a small value for the interpolation interval spacing or having a large number of waypoints may cause performance issues as the javascript is not super well optimmized. If this is an issue, try increasing the spacing or running the interpolation from the CLI instead.

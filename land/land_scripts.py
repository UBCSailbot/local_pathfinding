# import argparse
# import json
import json
from typing import List

import numpy as np
import plotly.graph_objects as go

# from custom_interfaces.msg import HelperLatLon
from shapely._ragged_array import get_parts
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon, shape

# from shapely.io import from_geojson

# from local_pathfinding.coord_systems import latlon_to_xy

GEOJSON_DIR = (
    "/workspaces/sailbot_workspace/src/local_pathfinding/land/geojson/north_america.geo.json"
)
CSV_DIR = "/workspaces/sailbot_workspace/src/local_pathfinding/land/csv"


def to_csv(resolution: int):
    """
    Parses the specified geojson file and converts all Features in the FeatureCollection into
    separate csv files of latlon coordinates for each polygon.
    """
    pass


def get_local_tiles(degree: int = 1) -> List[str]:
    pass


def to_polygons() -> List[Polygon]:
    """
    with open(GEOJSON_DIR, "r", encoding="utf8") as file:
        geo_data = json.loads(file.read())

    polygons = []
    reference = HelperLatLon(latitude=-0.32147752460578416, longitude=-136.1399055653524)

    # This just doesn't work well, geojson is a mess
    # geodata is now a dictionary
    for feature in geo_data.get("features"):
        for coordinate_set in feature["geometry"]["coordinates"]:
            for coord_subset in coordinate_set:
                latlons = [coord for coord in coord_subset]

                try:
                    points = [
                        latlon_to_xy(
                            reference=reference,
                            latlon=HelperLatLon(
                                latitude=float(coord[0]), longitude=float(coord[1])
                            ),
                        )
                        for coord in latlons
                        if isinstance(coord, list)
                    ]
                except TypeError as e:
                    print(e)
                    exit(1)
                polygon = Polygon([[point[0], point[1]] for point in points])
                polygons.append(polygon)
    """

    with open(GEOJSON_DIR, "r") as f:
        features = json.load(f)["features"]

    geo_collection = GeometryCollection(
        [shape(feature["geometry"]).buffer(0) for feature in features]
    )

    """with open(GEOJSON_DIR, "r") as file:
        geo_collection = from_geojson(file.read())"""

    polygons = get_parts(geo_collection).tolist()
    return polygons


def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--file", help="geojson file to be used to generate tiles")
    # parser.add_argument("--dir", help="The path to geojson directory.")
    # parser.add_argument("--res", help="grid resolution", type=float)
    # args = parser.parse_args()

    land = to_polygons()

    fig = go.Figure()

    for obj in land:

        x = []
        y = []

        if isinstance(obj, MultiPolygon):
            print("multi")
            for polygon in obj.geoms:
                exterior = polygon.exterior.coords.xy
                x.extend(exterior[0])
                y.extend(exterior[1])
                for interior in polygon.interiors:
                    x.extend(interior.coords.xy[0])
                    y.extend(interior.coords.xy[1])
        else:
            print("poly")
            x, y = np.array(obj.exterior.coords.xy)
            x = np.array(x)
            y = np.array(y)

        fig.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="lines",
                fill="toself",
                fillcolor="rgba(0,100,80,0.2)",
                line=dict(color="rgba(255,255,255,0.5)"),
            )
        )

    fig.show()


if __name__ == "__main__":
    main()

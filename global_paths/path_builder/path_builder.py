import csv
import os
import webbrowser
from typing import List, Tuple

import plotly.graph_objects as go
from custom_interfaces.msg import HelperLatLon, Path
from flask import Flask, jsonify, render_template, request

# TODO uncomment when I can get the import to work
# from global_paths.plot_global_path import get_lats_and_lons, plot_global_path
from local_pathfinding.node_mock_global_path import MockGlobalPath

app = Flask(__name__)

DEFAULT_DIR = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/"


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/plot_path", methods=["POST"])
def plot_path():
    data = request.json
    result = handle_plot(data)
    return jsonify(result)


def handle_plot(data):
    filename = data.get("filename", "")
    file_path = os.path.join(DEFAULT_DIR, filename)

    if not str(filename).endswith(".csv"):
        file_path = file_path + ".csv"

    try:
        # pos and interval_spacing are not used in this case, 30.0 and [0.0, 0.0] are placeholders
        # if the user tries to plot a single waypoint path, they will get an error
        lats, lons = get_lats_and_lons(
            file_path=file_path, interval_spacing=30.0, pos=[0.0, 0.0], write=False
        )
        plot_global_path(lats, lons)
        return {"status": "success", "message": "Path plotted successfully"}
    except Exception as e:
        return {"status": "error", "message": f"Error plotting path: {str(e)}"}


# TODO REMOVE THIS ONCE IMPORT WORKS --------------------------------------------------------------
def get_lats_and_lons(
    file_path: str, interval_spacing: float, pos: tuple, write: bool
) -> Tuple[List[float], List[float]]:
    lats = []
    lons = []

    with open(file_path, "r") as file:
        reader = csv.reader(file)
        # skip header
        reader.__next__()
        for row in reader:
            lats.append(float(row[0]))
            lons.append(float(row[1]))

    if len(lats) < 2:
        pos = HelperLatLon(latitude=float(pos[0]), longitude=float(pos[1]))
        dest = HelperLatLon(latitude=lats[0], longitude=lons[0])
        path = MockGlobalPath.generate_path(
            dest=dest,
            interval_spacing=interval_spacing,
            pos=pos,
            write=write,
            file_path=file_path,
        )
        lats = []
        lons = []
        for item in path.waypoints:
            lats.append(item.latitude)
            lons.append(item.longitude)

    return lats, lons


def plot_global_path(lats, lons):
    fig = go.Figure(
        data=go.Scattergeo(
            lat=lats,
            lon=lons,
            mode="markers+lines",
            line=dict(width=2, color="blue"),
        )
    )

    fig.update_layout(
        title_text="Mock Global Path Plot",
        showlegend=True,
        geo=dict(
            showland=True,
            showcountries=True,
            showocean=True,
            countrywidth=0.5,
            landcolor="rgb(230, 145, 56)",
            lakecolor="rgb(0, 255, 255)",
            oceancolor="rgb(0, 255, 255)",
            projection=dict(type="orthographic", rotation=dict(lon=-100, lat=40, roll=0)),
            lonaxis=dict(showgrid=True, gridcolor="rgb(102, 102, 102)", gridwidth=0.5),
            lataxis=dict(showgrid=True, gridcolor="rgb(102, 102, 102)", gridwidth=0.5),
        ),
    )

    fig.show()

    # TODO REMOVE THIS ONCE IMPORT WORKS ---------------------------------------------------------


@app.route("/export_waypoints", methods=["POST"])
def export_waypoints():
    data = request.json
    result = handle_export(data)
    return jsonify(result)


def handle_export(data):
    filename = data.get("filename", "")
    waypoints = data.get("waypoints", [])

    file_path = os.path.join(DEFAULT_DIR, filename)

    if not str(filename).endswith(".csv"):
        file_path = file_path + ".csv"

    # convert from json to list of HelperLatLon
    waypoints = [
        (HelperLatLon(latitude=float(item["lat"]), longitude=float(item["lon"])))
        for item in waypoints
    ]

    # convert to Path, to pass to file writer
    path = Path(waypoints=waypoints)

    try:
        MockGlobalPath.write_to_file(file_path=file_path, global_path=path, tmstmp=False)
        return {"status": "success", "message": "Waypoints exported successfully"}
    except Exception as e:
        return {"status": "error", "message": f"Error exporting waypoints: {str(e)}"}


if __name__ == "__main__":
    webbrowser.open("http://127.0.0.1:5000")

    app.run(host="0.0.0.0", port=5000)

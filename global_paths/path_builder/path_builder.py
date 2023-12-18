import os
import re
import webbrowser
from os import walk

import numpy as np
import plotly.graph_objects as go
from custom_interfaces.msg import HelperLatLon, Path
from flask import Flask, jsonify, render_template, request

# TODO uncomment when I can get the import to work
# from global_paths.plot_global_path import get_lats_and_lons, plot_global_path
# from global_paths.purge_paths import delete_files
from local_pathfinding.node_mock_global_path import MockGlobalPath

app = Flask(__name__)
# app.config["TEMPLATES_AUTO_RELOAD"] = True

DEFAULT_DIR = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/"


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/plot_path", methods=["POST"])
def plot_path():
    data = request.json
    result = handle_plot(data)
    return jsonify(result)


@app.route("/delete_paths", methods=["POST"])
def delete_paths():
    data = request.json
    result = handle_delete(data)
    return jsonify(result)


def handle_plot(data):
    waypoints = data.get("waypoints", [])

    # convert from json to list of HelperLatLon
    waypoints = [(float(item["lat"]), float(item["lon"])) for item in waypoints]
    waypoints = np.array(waypoints)
    lats, lons = waypoints.T
    try:
        plot_global_path(lats, lons)
        return {"status": "success", "message": "Path plotted successfully"}
    except Exception as e:
        return {"status": "error", "message": f"Error plotting path: {str(e)}"}


def handle_delete(data):
    key = data.get("key", None)
    try:
        delete_files(key=key)
        return {"status": "success", "message": "Paths deleted successfully"}
    except Exception as e:
        return {"status": "error", "message": f"Error deleting paths: {str(e)}"}


# TODO REMOVE THIS ONCE IMPORT WORKS --------------------------------------------------------------
def delete_files(key=None):
    dir_path = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths"
    files = os.listdir(dir_path)

    timestamp_pattern = re.compile(r"_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.csv")

    # check if a keyword was entered
    if key is not None:
        key = re.compile(rf"{key}")

    for file_name in files:
        if re.search(timestamp_pattern, file_name) or (
            key is not None and re.search(key, file_name)
        ):
            file_path = os.path.join(dir_path, file_name)
            try:
                os.remove(file_path)
                print(f"Deleted: {os.path.basename(file_path)} from /global_paths")
            except OSError as e:
                print(f"Error deleting {file_path}: {e}")


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
    # tests
    try:
        MockGlobalPath.write_to_file(file_path=file_path, global_path=path, tmstmp=False)
        return {"status": "success", "message": "Waypoints exported successfully"}
    except Exception as e:
        return {"status": "error", "message": f"Error exporting waypoints: {str(e)}"}


if __name__ == "__main__":
    extra_dirs = [
        "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/path_builder",
    ]
    extra_files = extra_dirs[:]
    for extra_dir in extra_dirs:
        for dirname, dirs, files in walk(extra_dir):
            for filename in files:
                filename = os.path.join(dirname, filename)
                if os.path.isfile(filename):
                    extra_files.append(filename)

    webbrowser.open("http://127.0.0.1:5000")
    # opens two tabs on startup when debug=True
    app.run(host="0.0.0.0", port=5000, debug=True, extra_files=extra_files)

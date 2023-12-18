var MAP_ZOOM = 6;
// Initialize the map
var map = L.map('map').setView([49.1536, -125.9067], MAP_ZOOM);

// Load a tile layer
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
}).addTo(map);

var waypoints = [];
var undone_waypoints = [];

// Event listener for clicking on the map
map.on('click', function(e) {
    // Get coordinates of the clicked point
    var lat = e.latlng.lat.toFixed(MAP_ZOOM);
    var lon = e.latlng.lng.toFixed(MAP_ZOOM);

    // Add coordinates to the waypoints array
    waypoints.push({lat,lon});
    undone_waypoints = [];

    refresh_map();
});

function draw_marker(item){
    L.marker([item.lat, item.lon]).addTo(map).bindPopup(`Waypoint: ${item.lat}, ${item.lon}`);
}

function draw_polyline(item, index){
    if (index > 0){
        var prevLatLon = waypoints[index - 1];
        L.polyline([[prevLatLon.lat, prevLatLon.lon], [item.lat, item.lon]]).addTo(map);
    }
}

function refresh_map(){

    // clear map and redraw markers and polylines
    map.eachLayer(function(layer) {
        if (layer instanceof L.Marker || layer instanceof L.Polyline) {
            layer.remove();
        }
    });

    waypoints.forEach(draw_marker);

    if (waypoints.length > 1) {
        waypoints.forEach(draw_polyline);
    }

    update_waypoints_table();
}
// Button event handlers
function clear_path(){

    var confirmation = confirm("Are you sure you want to clear all waypoints? You cannot undo this action.");

    if (confirmation) {
        waypoints = [];
        refresh_map();
    }
}
function import_file(){
}
function interpolate(){
}
function delete_paths(){

    var key = window.prompt("Enter the keyword in the filenames you want to delete:", "test");

    if (key === null) {
        // User clicked Cancel, do nothing
        return;
    }

    var confirmation = confirm(`Are you sure you want to delete all paths containing the keyword: ${key}?`);

    if (confirmation) {

        fetch('/delete_paths', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ key: key }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'error'){
                alert('Error deleting paths. Please check the server logs for details.');
            }
            else{
                alert('Test Paths deleted successfully.');
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
    }
}

function undo(){
    if (waypoints.length > 0) {
        // Remove the last waypoint from the waypoints array
        undone_waypoints.push(waypoints.pop());

        refresh_map();

        updateWaypointsTable();
    }
}

function redo(){
    if (undone_waypoints.length > 0) {
        // Remove the last waypoint from the waypoints array
        waypoints.push(undone_waypoints.pop());

        refresh_map();

        updateWaypointsTable();
    }
}

function plot(){
    if (waypoints.length < 2) {
        alert('Please add at least two waypoints to plot a path.');
        return;
    }
    else{
        fetch('/plot_path', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ waypoints: waypoints }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'error'){
                alert('Error plotting waypoints. Please check the server logs for details.');
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
    }
}


function prompt_and_export() {
    if (waypoints.length < 2) {
        alert('Please add at least two waypoints to export.');
        return;
    }
    // Prompt user for filename
    var filename = window.prompt("Enter the desired filename for the CSV file:", "");

    if (filename === null) {
        // User clicked Cancel, do nothing
        return;
    }

    if (filename === "") {
        alert("You must enter a filename.");
        return;
    }

    fetch('/export_waypoints', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ filename: filename, waypoints: waypoints }),
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success'){
            alert('Waypoints exported successfully.');
        } else {
            alert('Error exporting waypoints. Please check the server logs for details.');
        }
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

function delete_waypoint(index) {
    // Remove the waypoint from the waypoints array
    waypoints.splice(index, 1);

    refresh_map();

    update_waypoints_table();
}

function update_waypoints_table() {
    var tableBody = document.getElementById("waypointsTable").getElementsByTagName('tbody')[0];

    // Clear existing rows
    tableBody.innerHTML = '';

    // Add waypoints to the table
    waypoints.forEach(function (waypoint, index) {
        var newRow = tableBody.insertRow(tableBody.rows.length);

        var cell1 = newRow.insertCell(0);
        var cell2 = newRow.insertCell(1);
        var cell3 = newRow.insertCell(2);

        cell1.innerHTML = waypoint.lat;
        cell2.innerHTML = waypoint.lon;
        cell3.innerHTML = `<button type="button" onclick="delete_waypoint(${index})">Delete</button>`;
    });
}

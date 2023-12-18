var MAP_ZOOM = 6;
// Initialize the map
var map = L.map('map').setView([49.1536, -125.9067], MAP_ZOOM);

// Load a tile layer
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
}).addTo(map);

var waypoints = [];

// Event listener for clicking on the map
map.on('click', function(e) {
    // Get coordinates of the clicked point
    var lat = e.latlng.lat.toFixed(MAP_ZOOM);
    var lon = e.latlng.lng.toFixed(MAP_ZOOM);

    // Add coordinates to the waypoints array
    waypoints.push({lat,lon});

    // Add a marker at the clicked point
    L.marker([lat, lon]).addTo(map).bindPopup(`Waypoint: ${lat}, ${lon}`);

    // Draw a polyline if there are at least two waypoints
    if (waypoints.length > 1) {
        var prevLatLon = waypoints[waypoints.length - 2];
        polyline = L.polyline([[prevLatLon.lat, prevLatLon.lon], [lat, lon]]).addTo(map);
    }

    updateWaypointsTable();
});

// Button event handlers
function clear(){

    var confirmation = confirm("Are you sure you want to clear all waypoints?");

    if (confirmation) {

        waypoints = [];
        map.eachLayer(function(layer) {
            if (layer instanceof L.Marker || layer instanceof L.Polyline) {
                layer.remove();
            }
        });
        updateWaypointsTable();
    }
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


function promptAndExport() {
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

function updateWaypointsTable() {
    var tableBody = document.getElementById("waypointsTable").getElementsByTagName('tbody')[0];

    // Clear existing rows
    tableBody.innerHTML = '';

    // Add waypoints to the table
    waypoints.forEach(function(waypoint) {
        var newRow = tableBody.insertRow(tableBody.rows.length);

        var cell1 = newRow.insertCell(0);
        var cell2 = newRow.insertCell(1);

        cell1.innerHTML = waypoint.lat;
        cell2.innerHTML = waypoint.lon;
    });
}

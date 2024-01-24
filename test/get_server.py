"""
This is a basic http server to handle GET requests from the global path module until the WEB
API is implemented.

It serves a hardcoded JSON response for the /api/gps endpoint.
"""
import json
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler


class CustomRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/gps":
            # Respond with the desired JSON message
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()

            # Sample GPS data
            gps_data = [
                {
                    "latitude": 49.376629216076275,
                    "longitude": -123.27266500802254,
                    "speed": 0,
                    "heading": 0,
                    "timestamp": "2024-01-19T20:42:50.509Z",
                },
                {
                    "latitude": 49.37711663428484,
                    "longitude": -123.27156381625609,
                    "speed": 0,
                    "heading": 0,
                    "timestamp": "2024-01-19T20:42:50.510Z",
                },
            ]

            response = {"success": True, "data": gps_data}
            self.wfile.write(json.dumps(response).encode())
        else:
            # Serve other requests using the default handler
            super().do_GET()


def run_server(port=3005) -> HTTPServer:
    server_address = ("localhost", port)
    httpd = HTTPServer(server_address, CustomRequestHandler)

    def run():
        print(f"Server running on http://localhost:{port}")
        httpd.serve_forever()

    # Start the server in a separate thread
    server_thread = threading.Thread(target=run)
    server_thread.start()

    return httpd


def shutdown_server(httpd: HTTPServer):
    httpd.shutdown()


if __name__ == "__main__":
    run_server()

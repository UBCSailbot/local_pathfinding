import json
from http.server import BaseHTTPRequestHandler, HTTPServer


class CustomRequestHandler(BaseHTTPRequestHandler):
    def _set_response(self, status_code=200, content_type="application/json"):
        self.send_response(status_code)
        self.send_header("Content-type", content_type)
        self.end_headers()

    def do_POST(self):
        content_length = int(self.headers["Content-Length"])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode("utf-8"))

        # Process the data as needed
        waypoints = data.get("waypoints", [])

        # For now, just print the waypoints
        print("Received waypoints:", waypoints)

        self._set_response(200)
        self.wfile.write(
            json.dumps({"message": "Global path received successfully"}).encode("utf-8")
        )


def run_server(port=8081):
    server_address = ("localhost", port)
    httpd = HTTPServer(server_address, CustomRequestHandler)
    print(f"Server running on http://localhost:{port}")
    httpd.serve_forever()


if __name__ == "__main__":
    run_server()

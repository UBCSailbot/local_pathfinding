from http.server import HTTPServer, SimpleHTTPRequestHandler


class CustomRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/gps":
            # Respond with a simple JSON message
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            response = '{"lat_lon": {"latitude": 40.7128, "longitude": -74.0060}}'
            self.wfile.write(response.encode())
        else:
            # Serve other requests using the default handler
            super().do_GET()


def run_server(port=3005):
    server_address = ("localhost", port)
    httpd = HTTPServer(server_address, CustomRequestHandler)
    print(f"Server running on http://localhost:{port}")
    httpd.serve_forever()


if __name__ == "__main__":
    run_server()

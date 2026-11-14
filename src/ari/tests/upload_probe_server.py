# A resumable upload endpoint for file_pick_upload_test.sh, as txwtf's PATCH
# /social/files/resumable/{id} is: PATCH /up appends the body at Upload-Offset and answers
# {"offset": N}, or 409 with its own Upload-Offset when the client's is elsewhere. GET /up answers
# "N SHA256" of what it holds. Prints its port, then serves until killed.
import hashlib
import http.server
import json
import sys

stored = bytearray()


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, status, body, ctype="application/json", headers=()):
        data = body.encode()
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        for name, value in headers:
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(data)

    def do_PATCH(self):
        chunk = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        if self.path != "/up":
            return self.send(404, "{}")
        if self.headers.get("Upload-Offset") != str(len(stored)):
            return self.send(409, json.dumps({"detail": "Offset mismatch"}),
                             headers=[("Upload-Offset", str(len(stored)))])
        stored.extend(chunk)
        self.send(200, json.dumps({"offset": len(stored)}))

    def do_GET(self):
        if self.path != "/up":
            return self.send(404, "{}")
        self.send(200, "%d %s" % (len(stored), hashlib.sha256(stored).hexdigest()), "text/plain")


server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
print(server.server_address[1], flush=True)
try:
    server.serve_forever()
except KeyboardInterrupt:
    sys.exit(0)

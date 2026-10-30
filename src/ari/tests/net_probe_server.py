# A stand-in for a registry's sign-in, for the wasm ari's network check (ci.yml, "ari terminal
# backend (wasm, node)"). POST /v1/auth/device/token answers an access and a refresh token;
# POST /v1/auth/token refreshes with the refresh token in a form; GET /v1/me wants the current
# access token. Prints its port, then serves until killed.
import http.server
import json
import urllib.parse

ACCESS = "cvcses_d2FzbS1hY2Nlc3MtdG9rZW4tZm9yLWNp"
REFRESH = "cvcref_d2FzbS1yZWZyZXNoLXRva2VuLWZvci1jaQ"
FRESH = "cvcses_d2FzbS1yZWZyZXNoZWQtYWNjZXNzLXRva2Vu"


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, status, body):
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode()
        if self.path == "/v1/auth/device/token":
            return self.reply(200, {"access_token": ACCESS, "refresh_token": REFRESH,
                                    "expires_in": 43200})
        if self.path == "/v1/auth/token":
            form = urllib.parse.parse_qs(body)
            if form.get("grant_type") == ["refresh_token"] and form.get("refresh_token") == [REFRESH]:
                return self.reply(200, {"access_token": FRESH, "expires_in": 43200})
            return self.reply(401, {})
        self.reply(404, {})

    def do_GET(self):
        if self.path == "/v1/me":
            ok = self.headers.get("Authorization") == "Bearer " + FRESH
            return self.reply(200 if ok else 401, {"user": "ci"} if ok else {})
        self.reply(404, {})


server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
print(server.server_address[1], flush=True)
server.serve_forever()

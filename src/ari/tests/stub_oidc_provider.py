"""A stub OpenID Provider for testing Ariadne sign-in (docs/ARIADNE.md §10.5) by hand, in the
ari/arigl tmux tests and under node for the browser builds. Standard library only.

  python3 stub_oidc_provider.py [--port N] [--secrets FILE] [--no-device] [--expires-in S]
                                [--auto-approve] [--client-id ID]

Prints its port, then serves until killed. Its issuer is http://127.0.0.1:PORT. Endpoints:

  /.well-known/openid-configuration  discovery
  /authorize      approves at once: a 302 back to redirect_uri with code, state and iss
  /token          authorization_code (PKCE S256 checked), refresh_token (rotated), device_code
  /device         device authorization (RFC 8628); interval 1
  /approve?user_code=C[&deny=1]      approves (or denies) a device code, as a person would
  /revoke, /logout, /api/me (wants the access token)

Every token, code and device code it issues is appended to --secrets, one a line, so a test can
check that none reached a host's output. Replies carry Access-Control-Allow-Origin: *, as a
provider that serves browser pages must on its token endpoint.
"""
import argparse
import base64
import hashlib
import http.server
import json
import secrets
import threading
import time
import urllib.parse

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, default=0)
parser.add_argument("--secrets", default="")
parser.add_argument("--no-device", action="store_true")
parser.add_argument("--expires-in", type=int, default=3600)
parser.add_argument("--auto-approve", action="store_true")
parser.add_argument("--client-id", default="ariadne-test-client")
args = parser.parse_args()

lock = threading.Lock()
grants = {}    # code -> (redirect_uri, challenge, nonce, scope)
refresh = {}   # refresh token -> True
access = {}    # access token -> True
devices = {}   # device_code -> {"user_code", "decision", "scope"}


def b64url(data):
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode()


def mint(prefix):
    token = prefix + secrets.token_urlsafe(24)
    if args.secrets:
        with open(args.secrets, "a") as f:
            f.write(token + "\n")
    return token


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    @property
    def issuer(self):
        return "http://127.0.0.1:%d" % self.server.server_address[1]

    def send(self, status, body=b"", ctype="application/json", headers=()):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def error(self, code, status=400):
        self.send(status, {"error": code})

    def tokens(self, nonce, with_id):
        a, r = mint("oidcAT_"), mint("oidcRT_")
        access[a] = True
        refresh[r] = True
        reply = {"access_token": a, "token_type": "Bearer", "expires_in": args.expires_in,
                 "refresh_token": r}
        if with_id:
            now = int(time.time())
            claims = {"iss": self.issuer, "sub": "user-42", "aud": args.client_id,
                      "exp": now + 600, "iat": now, "name": "Ada Lovelace",
                      "email": "ada@example.test", "preferred_username": "ada"}
            if nonce:
                claims["nonce"] = nonce
            jwt = ".".join([b64url(b'{"alg":"RS256","kid":"stub"}'),
                            b64url(json.dumps(claims).encode()),
                            b64url(b"placeholder-signature-" + secrets.token_bytes(6))])
            if args.secrets:
                with open(args.secrets, "a") as f:
                    f.write(jwt + "\n")
            reply["id_token"] = jwt
        self.send(200, reply)

    def do_OPTIONS(self):
        self.send(204, b"", headers=[("Access-Control-Allow-Methods", "GET, POST"),
                                    ("Access-Control-Allow-Headers", "Content-Type, Authorization"),
                                    ("Access-Control-Max-Age", "600")])

    def do_GET(self):
        url = urllib.parse.urlsplit(self.path)
        q = dict(urllib.parse.parse_qsl(url.query))
        with lock:
            if url.path == "/.well-known/openid-configuration":
                doc = {"issuer": self.issuer,
                       "authorization_endpoint": self.issuer + "/authorize",
                       "token_endpoint": self.issuer + "/token",
                       "revocation_endpoint": self.issuer + "/revoke",
                       "end_session_endpoint": self.issuer + "/logout",
                       "code_challenge_methods_supported": ["S256"],
                       "response_types_supported": ["code"]}
                if not args.no_device:
                    doc["device_authorization_endpoint"] = self.issuer + "/device"
                return self.send(200, doc)
            if url.path == "/authorize":
                if (q.get("response_type") != "code" or q.get("client_id") != args.client_id
                        or q.get("code_challenge_method") != "S256" or not q.get("state")):
                    return self.send(400, "bad authorization request", "text/plain")
                code = mint("oidcAC_")
                grants[code] = (q["redirect_uri"], q["code_challenge"], q.get("nonce", ""),
                                q.get("scope", ""))
                sep = "&" if "?" in q["redirect_uri"] else "?"
                loc = q["redirect_uri"] + sep + urllib.parse.urlencode(
                    {"code": code, "state": q["state"], "iss": self.issuer})
                return self.send(302, b"", "text/plain", [("Location", loc)])
            if url.path == "/approve":
                for d in devices.values():
                    if d["user_code"] == q.get("user_code"):
                        d["decision"] = -1 if q.get("deny") else 1
                        return self.send(200, "approved", "text/plain")
                return self.send(404, "no such code", "text/plain")
            if url.path == "/logout":
                return self.send(200, "<p>signed out</p>", "text/html")
            if url.path == "/api/me":
                auth = self.headers.get("Authorization", "")
                if auth.startswith("Bearer ") and auth[7:] in access:
                    return self.send(200, {"sub": "user-42"})
                return self.send(401, {})
        self.send(404, b"", "text/plain")

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode()
        f = dict(urllib.parse.parse_qsl(body))
        path = urllib.parse.urlsplit(self.path).path
        with lock:
            if path == "/token":
                grant = f.get("grant_type")
                if grant == "authorization_code":
                    g = grants.pop(f.get("code", ""), None)
                    if not g or f.get("redirect_uri") != g[0] or f.get("client_id") != args.client_id:
                        return self.error("invalid_grant")
                    digest = hashlib.sha256(f.get("code_verifier", "").encode()).digest()
                    if b64url(digest) != g[1]:
                        return self.error("invalid_grant")
                    return self.tokens(g[2], "openid" in g[3].split())
                if grant == "refresh_token":
                    if not refresh.pop(f.get("refresh_token", ""), None):
                        return self.error("invalid_grant")
                    return self.tokens("", True)
                if grant == "urn:ietf:params:oauth:grant-type:device_code":
                    d = devices.get(f.get("device_code", ""))
                    if not d:
                        return self.error("invalid_grant")
                    if args.auto_approve:
                        d["decision"] = 1
                    if d["decision"] < 0:
                        return self.error("access_denied")
                    if d["decision"] == 0:
                        return self.error("authorization_pending")
                    del devices[f["device_code"]]
                    return self.tokens("", "openid" in d["scope"].split())
                return self.error("unsupported_grant_type")
            if path == "/device" and not args.no_device:
                if f.get("client_id") != args.client_id:
                    return self.error("invalid_client", 401)
                dc = mint("oidcDC_")
                uc = "WDJB-%d" % (1000 + len(devices))
                devices[dc] = {"user_code": uc, "decision": 0, "scope": f.get("scope", "")}
                return self.send(200, {"device_code": dc, "user_code": uc,
                                       "verification_uri": self.issuer + "/activate",
                                       "verification_uri_complete":
                                           self.issuer + "/activate?user_code=" + uc,
                                       "expires_in": 600, "interval": 1})
            if path == "/revoke":
                refresh.pop(f.get("token", ""), None)
                access.pop(f.get("token", ""), None)
                return self.send(200, b"", "text/plain")
        self.send(404, b"", "text/plain")


server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
print(server.server_address[1], flush=True)
server.serve_forever()

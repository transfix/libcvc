# A WebSocket server for the wasm ari's frame check (ci.yml, "ari terminal backend (wasm, node)";
# tests/l4_ws.ari), on the standard library only: it answers the handshake after a second, then
# sends one text message ("slow hello") a second later, and answers the client's close. Prints its
# port, then serves until killed.
import base64
import hashlib
import socketserver
import time

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = self.request.recv(4096)
            if not chunk:
                return
            data += chunk
        key = ""
        for line in data.decode("latin-1").split("\r\n"):
            if line.lower().startswith("sec-websocket-key:"):
                key = line.split(":", 1)[1].strip()
        accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
        time.sleep(1.0)  # a slow handshake: ws-open waits this long
        self.request.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                              "Connection: Upgrade\r\nSec-WebSocket-Accept: " + accept +
                              "\r\n\r\n").encode())
        time.sleep(1.0)  # a slow message: ws-recv waits this long
        payload = b"slow hello"
        self.request.sendall(bytes([0x81, len(payload)]) + payload)
        # Until the client closes: answer its close frame (opcode 8) with ours and hang up, so its
        # WebSocket finishes closing and node can exit.
        try:
            while True:
                frame = self.request.recv(4096)
                if not frame:
                    break
                if frame[0] & 0x0F == 0x8:
                    self.request.sendall(b"\x88\x00")
                    break
        except OSError:
            pass


class Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


server = Server(("127.0.0.1", 0), Handler)
print(server.server_address[1], flush=True)
server.serve_forever()

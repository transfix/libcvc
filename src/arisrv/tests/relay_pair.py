"""Run a relay and a client against it, as the room-call ctests do (src/arisrv/CMakeLists.txt):

    python relay_pair.py --relay ARISRVX --client ARI --scheme grpc|grpcs|ws
                         [--relay-arg A ...] [--client-arg A ...] [--expect REGEX ...]
                         [--client-only [--client-status N] [--client-expect REGEX]]
                         [--client-env NAME=VALUE ...]

The relay listens on ephemeral loopback ports; this reads the one the scheme dials from its
output, then runs the client with --relay URL (and its own args), and waits for the relay to stop
(the client's last call stops it). It passes when both exit 0 and the relay's output matches each
--expect. With --client-only the relay is stopped after the client, which must exit with
--client-status (0) and match --client-expect. --client-env sets the client's environment (its
CVC_ARIADNE_ROOM_TOKEN, say). Both outputs are printed either way.
"""

import argparse
import os
import re
import subprocess
import sys
import threading
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--relay", required=True)
    ap.add_argument("--client", required=True)
    ap.add_argument("--scheme", choices=("grpc", "grpcs", "ws"), default="grpc")
    ap.add_argument("--host", default="127.0.0.1", help="the host the client dials")
    ap.add_argument("--relay-arg", action="append", default=[])
    ap.add_argument("--client-arg", action="append", default=[])
    ap.add_argument("--expect", action="append", default=[])
    ap.add_argument("--client-only", action="store_true")
    ap.add_argument("--client-status", type=int, default=0)
    ap.add_argument("--client-expect", action="append", default=[])
    ap.add_argument("--client-env", action="append", default=[])
    a = ap.parse_args()

    relay = subprocess.Popen([a.relay, *a.relay_arg], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                             errors="replace")
    lines = []
    port = {}
    ready = threading.Event()

    def read():
        for line in relay.stdout:
            lines.append(line)
            m = re.search(r"relay up .*\| state=\S+?:(\d+) ", line)
            if m:
                port["grpc"] = port["grpcs"] = m.group(1)
            m = re.search(r"websocket \(browser\) clients: ws=\S+?:(\d+)", line)
            if m:
                port["ws"] = m.group(1)
                ready.set()
            if "relay up" in line and a.scheme != "ws":
                ready.set()
        ready.set()

    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    status = 1
    try:
        if not ready.wait(60) or a.scheme not in port:
            print("relay_pair: the relay never listened", file=sys.stderr)
            return 1
        url = f"{a.scheme}://{a.host}:{port[a.scheme]}"
        env = dict(os.environ)
        for kv in a.client_env:
            name, _, value = kv.partition("=")
            env[name] = value
        client = subprocess.run([a.client, "test", "--relay", url, *a.client_arg],
                                capture_output=True, text=True, encoding="utf-8",
                                errors="replace", timeout=120, env=env)
        print(f"--- client ({url}): exit {client.returncode}")
        print(client.stdout + client.stderr)
        if a.client_only:
            relay.kill()
            relay.wait(30)
            reader.join(5)
            print("--- relay")
            print("".join(lines))
            status = 0 if client.returncode == a.client_status else 1
            if status:
                print(f"relay_pair: the client exited {client.returncode}, expected "
                      f"{a.client_status}", file=sys.stderr)
            for regex in a.client_expect:
                if not re.search(regex, client.stdout + client.stderr):
                    print(f"relay_pair: the client's output does not match: {regex}",
                          file=sys.stderr)
                    status = 1
            if status == 0:
                print("relay_pair: ok")
            return status
        try:
            relay.wait(30)
        except subprocess.TimeoutExpired:
            print("relay_pair: the relay did not stop after the client's bye", file=sys.stderr)
            return 1
        reader.join(5)
        out = "".join(lines)
        print(f"--- relay: exit {relay.returncode}")
        print(out)
        status = 0 if client.returncode == 0 and relay.returncode == 0 else 1
        for regex in a.expect:
            if not re.search(regex, out):
                print(f"relay_pair: the relay's output does not match: {regex}", file=sys.stderr)
                status = 1
        if status == 0:
            print("relay_pair: ok")
        return status
    finally:
        if relay.poll() is None:
            relay.kill()
            time.sleep(0.2)


if __name__ == "__main__":
    sys.exit(main())

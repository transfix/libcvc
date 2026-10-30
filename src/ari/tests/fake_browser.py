"""A stand-in for the user's web browser, for $BROWSER in the sign-in tests (docs/ARIADNE.md §10.4).

  BROWSER="python3 fake_browser.py" ari run ...

The host runs it with the authorization URL. It GETs that URL without following the redirect, then
GETs where the provider sent it (the host's loopback listener), as a browser would, and appends
what it saw to $FAKE_BROWSER_LOG when that is set. Standard library only; it backgrounds itself so
the host's launch returns at once.
"""
import http.client
import os
import sys
import urllib.parse


def get(url):
    u = urllib.parse.urlsplit(url)
    conn = http.client.HTTPConnection(u.hostname, u.port or 80, timeout=20)
    conn.request("GET", (u.path or "/") + ("?" + u.query if u.query else ""))
    r = conn.getresponse()
    return r.status, r.getheader("Location", ""), r.read().decode("utf-8", "replace")


def main(url):
    log = os.environ.get("FAKE_BROWSER_LOG", "")
    status, location, _ = get(url)
    lines = ["authorize %d" % status]
    if location:
        status, _, body = get(location)
        lines.append("callback %d %s" % (status, "close this tab" in body))
    if log:
        with open(log, "a") as f:
            f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    if os.fork() == 0:
        os.setsid()
        main(sys.argv[1])
        os._exit(0)

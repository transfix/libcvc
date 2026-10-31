#ifndef CVC_NET_WS_CLIENT_H
#define CVC_NET_WS_CLIENT_H

// cvc::net — a WebSocket client, for Ariadne's ws-* verbs (docs/ARIADNE.md §10.5).
//
// One interface, two backends picked at build time, as for the HTTP client (http_client.h):
//   - native: libcurl's WebSocket API (CURLOPT_CONNECT_ONLY = 2, curl_ws_send / curl_ws_recv), so
//     wss:// has the same TLS, CA store and proxy handling as https:// requests. A reader thread
//     takes frames off the socket into a queue; libcurl answers pings itself.
//   - wasm: the browser's WebSocket (emscripten/websocket.h); its callbacks fill the same queue.
//     A browser WebSocket cannot carry request headers, so headers are refused there.
//
// Messages are whole: a fragmented message is reassembled before it is queued. Text and binary
// messages are both delivered (data as bytes in a std::string); a close is the last event.
//
// NOT BLOCKING: ws_start and WsConnection::recv_async are the other entries, as cvc::net::start is
// for HTTP (http_client.h). Where the backend can (the browser without threads, ws_starts_async()),
// they return at once and answer from the event loop; elsewhere they wait in place and answer
// before returning. Ariadne's ws-open and ws-recv use them there, so a page's frame loop does not
// wait on a connect or a message (docs/ARIADNE.md 10.5).

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
namespace net {

struct WsEvent {
  enum class Kind { Text, Binary, Close, Error };
  Kind kind = Kind::Text;
  std::string data; // the message; for Close the reason, for Error the description
  int code = 0;     // Close: the close code (1005 when the peer sent none)
};

// What a non-blocking recv calls with: the next event, or nullopt when none came in time.
using WsReceived = std::function<void(std::optional<WsEvent>)>;

class WsConnection {
public:
  virtual ~WsConnection() = default;
  // The next event, waiting up to timeout_secs (0: don't wait). nullopt when none came in time,
  // or when cancelled() turned true while waiting (checked about every 50 ms).
  virtual std::optional<WsEvent> recv(double timeout_secs,
                                      const std::function<bool()> &cancelled = {}) = 0;
  // recv() without blocking: `done` gets the next event, before this returns if one is queued
  // already, else as one arrives, or nullopt once timeout_secs pass. Where ws_starts_async() it
  // returns at once and `done` runs from the event loop, on this thread; the default waits here,
  // as recv(), and calls `done` before returning. The function returned drops the wait (`done` is
  // then not called); it may be empty. One wait at a time per connection.
  virtual std::function<void()> recv_async(double timeout_secs, WsReceived done);
  // Send a text message. False with *error set when the connection is closed or the send fails.
  virtual bool send_text(const std::string &text, std::string *error = nullptr) = 0;
  // Start closing (code 1000 by default). Events already queued stay readable.
  virtual void close(int code = 1000) = 0;
  // Whether the connection is open (connected, and no close seen or sent).
  virtual bool open() const = 0;
};

struct WsOptions {
  std::vector<std::string> headers; // "Name: value" lines for the handshake (native only)
  double connect_timeout_secs = 15;
  std::size_t max_message_bytes = 16u << 20; // a longer message is an Error event, then a close
};

// Connect to a ws:// or wss:// URL. Natively it blocks until the handshake is done (call it off
// the scheduler thread); in a browser it returns at once and the first event is the open, or an
// Error. Null, with *error set, when the URL or the options can't be used or the connect fails.
std::shared_ptr<WsConnection> ws_connect(const std::string &url, const WsOptions &opts,
                                         std::string *error = nullptr);

// What a non-blocking connect calls with: the open connection, or null and why not.
using WsConnected = std::function<void(std::shared_ptr<WsConnection> conn, std::string error)>;

// ws_connect without blocking: where ws_starts_async() it returns at once and `done` runs from the
// event loop, on this thread, once the connection opens, fails or times out
// (opts.connect_timeout_secs); elsewhere it connects in place and calls `done` before returning.
// The function returned abandons a connect still under way (`done` is then not called, and the
// socket is closed); it may be empty.
std::function<void()> ws_start(const std::string &url, const WsOptions &opts, WsConnected done);

// Whether ws_start and WsConnection::recv_async return before they answer (the browser's
// WebSocket in a build without threads, or a connector that says so).
bool ws_starts_async();

// The transport behind ws_connect and ws_start: the compiled backend, or one a test or a host puts
// in its place (set_ws_connector), as set_http_client does for HTTP.
class WsConnector {
public:
  virtual ~WsConnector() = default;
  virtual std::shared_ptr<WsConnection> connect(const std::string &url, const WsOptions &opts,
                                                std::string *error) = 0;
  // Default: connect() in place, then `done`.
  virtual std::function<void()> start(const std::string &url, const WsOptions &opts,
                                      WsConnected done);
  virtual bool starts_async() const { return false; }
};

// Replace the process-global connector; null restores the compiled backend. Thread-safe; set it
// before the connections it should make.
void set_ws_connector(std::shared_ptr<WsConnector> connector);

// Whether this build has a WebSocket backend (libcurl with ws support, or the browser's).
bool have_ws_backend();

// The http(s) URL a ws(s) URL's origin is checked as ("ws://h/p" -> "http://h/p"), or the URL
// itself when it is not ws/wss.
std::string ws_http_equivalent(const std::string &url);

} // namespace net
} // namespace cvc

#endif // CVC_NET_WS_CLIENT_H

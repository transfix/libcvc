// cvc::net — the WebSocket client (inc/cvc/net/ws_client.h): libcurl natively, the browser's
// WebSocket under Emscripten. Exactly one backend is compiled, keyed on the same definitions as
// the HTTP client (CVC_NET_HAVE_CURL / CVC_NET_HAVE_FETCH); with neither, ws_connect fails.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cvc/net/ws_client.h>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#if defined(CVC_NET_HAVE_FETCH) && defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#include <emscripten/eventloop.h> // emscripten_set_timeout: the non-blocking waits' timers
#include <emscripten/websocket.h>
#define CVC_NET_WS_WEB 1
#elif defined(CVC_NET_HAVE_CURL)
#include <curl/curl.h>
#if LIBCURL_VERSION_NUM >= 0x075600 // curl_ws_* arrived in 7.86.0
#define CVC_NET_WS_CURL 1
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX // winsock2.h pulls in windows.h, whose min/max macros break std::min
#endif
#include <winsock2.h>
#else
#include <poll.h>
#endif
#endif
#endif

namespace cvc {
namespace net {

std::string ws_http_equivalent(const std::string &url) {
  if (url.rfind("ws://", 0) == 0)
    return "http://" + url.substr(5);
  if (url.rfind("wss://", 0) == 0)
    return "https://" + url.substr(6);
  return url;
}

namespace {

bool is_ws_url(const std::string &url) {
  return url.rfind("ws://", 0) == 0 || url.rfind("wss://", 0) == 0;
}

void set_error(std::string *error, const std::string &message) {
  if (error)
    *error = message;
}

// The queue both backends fill and recv() drains.
class EventQueue {
public:
  void push(WsEvent e) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events_.push_back(std::move(e));
    }
    cv_.notify_all();
  }
  std::optional<WsEvent> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (events_.empty())
      return std::nullopt;
    WsEvent e = std::move(events_.front());
    events_.pop_front();
    return e;
  }
  // Wait up to `secs` (in slices, checking `cancelled`) for an event; native only.
  std::optional<WsEvent> wait(double secs, const std::function<bool()> &cancelled) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(secs > 0 ? secs : 0));
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      if (!events_.empty()) {
        WsEvent e = std::move(events_.front());
        events_.pop_front();
        return e;
      }
      if (cancelled && cancelled())
        return std::nullopt;
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
        return std::nullopt;
      cv_.wait_until(lock, (std::min)(deadline, now + std::chrono::milliseconds(50)));
    }
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<WsEvent> events_;
};

// The close code at the start of a close frame's payload (RFC 6455 5.5.1), 1005 when absent.
int close_code(const std::string &payload) {
  if (payload.size() < 2)
    return 1005;
  return (static_cast<unsigned char>(payload[0]) << 8) | static_cast<unsigned char>(payload[1]);
}

#if defined(CVC_NET_WS_CURL)

void ensure_curl_global() {
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

class CurlWsConnection : public WsConnection {
public:
  CurlWsConnection(CURL *h, std::size_t max_bytes) : h_(h), max_bytes_(max_bytes) {
    curl_easy_getinfo(h_, CURLINFO_ACTIVESOCKET, &sock_);
    open_ = true;
    reader_ = std::thread([this] { read_loop(); });
  }
  ~CurlWsConnection() override {
    close(1001);
    stop_ = true;
    if (reader_.joinable())
      reader_.join();
    curl_easy_cleanup(h_);
  }

  std::optional<WsEvent> recv(double timeout_secs,
                              const std::function<bool()> &cancelled) override {
    return queue_.wait(timeout_secs, cancelled);
  }

  bool send_text(const std::string &text, std::string *error) override {
    if (!open_) {
      set_error(error, "ws: the connection is closed");
      return false;
    }
    return send_frame(text.data(), text.size(), CURLWS_TEXT, error);
  }

  void close(int code) override {
    bool was = open_.exchange(false);
    if (!was || closed_by_peer_)
      return;
    const char payload[2] = {static_cast<char>((code >> 8) & 0xff), static_cast<char>(code & 0xff)};
    send_frame(payload, 2, CURLWS_CLOSE, nullptr);
  }

  bool open() const override { return open_; }

private:
  bool wait_socket(bool for_write, int ms) {
#ifdef _WIN32
    WSAPOLLFD p{};
    p.fd = sock_;
    p.events = for_write ? POLLOUT : POLLIN;
    return WSAPoll(&p, 1, ms) > 0;
#else
    pollfd p{};
    p.fd = static_cast<int>(sock_);
    p.events = for_write ? POLLOUT : POLLIN;
    return ::poll(&p, 1, ms) > 0;
#endif
  }

  bool send_frame(const char *data, std::size_t len, unsigned int flags, std::string *error) {
    std::size_t done = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    // A zero-length message still needs one send.
    do {
      std::size_t sent = 0;
      CURLcode rc;
      {
        std::lock_guard<std::mutex> lock(handle_mutex_);
        rc = curl_ws_send(h_, data + done, len - done, &sent, 0, flags);
      }
      if (rc == CURLE_AGAIN) {
        if (std::chrono::steady_clock::now() > deadline) {
          set_error(error, "ws: send timed out");
          return false;
        }
        wait_socket(true, 50);
        continue;
      }
      if (rc != CURLE_OK) {
        set_error(error, std::string("ws: send failed: ") + curl_easy_strerror(rc));
        return false;
      }
      done += sent;
    } while (done < len);
    return true;
  }

  void read_loop() {
    std::string message; // the message being assembled
    bool binary = false;
    char buf[16384];
    while (!stop_) {
      std::size_t n = 0;
      const curl_ws_frame *meta = nullptr;
      CURLcode rc;
      {
        std::lock_guard<std::mutex> lock(handle_mutex_);
        rc = curl_ws_recv(h_, buf, sizeof buf, &n, &meta);
      }
      if (rc == CURLE_AGAIN) {
        wait_socket(false, 50);
        continue;
      }
      if (rc != CURLE_OK || !meta) {
        if (!stop_) {
          closed_by_peer_ = true;
          open_ = false;
          queue_.push(
              {WsEvent::Kind::Close, rc == CURLE_GOT_NOTHING ? "" : curl_easy_strerror(rc), 1006});
        }
        return;
      }
      if (meta->flags & CURLWS_CLOSE) {
        std::string payload(buf, n);
        closed_by_peer_ = true;
        open_ = false;
        queue_.push({WsEvent::Kind::Close, payload.size() > 2 ? payload.substr(2) : "",
                     close_code(payload)});
        return;
      }
      if (meta->flags & (CURLWS_PING | 0x40 /* CURLWS_PONG */))
        continue; // libcurl answers pings itself
      if (message.empty())
        binary = (meta->flags & CURLWS_BINARY) != 0;
      if (message.size() + n > max_bytes_) {
        close(1009); // first, so open() is false by the time the error can be read
        queue_.push({WsEvent::Kind::Error, "ws: a message is longer than the limit", 0});
        return;
      }
      message.append(buf, n);
      // A message is whole when this frame has no bytes left and no continuation follows.
      if (meta->bytesleft == 0 && !(meta->flags & CURLWS_CONT)) {
        queue_.push({binary ? WsEvent::Kind::Binary : WsEvent::Kind::Text, std::move(message), 0});
        message.clear();
      }
    }
  }

  CURL *h_;
  std::size_t max_bytes_;
  curl_socket_t sock_ = CURL_SOCKET_BAD;
  std::mutex handle_mutex_; // the easy handle is not thread-safe: sends and receives take turns
  std::atomic<bool> open_{false};
  std::atomic<bool> closed_by_peer_{false};
  std::atomic<bool> stop_{false};
  EventQueue queue_;
  std::thread reader_;
};

#elif defined(CVC_NET_WS_WEB)

class WebWsConnection : public WsConnection, public std::enable_shared_from_this<WebWsConnection> {
public:
  explicit WebWsConnection(EMSCRIPTEN_WEBSOCKET_T s) : s_(s) {
    emscripten_websocket_set_onopen_callback(s_, this, on_open);
    emscripten_websocket_set_onmessage_callback(s_, this, on_message);
    emscripten_websocket_set_onerror_callback(s_, this, on_error);
    emscripten_websocket_set_onclose_callback(s_, this, on_close);
  }
  ~WebWsConnection() override {
    clear_timer(connect_timer_);
    clear_timer(wait_timer_);
    close(1001);
    emscripten_websocket_delete(s_); // its callbacks are dropped: none runs after this
  }

  // ws_start's connect: `done` once the socket opens, fails or `secs` pass. The connection keeps
  // itself alive until then (self_); the function returned abandons it.
  std::function<void()> start_connect(double secs, WsConnected done) {
    connect_done_ = std::move(done);
    self_ = shared_from_this();
    connect_timer_ = emscripten_set_timeout(on_connect_timeout, secs * 1000.0, this);
    return [weak = std::weak_ptr<WebWsConnection>(shared_from_this())] {
      if (std::shared_ptr<WebWsConnection> c = weak.lock())
        c->abandon_connect();
    };
  }

  // Not blocking (WsConnection::recv_async): a queued event now, else the next one the callbacks
  // queue, else nullopt when the timer fires.
  std::function<void()> recv_async(double timeout_secs, WsReceived done) override {
    if (std::optional<WsEvent> e = queue_.pop()) {
      done(std::move(e));
      return {};
    }
    if (!(timeout_secs > 0)) {
      done(std::nullopt);
      return {};
    }
    clear_timer(wait_timer_);
    waiter_ = std::move(done);
    wait_timer_ = emscripten_set_timeout(on_wait_timeout, timeout_secs * 1000.0, this);
    return [weak = std::weak_ptr<WebWsConnection>(shared_from_this())] {
      if (std::shared_ptr<WebWsConnection> c = weak.lock()) {
        c->clear_timer(c->wait_timer_);
        c->waiter_ = nullptr;
      }
    };
  }

  // Wait for the open (or a failure) after ws_connect, yielding to the browser between checks.
  bool wait_open(double secs, std::string *error) {
    const double until = emscripten_get_now() + secs * 1000.0;
    while (!open_ && !failed_ && emscripten_get_now() < until)
      emscripten_sleep(10);
    if (open_)
      return true;
    set_error(error, failed_ ? "ws: the connection failed" : "ws: the connection timed out");
    return false;
  }

  std::optional<WsEvent> recv(double timeout_secs,
                              const std::function<bool()> &cancelled) override {
    const double until = emscripten_get_now() + (timeout_secs > 0 ? timeout_secs : 0) * 1000.0;
    for (;;) {
      if (auto e = queue_.pop())
        return e;
      if ((cancelled && cancelled()) || emscripten_get_now() >= until)
        return std::nullopt;
      emscripten_sleep(10); // under Asyncify: the browser runs (and delivers) between checks
    }
  }

  bool send_text(const std::string &text, std::string *error) override {
    if (!open_) {
      set_error(error, "ws: the connection is closed");
      return false;
    }
    if (emscripten_websocket_send_utf8_text(s_, text.c_str()) != EMSCRIPTEN_RESULT_SUCCESS) {
      set_error(error, "ws: send failed");
      return false;
    }
    return true;
  }

  void close(int code) override {
    if (!open_.exchange(false))
      return;
    // A page may send only 1000 or 3000-4999 (WebSocket.close throws on any other code, 1001 and
    // 1013 included): send 1000 for a code it can't.
    if (code != 1000 && (code < 3000 || code > 4999))
      code = 1000;
    emscripten_websocket_close(s_, static_cast<unsigned short>(code), "");
  }

  bool open() const override { return open_; }

private:
  void clear_timer(int &id) {
    if (id)
      emscripten_clear_timeout(id);
    id = 0;
  }
  // A recv_async waiter takes what the callbacks just queued.
  void deliver() {
    if (!waiter_)
      return;
    std::optional<WsEvent> e = queue_.pop();
    if (!e)
      return;
    clear_timer(wait_timer_);
    WsReceived done = std::move(waiter_);
    waiter_ = nullptr;
    done(std::move(e));
  }
  // ws_start's connect is over: hand the connection (or the error) on, once.
  void end_connect(bool ok, const std::string &error) {
    clear_timer(connect_timer_);
    WsConnected done = std::move(connect_done_);
    connect_done_ = nullptr;
    std::shared_ptr<WebWsConnection> keep = std::move(self_);
    if (!ok)
      close_socket();
    if (done)
      done(ok ? keep : nullptr, error);
  }
  void abandon_connect() {
    clear_timer(connect_timer_);
    connect_done_ = nullptr;
    std::shared_ptr<WebWsConnection> keep = std::move(self_); // released as this returns
    close_socket();
  }
  // Close the browser's socket whatever its state (one still connecting too), with the one code
  // below 3000 a page may send.
  void close_socket() {
    open_ = false;
    emscripten_websocket_close(s_, 1000, "");
  }
  static void on_connect_timeout(void *self) {
    auto *c = static_cast<WebWsConnection *>(self);
    c->connect_timer_ = 0;
    if (c->connect_done_)
      c->end_connect(false, "ws: the connection timed out");
  }
  static void on_wait_timeout(void *self) {
    auto *c = static_cast<WebWsConnection *>(self);
    c->wait_timer_ = 0;
    if (WsReceived done = std::move(c->waiter_)) {
      c->waiter_ = nullptr;
      done(std::nullopt);
    }
  }

  static EM_BOOL on_open(int, const EmscriptenWebSocketOpenEvent *, void *self) {
    auto *c = static_cast<WebWsConnection *>(self);
    c->open_ = true;
    if (c->connect_done_)
      c->end_connect(true, std::string());
    return EM_TRUE;
  }
  static EM_BOOL on_message(int, const EmscriptenWebSocketMessageEvent *e, void *self) {
    auto *c = static_cast<WebWsConnection *>(self);
    if (e->isText) // numBytes counts the terminating NUL
      c->queue_.push({WsEvent::Kind::Text,
                      std::string(reinterpret_cast<const char *>(e->data),
                                  e->numBytes > 0 ? e->numBytes - 1 : 0),
                      0});
    else
      c->queue_.push({WsEvent::Kind::Binary,
                      std::string(reinterpret_cast<const char *>(e->data), e->numBytes), 0});
    c->deliver();
    return EM_TRUE;
  }
  static EM_BOOL on_error(int, const EmscriptenWebSocketErrorEvent *, void *self) {
    auto *c = static_cast<WebWsConnection *>(self);
    if (!c->open_)
      c->failed_ = true;
    if (c->connect_done_) { // ws_start's connect failed: no connection, no event
      c->end_connect(false, "ws: the connection failed");
      return EM_TRUE;
    }
    c->queue_.push({WsEvent::Kind::Error, "ws: the connection failed", 0});
    c->deliver();
    return EM_TRUE;
  }
  static EM_BOOL on_close(int, const EmscriptenWebSocketCloseEvent *e, void *self) {
    auto *c = static_cast<WebWsConnection *>(self);
    c->open_ = false;
    c->failed_ = true;
    if (c->connect_done_) {
      c->end_connect(false, "ws: the connection failed");
      return EM_TRUE;
    }
    c->queue_.push({WsEvent::Kind::Close, e->reason, e->code});
    c->deliver();
    return EM_TRUE;
  }

  EMSCRIPTEN_WEBSOCKET_T s_;
  std::atomic<bool> open_{false};
  std::atomic<bool> failed_{false};
  EventQueue queue_;
  // Not blocking (single-threaded: the callbacks, the timers and the callers share one thread).
  WsConnected connect_done_;              // ws_start's connect under way
  std::shared_ptr<WebWsConnection> self_; // ... keeps the connection alive meanwhile
  int connect_timer_ = 0;
  WsReceived waiter_; // a recv_async waiting
  int wait_timer_ = 0;
};

#endif

// The connector set_ws_connector put in place, or null for the compiled backend.
std::mutex &connector_mutex() {
  static std::mutex m;
  return m;
}
std::shared_ptr<WsConnector> &connector_cell() {
  static std::shared_ptr<WsConnector> c;
  return c;
}
std::shared_ptr<WsConnector> injected_connector() {
  std::lock_guard<std::mutex> lock(connector_mutex());
  return connector_cell();
}

std::shared_ptr<WsConnection> connect_compiled(const std::string &url, const WsOptions &opts,
                                               std::string *error);

} // namespace

std::function<void()> WsConnection::recv_async(double timeout_secs, WsReceived done) {
  std::optional<WsEvent> e = recv(timeout_secs);
  if (done)
    done(std::move(e));
  return {};
}

std::function<void()> WsConnector::start(const std::string &url, const WsOptions &opts,
                                         WsConnected done) {
  std::string error;
  std::shared_ptr<WsConnection> conn = connect(url, opts, &error);
  if (done)
    done(std::move(conn), std::move(error));
  return {};
}

void set_ws_connector(std::shared_ptr<WsConnector> connector) {
  std::lock_guard<std::mutex> lock(connector_mutex());
  connector_cell() = std::move(connector);
}

bool ws_starts_async() {
  if (std::shared_ptr<WsConnector> c = injected_connector())
    return c->starts_async();
#if defined(CVC_NET_WS_WEB) && !defined(__EMSCRIPTEN_PTHREADS__)
  return true; // the page's main thread runs the frames: a wait must not hold it
#else
  return false;
#endif
}

std::shared_ptr<WsConnection> ws_connect(const std::string &url, const WsOptions &opts,
                                         std::string *error) {
  if (std::shared_ptr<WsConnector> c = injected_connector())
    return c->connect(url, opts, error);
  return connect_compiled(url, opts, error);
}

std::function<void()> ws_start(const std::string &url, const WsOptions &opts, WsConnected done) {
  if (std::shared_ptr<WsConnector> c = injected_connector())
    return c->start(url, opts, std::move(done));
#if defined(CVC_NET_WS_WEB) && !defined(__EMSCRIPTEN_PTHREADS__)
  const auto fail = [&](const std::string &why) -> std::function<void()> {
    if (done)
      done(nullptr, why);
    return {};
  };
  if (!is_ws_url(url))
    return fail("ws: a WebSocket URL starts with ws:// or wss://: " + url);
  if (!opts.headers.empty())
    return fail("ws: a browser WebSocket can't send request headers (auth, headers)");
  if (!emscripten_websocket_is_supported())
    return fail("ws: this browser has no WebSocket");
  EmscriptenWebSocketCreateAttributes attrs;
  emscripten_websocket_init_create_attributes(&attrs);
  attrs.url = url.c_str();
  attrs.createOnMainThread = EM_TRUE;
  const EMSCRIPTEN_WEBSOCKET_T s = emscripten_websocket_new(&attrs);
  if (s <= 0)
    return fail("ws: could not create the WebSocket");
  return std::make_shared<WebWsConnection>(s)->start_connect(opts.connect_timeout_secs,
                                                             std::move(done));
#else
  std::string error;
  std::shared_ptr<WsConnection> conn = connect_compiled(url, opts, &error);
  if (done)
    done(std::move(conn), std::move(error));
  return {};
#endif
}

bool have_ws_backend() {
#if defined(CVC_NET_WS_CURL)
  ensure_curl_global();
  const curl_version_info_data *v = curl_version_info(CURLVERSION_NOW);
  for (const char *const *p = v ? v->protocols : nullptr; p && *p; ++p)
    if (std::strcmp(*p, "ws") == 0)
      return true;
  return false;
#elif defined(CVC_NET_WS_WEB)
  return emscripten_websocket_is_supported();
#else
  return false;
#endif
}

namespace {

std::shared_ptr<WsConnection> connect_compiled(const std::string &url, const WsOptions &opts,
                                               std::string *error) {
  if (!is_ws_url(url)) {
    set_error(error, "ws: a WebSocket URL starts with ws:// or wss://: " + url);
    return nullptr;
  }
#if defined(CVC_NET_WS_CURL)
  if (!have_ws_backend()) {
    set_error(error, "ws: this libcurl has no WebSocket support");
    return nullptr;
  }
  CURL *h = curl_easy_init();
  if (!h) {
    set_error(error, "ws: could not init libcurl");
    return nullptr;
  }
  struct curl_slist *headers = nullptr;
  for (const std::string &line : opts.headers)
    headers = curl_slist_append(headers, line.c_str());
  curl_easy_setopt(h, CURLOPT_URL, url.c_str());
  curl_easy_setopt(h, CURLOPT_CONNECT_ONLY, 2L); // 2: do the WebSocket handshake, then stop
  curl_easy_setopt(h, CURLOPT_PROTOCOLS_STR, "ws,wss");
  curl_easy_setopt(h, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(h, CURLOPT_CONNECTTIMEOUT_MS,
                   static_cast<long>(opts.connect_timeout_secs * 1000.0));
  curl_easy_setopt(h, CURLOPT_TIMEOUT_MS, static_cast<long>(opts.connect_timeout_secs * 1000.0));
  curl_easy_setopt(h, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(h, CURLOPT_USERAGENT, "cvc-ari");
  const CURLcode rc = curl_easy_perform(h);
  curl_slist_free_all(headers); // the handshake is done: curl doesn't read them again
  if (rc != CURLE_OK) {
    long status = 0;
    curl_easy_getinfo(h, CURLINFO_RESPONSE_CODE, &status);
    set_error(error, std::string("ws: connect failed: ") + curl_easy_strerror(rc) +
                         (status > 0 ? " (HTTP " + std::to_string(status) + ")" : ""));
    curl_easy_cleanup(h);
    return nullptr;
  }
  // The handshake is over; from here the reader thread's polls bound each wait instead.
  curl_easy_setopt(h, CURLOPT_TIMEOUT_MS, 0L);
  return std::make_shared<CurlWsConnection>(h, opts.max_message_bytes);
#elif defined(CVC_NET_WS_WEB)
  if (!opts.headers.empty()) {
    set_error(error, "ws: a browser WebSocket can't send request headers (auth, headers)");
    return nullptr;
  }
  if (!emscripten_websocket_is_supported()) {
    set_error(error, "ws: this browser has no WebSocket");
    return nullptr;
  }
  EmscriptenWebSocketCreateAttributes attrs;
  emscripten_websocket_init_create_attributes(&attrs);
  attrs.url = url.c_str();
  attrs.createOnMainThread = EM_TRUE;
  const EMSCRIPTEN_WEBSOCKET_T s = emscripten_websocket_new(&attrs);
  if (s <= 0) {
    set_error(error, "ws: could not create the WebSocket");
    return nullptr;
  }
  auto c = std::make_shared<WebWsConnection>(s);
  if (!c->wait_open(opts.connect_timeout_secs, error))
    return nullptr;
  return c;
#else
  (void)opts;
  set_error(error, "ws: this build has no WebSocket backend");
  return nullptr;
#endif
}

} // namespace

} // namespace net
} // namespace cvc

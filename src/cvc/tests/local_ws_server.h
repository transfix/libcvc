// A localhost WebSocket server for tests (Boost.Beast): it binds an OS-assigned port on 127.0.0.1
// and runs `session` on each connection after the handshake, with the upgrade request's headers.
// The session owns the stream: it reads, writes and closes as the test needs.
#ifndef CVC_TESTS_LOCAL_WS_SERVER_H
#define CVC_TESTS_LOCAL_WS_SERVER_H

#include <atomic>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace cvc_test {

class LocalWsServer {
public:
  using Stream = boost::beast::websocket::stream<boost::asio::ip::tcp::socket>;
  using Session = std::function<void(Stream &)>;

  explicit LocalWsServer(Session session)
      : session_(std::move(session)),
        acceptor_(io_,
                  boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0)) {
    port_ = acceptor_.local_endpoint().port();
    thread_ = std::thread([this] { serve(); });
  }
  ~LocalWsServer() {
    stop_ = true;
    boost::system::error_code ec; // wake the blocking accept
    boost::asio::io_context io;
    boost::asio::ip::tcp::socket poke(io);
    poke.connect({boost::asio::ip::make_address("127.0.0.1"), port_}, ec);
    if (thread_.joinable())
      thread_.join();
    for (std::thread &t : sessions_)
      if (t.joinable())
        t.join();
  }
  LocalWsServer(const LocalWsServer &) = delete;
  LocalWsServer &operator=(const LocalWsServer &) = delete;

  unsigned short port() const { return port_; }
  std::string url(const std::string &path) const {
    return "ws://127.0.0.1:" + std::to_string(port_) + path;
  }
  // The headers of every upgrade request so far, "Name: value" each.
  std::vector<std::vector<std::string>> handshakes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return handshakes_;
  }

private:
  void serve() {
    while (!stop_) {
      boost::asio::ip::tcp::socket sock(io_);
      boost::system::error_code ec;
      acceptor_.accept(sock, ec);
      if (ec || stop_)
        return;
      sessions_.emplace_back([this, s = std::move(sock)]() mutable {
        try {
          boost::beast::flat_buffer buf;
          boost::beast::http::request<boost::beast::http::string_body> req;
          boost::beast::http::read(s, buf, req);
          {
            std::vector<std::string> headers;
            for (const auto &f : req)
              headers.push_back(std::string(f.name_string()) + ": " + std::string(f.value()));
            std::lock_guard<std::mutex> lock(mutex_);
            handshakes_.push_back(std::move(headers));
          }
          Stream ws(std::move(s));
          ws.accept(req);
          session_(ws);
        } catch (...) {
          // The client went away mid-session; the test checks what it saw.
        }
      });
    }
  }

  Session session_;
  boost::asio::io_context io_;
  boost::asio::ip::tcp::acceptor acceptor_;
  unsigned short port_ = 0;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::vector<std::thread> sessions_;
  mutable std::mutex mutex_;
  std::vector<std::vector<std::string>> handshakes_;
};

} // namespace cvc_test

#endif // CVC_TESTS_LOCAL_WS_SERVER_H

// A localhost HTTP/1.1 server for tests: it binds an OS-assigned port on 127.0.0.1, answers each
// connection's one request with `handler`, and records every request it saw. Two of them are two
// origins (the port differs), which is what the network-policy tests need.
#ifndef CVC_TESTS_LOCAL_HTTP_SERVER_H
#define CVC_TESTS_LOCAL_HTTP_SERVER_H

#include <atomic>
#include <boost/asio.hpp>
#include <cctype>
#include <cstddef>
#include <functional>
#include <istream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace cvc_test {

class LocalHttpServer {
public:
  struct Request {
    std::string method, target;
    std::vector<std::string> headers; // "Name: value"
    std::string body;
    // The first header named `name` (case-insensitive), "" when absent.
    std::string header(const std::string &name) const {
      for (const std::string &h : headers) {
        const std::size_t colon = h.find(':');
        if (colon == std::string::npos || colon != name.size())
          continue;
        bool eq = true;
        for (std::size_t i = 0; i < colon && eq; ++i)
          eq = std::tolower(static_cast<unsigned char>(h[i])) ==
               std::tolower(static_cast<unsigned char>(name[i]));
        if (!eq)
          continue;
        const std::size_t v = h.find_first_not_of(' ', colon + 1);
        return v == std::string::npos ? std::string() : h.substr(v);
      }
      return std::string();
    }
  };
  struct Reply {
    int status = 200;
    std::vector<std::string> headers;
    std::string body;
  };
  using Handler = std::function<Reply(const Request &)>;

  explicit LocalHttpServer(Handler handler)
      : handler_(std::move(handler)),
        acceptor_(io_,
                  boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0)) {
    port_ = acceptor_.local_endpoint().port();
    thread_ = std::thread([this] { serve(); });
  }
  ~LocalHttpServer() {
    stop_ = true;
    boost::system::error_code ec; // wake the blocking accept
    boost::asio::io_context io;
    boost::asio::ip::tcp::socket poke(io);
    poke.connect({boost::asio::ip::make_address("127.0.0.1"), port_}, ec);
    if (thread_.joinable())
      thread_.join();
  }
  LocalHttpServer(const LocalHttpServer &) = delete;
  LocalHttpServer &operator=(const LocalHttpServer &) = delete;

  unsigned short port() const { return port_; }
  std::string origin() const { return "http://127.0.0.1:" + std::to_string(port_); }
  std::string url(const std::string &path) const { return origin() + path; }
  std::vector<Request> requests() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

private:
  void serve() {
    while (!stop_) {
      boost::asio::ip::tcp::socket sock(io_);
      boost::system::error_code ec;
      acceptor_.accept(sock, ec);
      if (ec || stop_)
        return;
      Request req;
      if (!read_request(sock, req))
        continue;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.push_back(req);
      }
      const Reply r = handler_(req);
      std::string out = "HTTP/1.1 " + std::to_string(r.status) + " X\r\n";
      for (const std::string &h : r.headers)
        out += h + "\r\n";
      out += "Content-Length: " + std::to_string(r.body.size()) + "\r\nConnection: close\r\n\r\n";
      if (req.method != "HEAD")
        out += r.body;
      boost::asio::write(sock, boost::asio::buffer(out), ec);
      sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    }
  }

  static bool read_request(boost::asio::ip::tcp::socket &sock, Request &req) {
    boost::asio::streambuf buf;
    boost::system::error_code ec;
    boost::asio::read_until(sock, buf, "\r\n\r\n", ec);
    if (ec)
      return false;
    std::istream in(&buf);
    std::string line;
    std::getline(in, line);
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    const std::size_t sp1 = line.find(' '), sp2 = line.rfind(' ');
    if (sp1 == std::string::npos || sp2 == sp1)
      return false;
    req.method = line.substr(0, sp1);
    req.target = line.substr(sp1 + 1, sp2 - sp1 - 1);
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      if (line.empty())
        break;
      req.headers.push_back(line);
    }
    const std::string length = req.header("Content-Length");
    const std::size_t want = length.empty() ? 0 : std::stoul(length);
    std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (body.size() < want) {
      std::string rest(want - body.size(), '\0');
      boost::asio::read(sock, boost::asio::buffer(rest), ec);
      body += rest;
    }
    req.body = body.substr(0, want);
    return true;
  }

  Handler handler_;
  boost::asio::io_context io_;
  boost::asio::ip::tcp::acceptor acceptor_;
  unsigned short port_ = 0;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::vector<Request> requests_;
};

} // namespace cvc_test

#endif // CVC_TESTS_LOCAL_HTTP_SERVER_H

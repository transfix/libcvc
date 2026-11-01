// cvc::net — the loopback redirect URI of a native OAuth 2.0 client. See
// inc/cvc/net/loopback_redirect.h.

#include <chrono>
#include <cvc/net/loopback_redirect.h>
#include <string>
#include <vector>

#if !defined(__EMSCRIPTEN__)
#include <boost/asio.hpp>
#endif

namespace cvc {
namespace net {

#if defined(__EMSCRIPTEN__)

struct LoopbackRedirect::Impl {};
LoopbackRedirect::LoopbackRedirect() : m_(std::make_unique<Impl>()) {}
LoopbackRedirect::~LoopbackRedirect() = default;
bool LoopbackRedirect::available() { return false; }
bool LoopbackRedirect::listen(int, const std::string &, std::string *error) {
  if (error)
    *error = "a browser page cannot listen for a loopback redirect";
  return false;
}
std::string LoopbackRedirect::redirect_uri() const { return std::string(); }
std::optional<std::string> LoopbackRedirect::poll() { return std::nullopt; }
void LoopbackRedirect::respond(bool, const std::string &) {}
void LoopbackRedirect::close() {}

#else

namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace {

constexpr std::size_t kMaxRequest = 16 * 1024;
constexpr int kMaxPending = 8;
constexpr std::chrono::seconds kReadTimeout{10};

std::string html_escape(const std::string &s) {
  std::string out;
  for (char c : s) {
    switch (c) {
    case '&':
      out += "&amp;";
      break;
    case '<':
      out += "&lt;";
      break;
    case '>':
      out += "&gt;";
      break;
    case '"':
      out += "&quot;";
      break;
    default:
      out.push_back(c);
    }
  }
  return out;
}

void write_all(tcp::socket &sock, const std::string &text) {
  boost::system::error_code ec;
  sock.non_blocking(false, ec);
  asio::write(sock, asio::buffer(text), ec);
  sock.shutdown(tcp::socket::shutdown_both, ec);
  sock.close(ec);
}

std::string reply(int status, const char *reason, const std::string &html) {
  return "HTTP/1.1 " + std::to_string(status) + " " + reason +
         "\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\n"
         "Content-Security-Policy: default-src 'none'; style-src 'unsafe-inline'\r\n"
         "Referrer-Policy: no-referrer\r\nConnection: close\r\nContent-Length: " +
         std::to_string(html.size()) + "\r\n\r\n" + html;
}

} // namespace

struct LoopbackRedirect::Impl {
  struct Pending {
    tcp::socket sock;
    std::string data;
    std::chrono::steady_clock::time_point deadline;
  };
  asio::io_context io;
  std::unique_ptr<tcp::acceptor> acceptor;
  std::vector<std::unique_ptr<Pending>> pending;
  std::unique_ptr<tcp::socket> redirect; // the one redirect, held until respond()
  std::string path;
  unsigned short port = 0;
  bool taken = false; // the redirect has arrived (only one is accepted)

  // Read what a connection has sent. True when it is done with (answered or dropped); `query` is
  // set when it was the redirect.
  bool read(Pending &p, std::optional<std::string> &query) {
    char buf[2048];
    for (;;) {
      boost::system::error_code ec;
      const std::size_t n = p.sock.read_some(asio::buffer(buf), ec);
      if (ec == asio::error::would_block || ec == asio::error::try_again)
        break;
      if (ec || n == 0)
        return true; // closed before a whole request: drop it
      p.data.append(buf, n);
      if (p.data.size() > kMaxRequest)
        return true;
      if (p.data.find("\r\n\r\n") != std::string::npos)
        break;
    }
    const std::size_t head_end = p.data.find("\r\n\r\n");
    if (head_end == std::string::npos)
      return std::chrono::steady_clock::now() > p.deadline;
    const std::size_t line_end = p.data.find("\r\n");
    const std::string line = p.data.substr(0, line_end);
    const std::size_t sp1 = line.find(' ');
    const std::size_t sp2 = sp1 == std::string::npos ? sp1 : line.find(' ', sp1 + 1);
    const std::string method = line.substr(0, sp1);
    const std::string target =
        sp2 == std::string::npos ? std::string() : line.substr(sp1 + 1, sp2 - sp1 - 1);
    const std::size_t q = target.find('?');
    const std::string target_path = target.substr(0, q);
    if (method == "GET" && target_path == path && !taken) {
      taken = true;
      query = q == std::string::npos ? std::string() : target.substr(q + 1);
      redirect = std::make_unique<tcp::socket>(std::move(p.sock));
      return true;
    }
    write_all(p.sock, reply(404, "Not Found", "<!doctype html><title>Not found</title>"));
    return true;
  }
};

LoopbackRedirect::LoopbackRedirect() : m_(std::make_unique<Impl>()) {}
LoopbackRedirect::~LoopbackRedirect() { close(); }

bool LoopbackRedirect::available() { return true; }

bool LoopbackRedirect::listen(int port, const std::string &path, std::string *error) {
  close();
  if (path.empty() || path[0] != '/' || path.find_first_of("?# ") != std::string::npos) {
    if (error)
      *error = "the loopback redirect path must start with '/' and have no query";
    return false;
  }
  if (port < 0 || port > 65535) {
    if (error)
      *error = "the loopback port must be 0-65535";
    return false;
  }
  boost::system::error_code ec;
  auto acceptor = std::make_unique<tcp::acceptor>(m_->io);
  const tcp::endpoint at(asio::ip::make_address_v4("127.0.0.1"), static_cast<unsigned short>(port));
  acceptor->open(at.protocol(), ec);
  if (!ec)
    acceptor->bind(at, ec);
  if (!ec)
    acceptor->listen(asio::socket_base::max_listen_connections, ec);
  if (!ec)
    acceptor->non_blocking(true, ec);
  if (ec) {
    if (error)
      *error = "cannot listen on 127.0.0.1:" + std::to_string(port) + ": " + ec.message();
    return false;
  }
  m_->port = acceptor->local_endpoint(ec).port();
  m_->acceptor = std::move(acceptor);
  m_->path = path;
  m_->taken = false;
  return true;
}

std::string LoopbackRedirect::redirect_uri() const {
  if (!m_->acceptor)
    return std::string();
  return "http://127.0.0.1:" + std::to_string(m_->port) + m_->path;
}

std::optional<std::string> LoopbackRedirect::poll() {
  if (!m_->acceptor)
    return std::nullopt; // (after the redirect, other requests are still answered: 404)
  for (int k = 0; k < kMaxPending; ++k) {
    boost::system::error_code ec;
    tcp::socket sock(m_->io);
    m_->acceptor->accept(sock, ec);
    if (ec)
      break; // would_block: nobody is waiting
    sock.non_blocking(true, ec);
    if (static_cast<int>(m_->pending.size()) >= kMaxPending) {
      sock.close(ec);
      continue;
    }
    m_->pending.push_back(std::make_unique<Impl::Pending>(Impl::Pending{
        std::move(sock), std::string(), std::chrono::steady_clock::now() + kReadTimeout}));
  }
  std::optional<std::string> query;
  for (auto it = m_->pending.begin(); it != m_->pending.end();) {
    if (m_->read(**it, query)) {
      boost::system::error_code ec;
      (*it)->sock.close(ec);
      it = m_->pending.erase(it);
    } else {
      ++it;
    }
    if (query)
      break;
  }
  return query;
}

void LoopbackRedirect::respond(bool ok, const std::string &message) {
  if (m_->redirect) {
    const std::string title = ok ? "Signed in" : "Sign-in failed";
    const std::string html =
        "<!doctype html><html><head><meta charset=\"utf-8\"><title>" + title +
        "</title><style>body{font-family:system-ui,sans-serif;margin:3em auto;max-width:32em;"
        "line-height:1.5;padding:0 1em}h1{font-size:1.4em}</style></head><body><h1>" +
        title + "</h1><p>" + html_escape(message) +
        "</p><p>You can close this tab and go back to the application.</p></body></html>";
    write_all(*m_->redirect, reply(ok ? 200 : 400, ok ? "OK" : "Bad Request", html));
    m_->redirect.reset();
  }
  close();
}

void LoopbackRedirect::close() {
  boost::system::error_code ec;
  for (auto &p : m_->pending)
    p->sock.close(ec);
  m_->pending.clear();
  if (m_->redirect) {
    m_->redirect->close(ec);
    m_->redirect.reset();
  }
  if (m_->acceptor) {
    m_->acceptor->close(ec);
    m_->acceptor.reset();
  }
}

#endif

} // namespace net
} // namespace cvc

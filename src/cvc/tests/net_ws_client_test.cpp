// cvc::net's WebSocket client (inc/cvc/net/ws_client.h) against a localhost Beast server.

#include "local_ws_server.h"

#include <chrono>
#include <cvc/net/ws_client.h>
#include <gtest/gtest.h>
#include <string>

namespace {

using cvc::net::WsEvent;
using cvc_test::LocalWsServer;

#define REQUIRE_WS_BACKEND()                                                                       \
  if (!cvc::net::have_ws_backend())                                                                \
  GTEST_SKIP() << "libcvc built without a WebSocket backend"

void echo(LocalWsServer::Stream &ws) {
  for (;;) {
    boost::beast::flat_buffer buf;
    ws.read(buf);
    ws.text(ws.got_text());
    ws.write(buf.data());
  }
}

TEST(NetWsClient, HttpEquivalent) {
  EXPECT_EQ(cvc::net::ws_http_equivalent("ws://h:1/p?q"), "http://h:1/p?q");
  EXPECT_EQ(cvc::net::ws_http_equivalent("wss://h/live"), "https://h/live");
  EXPECT_EQ(cvc::net::ws_http_equivalent("https://h/"), "https://h/");
}

TEST(NetWsClient, RefusesANonWebSocketUrl) {
  std::string error;
  EXPECT_EQ(cvc::net::ws_connect("http://127.0.0.1:1/", {}, &error), nullptr);
  EXPECT_NE(error.find("ws://"), std::string::npos) << error;
}

TEST(NetWsClient, ConnectFailureIsAnError) {
  REQUIRE_WS_BACKEND();
  unsigned short port;
  { // a port nobody listens on any more
    boost::asio::io_context io;
    boost::asio::ip::tcp::acceptor a(
        io, boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0));
    port = a.local_endpoint().port();
  }
  cvc::net::WsOptions opts;
  opts.connect_timeout_secs = 5;
  std::string error;
  EXPECT_EQ(cvc::net::ws_connect("ws://127.0.0.1:" + std::to_string(port) + "/", opts, &error),
            nullptr);
  EXPECT_FALSE(error.empty());
}

TEST(NetWsClient, EchoesTextAndSendsHeaders) {
  REQUIRE_WS_BACKEND();
  LocalWsServer server(echo);
  cvc::net::WsOptions opts;
  opts.headers = {"X-Test: yes"};
  std::string error;
  auto c = cvc::net::ws_connect(server.url("/live"), opts, &error);
  ASSERT_TRUE(c) << error;
  EXPECT_TRUE(c->open());
  ASSERT_TRUE(c->send_text("hello", &error)) << error;
  auto e = c->recv(5);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->kind, WsEvent::Kind::Text);
  EXPECT_EQ(e->data, "hello");
  // An empty message is a message too.
  ASSERT_TRUE(c->send_text("", &error)) << error;
  e = c->recv(5);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->kind, WsEvent::Kind::Text);
  EXPECT_EQ(e->data, "");
  const auto handshakes = server.handshakes();
  ASSERT_EQ(handshakes.size(), 1u);
  bool saw = false;
  for (const std::string &h : handshakes[0])
    saw = saw || h == "X-Test: yes";
  EXPECT_TRUE(saw);
  c->close();
  EXPECT_FALSE(c->open());
}

TEST(NetWsClient, ReassemblesFragmentsAndKeepsBinary) {
  REQUIRE_WS_BACKEND();
  const std::string big(200000, 'x');
  LocalWsServer server([&](LocalWsServer::Stream &ws) {
    ws.auto_fragment(true);
    ws.write_buffer_bytes(4096); // many frames per message
    ws.text(true);
    ws.write(boost::asio::buffer(big));
    ws.binary(true);
    const std::string bin("\x00\x01\xff", 3);
    ws.write(boost::asio::buffer(bin));
    echo(ws);
  });
  auto c = cvc::net::ws_connect(server.url("/"), {});
  ASSERT_TRUE(c);
  auto e = c->recv(10);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->kind, WsEvent::Kind::Text);
  EXPECT_EQ(e->data.size(), big.size());
  EXPECT_EQ(e->data, big);
  e = c->recv(10);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->kind, WsEvent::Kind::Binary);
  EXPECT_EQ(e->data, std::string("\x00\x01\xff", 3));
}

TEST(NetWsClient, PeerCloseCarriesItsCode) {
  REQUIRE_WS_BACKEND();
  LocalWsServer server([](LocalWsServer::Stream &ws) {
    ws.close(boost::beast::websocket::close_reason(
        static_cast<boost::beast::websocket::close_code>(4004), "no live updates"));
    boost::beast::flat_buffer buf;
    boost::system::error_code ec;
    ws.read(buf, ec); // until the client's reply
  });
  auto c = cvc::net::ws_connect(server.url("/"), {});
  ASSERT_TRUE(c);
  auto e = c->recv(10);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->kind, WsEvent::Kind::Close);
  EXPECT_EQ(e->code, 4004);
  EXPECT_EQ(e->data, "no live updates");
  EXPECT_FALSE(c->open());
  std::string error;
  EXPECT_FALSE(c->send_text("late", &error));
  EXPECT_FALSE(error.empty());
}

TEST(NetWsClient, RecvTimesOutAndHonoursCancel) {
  REQUIRE_WS_BACKEND();
  LocalWsServer server(echo);
  auto c = cvc::net::ws_connect(server.url("/"), {});
  ASSERT_TRUE(c);
  auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(c->recv(0.2));
  EXPECT_GE(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(150));
  EXPECT_FALSE(c->recv(0)); // a poll never waits
  t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(c->recv(30, [] { return true; }));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(2));
}

TEST(NetWsClient, AnOversizedMessageIsAnError) {
  REQUIRE_WS_BACKEND();
  LocalWsServer server([](LocalWsServer::Stream &ws) {
    ws.text(true);
    ws.write(boost::asio::buffer(std::string(5000, 'y')));
    echo(ws);
  });
  cvc::net::WsOptions opts;
  opts.max_message_bytes = 1000;
  auto c = cvc::net::ws_connect(server.url("/"), opts);
  ASSERT_TRUE(c);
  auto e = c->recv(10);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->kind, WsEvent::Kind::Error);
  EXPECT_FALSE(c->open());
}

} // namespace

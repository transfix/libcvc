// relay_session.cpp — see the header: a host joins a relay as a client.

#include <cctype>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/conference/relay_session.h>
#include <cvc/ariadne/conference/room_calls.h>
#include <cvc/ariadne/loader.h> // register_ari_block, LoadResult
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_device_key.h> // load_device_seed, state_device_signer
#include <cvc/state/state_identity.h>   // default_device_id, scoped_device_id, valid_room_token
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <stdexcept>
#include <thread>
#ifdef CVC_ENABLE_GRPC
#include <cvc/state/state_transport_grpc.h>
#endif
#ifdef CVC_ENABLE_WS
#include <cvc/state/state_transport_ws.h>
#endif

namespace cvc {
namespace ariadne {
namespace conference {

namespace fs = std::filesystem;

namespace {

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }

// `path`, relative to the document's directory unless it is absolute.
std::string beside(const std::string &doc_path, const std::string &path) {
  const fs::path p(path);
  if (p.is_absolute() || doc_path.empty())
    return path;
  return (fs::path(doc_path).parent_path() / p).string();
}

bool read_file(const std::string &path, std::string &out) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    return false;
  out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return true;
}

// token: env:NAME or file:PATH (its first line) as the token; "" or the problem.
std::string resolve_token(const std::string &spec, const std::string &doc_path, std::string &out) {
  if (spec.empty())
    return std::string();
  if (spec == "oidc")
    return "relay: token: oidc is not supported yet (it comes with the relay's authenticate "
           "hook, S5); give env:NAME or file:PATH";
  if (starts_with(spec, "env:")) {
    const char *v = std::getenv(spec.substr(4).c_str());
    if (!v || !*v)
      return "relay: token: " + spec + ": the variable is not set";
    out = v;
  } else if (starts_with(spec, "file:")) {
    const std::string path = beside(doc_path, spec.substr(5));
    std::ifstream in(path);
    if (!in || !std::getline(in, out))
      return "relay: token: cannot read '" + path + "'";
    while (!out.empty() && std::isspace(static_cast<unsigned char>(out.back())))
      out.pop_back();
  } else {
    return "relay: token: is env:NAME or file:PATH, never the token itself (a document is read "
           "by whoever has it)";
  }
  if (!cvc::valid_room_token(out))
    return "relay: token: " + spec +
           " is not a room token (16 to 256 characters from [A-Za-z0-9._~-])";
  return std::string();
}

// A participant's node id: `name` as a C identifier, plus a random per-launch suffix (the relay
// drops a write whose origin and sequence it has seen, so a relaunch under one id would be
// ignored, as arigl's ConferenceSession explains).
std::string node_id_for(const std::string &name) {
  std::string id;
  for (char c : name)
    id += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
  if (id.empty() || std::isdigit(static_cast<unsigned char>(id.front())))
    id.insert(0, "n_");
  std::random_device rd;
  const char *digits = "0123456789abcdefghijklmnopqrstuvwxyz";
  id += '_';
  for (int i = 0; i < 6; ++i)
    id += digits[rd() % 36];
  return id;
}

} // namespace

std::string relay_join_from_block(const Value &block, const std::string &doc_path,
                                  relay_join &out) {
  if (block.is_scalar()) { // relay: grpc://host:port
    out.url = block.as_string();
  } else if (block.is_map()) {
    for (const Value::Entry &e : block.entries) {
      const std::string &k = e.first;
      if (k != "url" && k != "room" && k != "token" && k != "ca" && k != "subscribe")
        return "relay: unknown key '" + k + "' (it takes url, room, token, ca, subscribe)";
    }
    if (const Value *v = block.find("url"))
      out.url = v->as_string();
    if (const Value *v = block.find("room"); v && !v->as_string().empty())
      out.room = v->as_string();
    if (const Value *v = block.find("token"))
      if (const std::string e = resolve_token(v->as_string(), doc_path, out.token); !e.empty())
        return e;
    if (const Value *v = block.find("ca")) {
      const std::string path = beside(doc_path, v->as_string());
      if (!read_file(path, out.ca_pem) || out.ca_pem.empty())
        return "relay: ca: cannot read '" + path + "'";
    }
    if (const Value *v = block.find("subscribe")) {
      if (v->is_scalar() && !v->as_string().empty())
        out.subscribe.push_back(v->as_string());
      else
        for (const Value &i : v->items)
          if (!i.as_string().empty())
            out.subscribe.push_back(i.as_string());
    }
  } else {
    return "relay: is a URL, or a map of url, room, token, ca and subscribe";
  }
  if (out.url.empty())
    return "relay: names no url";
  return std::string();
}

struct relay_session::impl {
  explicit impl(cvc::app &a) : app(a) {}
  cvc::app &app;
  relay_join join;
  std::string pid;
  bool joined = false;
  // The shard first: the transport (whose destructor stops its reader) goes before it.
  std::unique_ptr<cvc::state_cluster_shard> shard;
#ifdef CVC_ENABLE_GRPC
  std::unique_ptr<cvc::state_transport_grpc> grpc;
#endif
#ifdef CVC_ENABLE_WS
  std::unique_ptr<cvc::state_transport_ws> ws;
#endif
  cvc::state_transport *tp = nullptr;
  std::shared_ptr<room_call_client> calls;

  void leave() {
    if (calls)
      room_call_client::install(app, nullptr);
    calls.reset();
#ifdef CVC_ENABLE_GRPC
    if (grpc)
      grpc->stop();
#endif
#ifdef CVC_ENABLE_WS
    if (ws)
      ws->stop();
#endif
    if (shard)
      shard->detach();
    joined = false;
  }
};

relay_session::relay_session(cvc::app &app) : _impl(std::make_unique<impl>(app)) {}

relay_session::~relay_session() { _impl->leave(); }

std::string relay_session::join(const relay_join &j, std::chrono::milliseconds timeout) {
  impl &s = *_impl;
  if (s.joined)
    return "relay: this host has joined a relay already";
  s.join = j;
  const std::string &url = j.url;
  const bool grpc = starts_with(url, "grpc://"), grpcs = starts_with(url, "grpcs://");
  const bool ws = starts_with(url, "ws://");
  if (starts_with(url, "wss://"))
    return "relay: " + url +
           ": wss:// needs the native WebSocket client's TLS, which libcvc does not have yet; "
           "dial the relay's gRPC port with grpcs:// (arisrv --tls-cert) instead";
  if (!grpc && !grpcs && !ws)
    return "relay: " + url + ": the URL is grpc://, grpcs:// or ws://";
  std::string address = url.substr(grpcs ? 8 : 7);
  if ((grpc || grpcs) && !address.empty() && address.back() == '/')
    address.pop_back();
  if ((grpc || grpcs) && (address.empty() || address.find('/') != std::string::npos))
    return "relay: " + url + ": a gRPC relay is host:port, with no path (the room is room:)";

  s.pid = node_id_for(j.name);
  // As arigl's conference session: the shard replicates conf (this host's participant subtree)
  // and applies only what it subscribed to: conf, and the published namespaces.
  s.shard = std::make_unique<cvc::state_cluster_shard>(s.app, "conf", s.pid, "conf");
  s.shard->add_interest("conf");
  for (const std::string &ns : j.subscribe)
    s.shard->add_interest(ns);
  s.shard->set_enforce_interest(true);
  s.shard->attach();
  // Who this device is to the relay (state_identity.h, state_device_key.h), scoped to it.
  const std::string deviceId = cvc::scoped_device_id(cvc::default_device_id(), url);
  const std::shared_ptr<const cvc::state_device_signer> deviceKey =
      cvc::state_device_signer::derive(cvc::load_device_seed(), url);
  bool ok = false;
  if (grpc || grpcs) {
#ifdef CVC_ENABLE_GRPC
    s.grpc = std::make_unique<cvc::state_transport_grpc>();
    if (grpcs)
      s.grpc->set_tls_config({std::string(), std::string(), j.ca_pem, false, /*client_tls=*/true});
    s.grpc->set_auth_config({std::string(), j.token});
    s.grpc->set_device_id(deviceId);
    s.grpc->set_device_signer(deviceKey);
    s.grpc->start("127.0.0.1:0", s.pid, "conf"); // gRPC starts (an ephemeral port) even to dial
    s.grpc->register_shard(s.shard.get());
    s.tp = s.grpc.get();
    ok = s.grpc->connect_to_peer(address, timeout);
#else
    return "relay: " + url + ": this build has no gRPC (CVC_ENABLE_GRPC=OFF)";
#endif
  } else {
#ifdef CVC_ENABLE_WS
    s.ws = std::make_unique<cvc::state_transport_ws>();
    s.ws->set_auth_config({std::string(), j.token});
    s.ws->set_device_id(deviceId);
    s.ws->set_device_signer(deviceKey);
    s.ws->register_shard(s.shard.get());
    s.tp = s.ws.get();
    ok = s.ws->connect_to_peer(url, timeout);
#else
    return "relay: " + url + ": this build has no WebSocket transport (CVC_ENABLE_WS=OFF)";
#endif
  }
  if (!ok) {
    s.leave();
    return "relay: could not reach " + url;
  }
  s.shard->set_transport(s.tp);
  // Admitted once the room's tree arrives: the relay holds a new connection (its admission hold)
  // and drops what it would send it meanwhile, a call's reply among them.
  const std::string roomPath = "conf.rooms." + j.room;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!s.app.root().findDescendantShared(roomPath)) {
    if (std::chrono::steady_clock::now() >= deadline) {
      s.leave();
      return "relay: " + url + " admitted no room '" + j.room +
             "' in time (is it the relay's --room?)";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  s.calls = std::make_shared<room_call_client>(*s.shard, j.room, s.pid);
  room_call_client::install(s.app, s.calls);
  s.joined = true;
  return std::string();
}

void relay_session::pump() {
  if (_impl->joined && _impl->tp)
    _impl->tp->pump_all();
}

void relay_session::link_subscriptions(const std::string &prefix) {
  if (prefix.empty())
    return; // the document's root is the app's: the namespaces are at their own paths already
  cvc::state &root = cvc::state::instance(_impl->app);
  for (const std::string &ns : _impl->join.subscribe) {
    cvc::state &link = root(prefix + cvc::state::SEPARATOR + ns);
    link.linkTo(ns, cvc::state::link_mode::transparent);
    link.setLinkWritable(false);
  }
}

bool relay_session::joined() const { return _impl->joined; }
const std::string &relay_session::pid() const { return _impl->pid; }

std::string relay_session::describe() const {
  const relay_join &j = _impl->join;
  std::string d = j.url + " (room " + j.room + ") as " + _impl->pid;
  if (!j.subscribe.empty()) {
    d += ", mirroring ";
    for (std::size_t i = 0; i < j.subscribe.size(); ++i)
      d += (i ? ", " : "") + j.subscribe[i];
  }
  return d;
}

void register_relay_block() {
  register_ari_block("relay", [](const Value &content, LoadResult &out) {
    out.extras.emplace_back("relay", content);
  });
}

std::shared_ptr<relay_session> attach_relay(cvc::app &app, AppRuntime &rt, relay_join cli,
                                            std::function<void(const std::string &)> note) {
  register_relay_block();
  register_room_call_intrinsics(app);
  auto session = std::make_shared<relay_session>(app);
  std::weak_ptr<relay_session> weak = session;
  const std::string prefix = rt.prefix();
  rt.add_load_hook([weak, cli, prefix, note](LoadResult &lr, const std::string &path) {
    const std::shared_ptr<relay_session> s = weak.lock();
    if (!s || s->joined())
      return; // a reload keeps the relay it joined
    // The block, then the host's flags over it: --relay's URL (a block may name none then), room,
    // CA and token. Its subscriptions stay the block's.
    const Value *block = nullptr;
    for (const auto &[key, value] : lr.extras)
      if (key == "relay")
        block = &value;
    if (!block && cli.url.empty())
      return; // a local document
    relay_join j;
    j.name = cli.name;
    if (block) {
      relay_join from_block;
      from_block.url = cli.url; // so a block that names no url is no error under --relay
      if (const std::string e = relay_join_from_block(*block, path, from_block); !e.empty())
        throw std::runtime_error(e);
      j.url = from_block.url;
      j.room = from_block.room;
      j.token = from_block.token;
      j.ca_pem = from_block.ca_pem;
      j.subscribe = from_block.subscribe;
    }
    if (!cli.url.empty())
      j.url = cli.url;
    if (!cli.room.empty())
      j.room = cli.room;
    if (!cli.ca_pem.empty())
      j.ca_pem = cli.ca_pem;
    if (!cli.token.empty())
      j.token = cli.token;
    if (const std::string e = s->join(j, std::chrono::milliseconds(10000)); !e.empty())
      throw std::runtime_error(e);
    s->link_subscriptions(prefix);
    if (note)
      note("relay: joined " + s->describe());
  });
  rt.add_frame_hook([weak] {
    if (const std::shared_ptr<relay_session> s = weak.lock())
      s->pump();
  });
  return session;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

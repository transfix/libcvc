// relay_session.cpp — see the header: a host joins a relay as a client.

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/conference/relay_session.h>
#include <cvc/ariadne/conference/room_calls.h>
#include <cvc/ariadne/loader.h>   // register_ari_block, LoadResult
#include <cvc/ariadne/net_host.h> // net_host_access: the host's credentials
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/net/credentials.h> // CredentialStore, CredentialRenewer
#include <cvc/net/net_policy.h>  // parse_origin
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_device_key.h> // load_device_seed, state_device_signer
#include <cvc/state/state_identity.h>   // default_device_id, scoped_device_id, valid_bearer_token
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <thread>
#include <utility>
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

// A credential's name: 1 to 56 of A-Z a-z 0-9 . _ - (an (oidc-sign-in ...)'s "credential").
bool valid_credential_name(const std::string &name) {
  return !name.empty() && name.size() <= 56 &&
         std::all_of(name.begin(), name.end(), [](unsigned char c) {
           return std::isalnum(c) != 0 || c == '.' || c == '_' || c == '-';
         });
}

// token: env:NAME or file:PATH (its first line) as the token, or credential:NAME / oidc:NAME as the
// host's credential to read it from at each join; "" or the problem.
std::string resolve_token(const std::string &spec, const std::string &doc_path, std::string &out,
                          std::string &credential) {
  if (spec.empty())
    return std::string();
  if (spec == "oidc")
    return "relay: token: oidc names the sign-in's credential: oidc:NAME, as its (oidc-sign-in "
           "...) keeps it";
  if (starts_with(spec, "credential:") || starts_with(spec, "oidc:")) {
    credential = spec.substr(spec.find(':') + 1);
    if (!valid_credential_name(credential))
      return "relay: token: " + spec + ": a credential's name is 1 to 56 of A-Z a-z 0-9 . _ -";
    return std::string();
  }
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
    return "relay: token: is env:NAME, file:PATH, credential:NAME or oidc:NAME, never the token "
           "itself (a document is read by whoever has it)";
  }
  // A room token, or a bearer for the relay's authenticate hook (an OIDC access token, say).
  if (!cvc::valid_bearer_token(out))
    return "relay: token: " + spec + " is not a token (1 to 4096 characters from [A-Za-z0-9._~-])";
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
      if (const std::string e = resolve_token(v->as_string(), doc_path, out.token, out.credential);
          !e.empty())
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

namespace {

// The token credential `name` sends to `origin` from the host's credential store (net_host.h),
// renewed first when the relay refused `rejected` or the token is about to expire.
std::string host_credential_token(cvc::app &app, const std::string &name, const std::string &origin,
                                  const std::string &rejected, std::string &why) {
  const std::optional<NetAccess> access = net_host_access(app);
  if (!access || !access->credentials) {
    why = "the host keeps no credentials (turn the network on: --net-policy, --credential-store)";
    return std::string();
  }
  const std::optional<cvc::net::Origin> to = cvc::net::parse_origin(origin, &why);
  if (!to)
    return std::string();
  if (const std::shared_ptr<cvc::net::CredentialRenewer> renewer =
          access->credentials->renewer(name))
    if (!rejected.empty() || renewer->expiring())
      if (const std::string e = renewer->renew(rejected); !e.empty())
        why = "renewing credential " + name + ": " + e;
  const std::optional<std::string> token = access->credentials->token_for(name, *to);
  if (!token || token->empty()) {
    if (why.empty())
      why = "credential " + name + " holds no token for " + origin +
            " (sign in, or bind it there "
            "with a policy line: credential " +
            name + " " + origin + ")";
    return std::string();
  }
  why.clear();
  return *token;
}

} // namespace

std::string relay_credential_origin(const std::string &url) {
  std::string scheme;
  std::size_t at = 0;
  for (const auto &[prefix, origin_scheme] :
       {std::pair<const char *, const char *>{"grpcs://", "https"},
        {"wss://", "https"},
        {"grpc://", "http"},
        {"ws://", "http"}})
    if (starts_with(url, prefix)) {
      scheme = origin_scheme;
      at = std::strlen(prefix);
      break;
    }
  if (scheme.empty())
    return std::string();
  const std::string authority = url.substr(at, url.find('/', at) - at);
  return authority.empty() ? std::string() : scheme + "://" + authority;
}

// One connection to the relay: its transport, of the URL's kind.
struct relay_session::link {
#ifdef CVC_ENABLE_GRPC
  std::unique_ptr<cvc::state_transport_grpc> grpc;
#endif
#ifdef CVC_ENABLE_WS
  std::unique_ptr<cvc::state_transport_ws> ws;
#endif
  cvc::state_transport *tp = nullptr;

  void stop(cvc::state_cluster_shard *shard) {
#ifdef CVC_ENABLE_GRPC
    if (grpc) {
      grpc->stop();
      if (shard)
        grpc->unregister_shard(shard);
    }
#endif
#ifdef CVC_ENABLE_WS
    if (ws) {
      ws->stop();
      if (shard)
        ws->unregister_shard(shard);
    }
#endif
    tp = nullptr;
  }
  // The relay's Hello has arrived over it: the relay admitted it. False once it has closed.
  bool admitted(bool &gone) const {
    const std::vector<cvc::state_link_report> links = tp->link_reports();
    gone = links.empty();
    for (const cvc::state_link_report &l : links)
      if (!l.remote_node_id.empty())
        return true;
    return false;
  }
};

struct relay_session::impl {
  explicit impl(cvc::app &a)
      : app(a), source([&a](const std::string &name, const std::string &origin,
                            const std::string &rejected, std::string &why) {
          return host_credential_token(a, name, origin, rejected, why);
        }) {}
  // How a dial ended.
  enum class outcome { joined, no_token, unreachable, refused, silent };

  cvc::app &app;
  relay_join join;
  std::string pid, address, origin;
  bool grpc = false, grpcs = false, ws = false;
  std::string device_id;
  std::shared_ptr<const cvc::state_device_signer> device_key;
  token_source source;      // the host's credential store (host_credential_token) unless set
  std::string status_path;  // set_status_path: where status() and joins() are written
  std::string shown_status; // ... and what was written there last
  std::uint64_t shown_joins = ~std::uint64_t(0);
  // The shard first: a link (whose transport's destructor stops its reader) goes before it.
  std::unique_ptr<cvc::state_cluster_shard> shard;
  std::unique_ptr<link> live;
  std::shared_ptr<room_call_client> calls;

  enum class phase { idle, waiting, dialing, joined, backoff };
  phase now = phase::idle;
  bool started = false;
  std::uint64_t joins = 0;
  std::chrono::steady_clock::time_point next_try;
  std::chrono::seconds wait{1};
  std::string why;      // the last attempt's failure
  std::string rejected; // the token the relay last refused (its credential is renewed first)

  // The dial in flight, on its own thread; its result, taken by pump().
  std::thread dialer;
  std::mutex dial_mu;
  bool dial_done = false;
  std::unique_ptr<link> dialed;
  outcome dial_outcome = outcome::no_token;
  std::string dial_why, dial_token;

  // URL checks and the shard; "" or what is wrong with `j`.
  std::string prepare(const relay_join &j) {
    join = j;
    const std::string &url = j.url;
    grpc = starts_with(url, "grpc://");
    grpcs = starts_with(url, "grpcs://");
    ws = starts_with(url, "ws://");
    if (starts_with(url, "wss://"))
      return "relay: " + url +
             ": wss:// needs the native WebSocket client's TLS, which libcvc does not have yet; "
             "dial the relay's gRPC port with grpcs:// (arisrv --tls-cert) instead";
    if (!grpc && !grpcs && !ws)
      return "relay: " + url + ": the URL is grpc://, grpcs:// or ws://";
    address = url.substr(grpcs ? 8 : 7);
    if ((grpc || grpcs) && !address.empty() && address.back() == '/')
      address.pop_back();
    if ((grpc || grpcs) && (address.empty() || address.find('/') != std::string::npos))
      return "relay: " + url + ": a gRPC relay is host:port, with no path (the room is room:)";
#ifndef CVC_ENABLE_GRPC
    if (grpc || grpcs)
      return "relay: " + url + ": this build has no gRPC (CVC_ENABLE_GRPC=OFF)";
#endif
#ifndef CVC_ENABLE_WS
    if (ws)
      return "relay: " + url + ": this build has no WebSocket transport (CVC_ENABLE_WS=OFF)";
#endif
    origin = relay_credential_origin(url);
    pid = node_id_for(j.name);
    // As arigl's conference session: the shard replicates conf (this host's participant subtree)
    // and applies only what it subscribed to: conf, and the published namespaces.
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", pid, "conf");
    shard->add_interest("conf");
    for (const std::string &ns : j.subscribe)
      shard->add_interest(ns);
    shard->set_enforce_interest(true);
    shard->attach();
    // Who this device is to the relay (state_identity.h, state_device_key.h), scoped to it.
    device_id = cvc::scoped_device_id(cvc::default_device_id(), url);
    device_key = cvc::state_device_signer::derive(cvc::load_device_seed(), url);
    started = true;
    return std::string();
  }

  // The token for the next dial: the join's own, or its credential's; "" and `w` when none.
  std::string token(std::string &w) {
    if (join.credential.empty())
      return join.token;
    return source ? source(join.credential, origin, rejected, w) : std::string();
  }

  // Wording for a failed dial, after "relay: <url> ".
  static std::string said(outcome o, const std::string &w) {
    switch (o) {
    case outcome::no_token:
      return w;
    case outcome::unreachable:
      return "could not be reached";
    case outcome::refused:
      return "refused this host (unauthenticated: a wrong token, or the relay's authenticate "
             "hook turned it away)";
    case outcome::silent:
      return "admitted nothing in time";
    case outcome::joined:
      break;
    }
    return std::string();
  }

  // Connect with `token` and wait, at most `timeout`, for the relay's Hello. On this thread.
  std::unique_ptr<link> dial(const std::string &tok, std::chrono::milliseconds timeout,
                             outcome &o) {
    auto l = std::make_unique<link>();
    bool ok = false;
#ifdef CVC_ENABLE_GRPC
    if (grpc || grpcs) {
      l->grpc = std::make_unique<cvc::state_transport_grpc>();
      if (grpcs)
        l->grpc->set_tls_config(
            {std::string(), std::string(), join.ca_pem, false, /*client_tls=*/true});
      l->grpc->set_auth_config({std::string(), tok});
      l->grpc->set_device_id(device_id);
      l->grpc->set_device_signer(device_key);
      l->grpc->start("127.0.0.1:0", pid, "conf"); // gRPC starts (an ephemeral port) even to dial
      l->grpc->register_shard(shard.get());
      l->tp = l->grpc.get();
      ok = l->grpc->connect_to_peer(address, timeout);
    }
#endif
#ifdef CVC_ENABLE_WS
    if (ws) {
      l->ws = std::make_unique<cvc::state_transport_ws>();
      l->ws->set_auth_config({std::string(), tok});
      l->ws->set_device_id(device_id);
      l->ws->set_device_signer(device_key);
      l->ws->register_shard(shard.get());
      l->tp = l->ws.get();
      ok = l->ws->connect_to_peer(join.url, timeout);
    }
#endif
    // A relay that refuses the connection (a wrong token, or its authenticate hook turned it
    // away) closes it before its Hello, at once. (A WebSocket relay refuses the upgrade itself, so
    // there it looks like no answer.)
    if (!ok) {
      o = outcome::unreachable;
      l->stop(shard.get());
      return nullptr;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool gone = false;
    while (!l->admitted(gone) && !gone && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (gone || !l->admitted(gone)) {
      o = gone ? outcome::refused : outcome::silent;
      l->stop(shard.get());
      return nullptr;
    }
    o = outcome::joined;
    return l;
  }

  // A dial succeeded: it is the live link, and room-call reaches the relay through it.
  void take(std::unique_ptr<link> l) {
    live = std::move(l);
    shard->set_transport(live->tp);
    calls = std::make_shared<room_call_client>(*shard, join.room, pid);
    room_call_client::install(app, calls);
    now = phase::joined;
    wait = std::chrono::seconds(1);
    rejected.clear();
    why.clear();
    ++joins;
  }

  // The live link is gone: calls end unavailable, and the next dial is due after `wait`.
  void lose(const std::string &w) {
    if (calls) {
      room_call_client::install(app, nullptr);
      calls->fail_all("room-call: the relay closed this host's connection");
      calls.reset();
    }
    if (shard)
      shard->set_transport(nullptr);
    if (live)
      live->stop(shard.get());
    live.reset();
    back_off(w);
  }

  void back_off(const std::string &w) {
    why = w;
    now = phase::backoff;
    next_try = std::chrono::steady_clock::now() + wait;
    wait = (std::min)(wait * 2, std::chrono::seconds(30));
  }

  // Start a dial on its own thread: the token (a credential's, which may be renewed), then the
  // connection. pump() takes its result. A single-threaded browser build, where std::thread would
  // throw, dials in the frame instead, as a token join's dial does at the load.
  void start_dial() {
    if (dialer.joinable())
      dialer.join();
    now = phase::dialing;
    {
      std::lock_guard<std::mutex> lk(dial_mu);
      dial_done = false;
      dialed.reset();
    }
    auto run = [this] {
      std::string w;
      const std::string tok = token(w);
      outcome o = outcome::no_token;
      std::unique_ptr<link> l;
      if (join.credential.empty() || !tok.empty())
        l = dial(tok, std::chrono::milliseconds(10000), o);
      std::lock_guard<std::mutex> lk(dial_mu);
      dialed = std::move(l);
      dial_outcome = o;
      dial_why = said(o, w);
      dial_token = tok;
      dial_done = true;
    };
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
    run();
#else
    dialer = std::thread(run);
#endif
  }

  void leave() {
    if (dialer.joinable())
      dialer.join();
    if (calls)
      room_call_client::install(app, nullptr);
    calls.reset();
    if (dialed)
      dialed->stop(shard.get());
    dialed.reset();
    if (live)
      live->stop(shard.get());
    live.reset();
    if (shard)
      shard->detach();
    now = phase::idle;
  }
};

relay_session::relay_session(cvc::app &app) : _impl(std::make_shared<impl>(app)) {}

relay_session::~relay_session() { _impl->leave(); }

void relay_session::set_token_source(token_source source) { _impl->source = std::move(source); }

void relay_session::set_status_path(std::string path) { _impl->status_path = std::move(path); }

std::string relay_session::join(const relay_join &j, std::chrono::milliseconds timeout) {
  impl &s = *_impl;
  if (s.started)
    return "relay: this host has joined a relay already";
  if (const std::string e = s.prepare(j); !e.empty()) {
    s.started = false;
    return e;
  }
  std::string w;
  const std::string tok = s.token(w);
  if (!j.credential.empty() && tok.empty()) {
    s.leave();
    s.started = false;
    return "relay: " + j.url + ": " + w;
  }
  impl::outcome o = impl::outcome::no_token;
  std::unique_ptr<link> l = s.dial(tok, timeout, o);
  if (!l) {
    s.leave();
    s.started = false;
    return o == impl::outcome::unreachable ? "relay: could not reach " + j.url
                                           : "relay: " + j.url + " " + impl::said(o, w);
  }
  s.take(std::move(l));
  // The first join waits for the room's tree too, so the subscriptions' snapshot has arrived and a
  // relay with no such room is said to have none.
  const std::string roomPath = "conf.rooms." + j.room;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!s.app.root().findDescendantShared(roomPath)) {
    bool gone = false;
    (void)s.live->admitted(gone);
    if (gone || std::chrono::steady_clock::now() >= deadline) {
      s.leave();
      s.started = false;
      return gone ? "relay: " + j.url + " closed the connection as this host joined"
                  : "relay: " + j.url + " admitted no room '" + j.room +
                        "' in time (is it the relay's --room?)";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return std::string();
}

std::string relay_session::begin(const relay_join &j) {
  impl &s = *_impl;
  if (s.started)
    return "relay: this host has joined a relay already";
  if (const std::string e = s.prepare(j); !e.empty()) {
    s.started = false;
    return e;
  }
  s.start_dial();
  return std::string();
}

void relay_session::pump() {
  impl &s = *_impl;
  if (!s.started)
    return;
  step();
  // Where it stands, for the document (a "status" without the countdown, so it changes only when
  // the phase or the reason does).
  if (!s.status_path.empty()) {
    std::string shown = status();
    if (s.now == impl::phase::backoff)
      shown = "lost; dialing again" + (s.why.empty() ? std::string() : " (" + s.why + ")");
    cvc::state &root = cvc::state::instance(s.app);
    if (shown != s.shown_status) {
      root(s.status_path + ".status").value(shown);
      s.shown_status = shown;
    }
    if (s.joins != s.shown_joins) {
      root(s.status_path + ".joins").value(std::to_string(s.joins));
      s.shown_joins = s.joins;
    }
  }
}

void relay_session::step() {
  impl &s = *_impl;
  const auto t = std::chrono::steady_clock::now();
  switch (s.now) {
  case impl::phase::joined: {
    bool gone = false;
    (void)s.live->admitted(gone);
    if (gone) {
      std::fprintf(stderr, "relay: lost %s; dialing again\n", s.join.url.c_str());
      s.lose("the relay closed the connection");
      return;
    }
    s.live->tp->pump_all();
    return;
  }
  case impl::phase::dialing: {
    std::unique_ptr<link> l;
    std::string w, tok;
    impl::outcome o = impl::outcome::no_token;
    {
      std::lock_guard<std::mutex> lk(s.dial_mu);
      if (!s.dial_done)
        return;
      l = std::move(s.dialed);
      o = s.dial_outcome;
      w = s.dial_why;
      tok = s.dial_token;
      s.dial_done = false;
    }
    if (s.dialer.joinable())
      s.dialer.join();
    if (l) {
      s.take(std::move(l));
      std::fprintf(stderr, "relay: %s %s\n", s.joins > 1 ? "joined again" : "joined",
                   describe().c_str());
    } else if (o == impl::outcome::no_token) {
      s.why = w; // a credential with no token yet: look again in a second
      s.now = impl::phase::waiting;
      s.next_try = t + std::chrono::seconds(1);
    } else {
      if (o == impl::outcome::refused)
        s.rejected = tok; // its credential is renewed before the next dial
      s.back_off(w);
    }
    return;
  }
  case impl::phase::waiting:
  case impl::phase::backoff:
    if (t >= s.next_try)
      s.start_dial();
    return;
  case impl::phase::idle:
    return;
  }
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

bool relay_session::joined() const { return _impl->now == impl::phase::joined; }
bool relay_session::started() const { return _impl->started; }
std::uint64_t relay_session::joins() const { return _impl->joins; }
const std::string &relay_session::pid() const { return _impl->pid; }

std::string relay_session::status() const {
  const impl &s = *_impl;
  switch (s.now) {
  case impl::phase::joined:
    return "joined";
  case impl::phase::dialing:
    return "dialing";
  case impl::phase::waiting:
    return "waiting for credential " + s.join.credential +
           (s.why.empty() ? "" : " (" + s.why + ")");
  case impl::phase::backoff: {
    const auto left = std::chrono::duration_cast<std::chrono::seconds>(
        s.next_try - std::chrono::steady_clock::now());
    return "lost; dialing again in " + std::to_string((std::max)(left.count(), std::int64_t(0))) +
           " s" + (s.why.empty() ? "" : " (" + s.why + ")");
  }
  case impl::phase::idle:
    break;
  }
  return "not joined";
}

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

void register_relay_block(cvc::app &app) {
  register_ari_block(app, "relay", [](const Value &content, LoadResult &out) {
    out.extras.emplace_back("relay", content);
  });
}

std::shared_ptr<relay_session> attach_relay(cvc::app &app, AppRuntime &rt, relay_join cli,
                                            std::function<void(const std::string &)> note) {
  register_relay_block(app);
  register_room_call_intrinsics(app);
  auto session = std::make_shared<relay_session>(app);
  std::weak_ptr<relay_session> weak = session;
  const std::string prefix = rt.prefix();
  // relay.status and relay.joins under the document's root.
  session->set_status_path(prefix.empty() ? std::string("relay") : prefix + ".relay");
  rt.add_load_hook([weak, cli, prefix, note](LoadResult &lr, const std::string &path) {
    const std::shared_ptr<relay_session> s = weak.lock();
    if (!s || s->started())
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
      j.credential = from_block.credential;
      j.ca_pem = from_block.ca_pem;
      j.subscribe = from_block.subscribe;
    }
    if (!cli.url.empty())
      j.url = cli.url;
    if (!cli.room.empty())
      j.room = cli.room;
    if (!cli.ca_pem.empty())
      j.ca_pem = cli.ca_pem;
    if (!cli.token.empty()) {
      j.token = cli.token;
      j.credential.clear();
    }
    if (!cli.credential.empty()) {
      j.credential = cli.credential;
      j.token.clear();
    }
    if (!j.credential.empty()) {
      // A credential's token may come later (a sign-in this document starts): the join happens
      // from the frames once it is there.
      if (const std::string e = s->begin(j); !e.empty())
        throw std::runtime_error(e);
      s->link_subscriptions(prefix);
      if (note)
        note("relay: joining " + s->describe() + " once credential " + j.credential +
             " holds a token");
      return;
    }
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

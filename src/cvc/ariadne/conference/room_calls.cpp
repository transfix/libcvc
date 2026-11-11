// room_calls.cpp — see the header: room-call's wire format, the relay's side and a client's.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/conference/room_calls.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_exec/async_call.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h> // intrinsics_context
#include <cvc/state/state_exec/types.h>
#include <cvc/state/state_exec/value_cbor.h> // the wire: DSL values as typed CBOR
#include <cvc/state/state_message.h>
#include <cvc/state/state_transport.h>
#include <map>
#include <span>
#include <unordered_map>
#include <utility>

namespace cvc {
namespace ariadne {
namespace conference {

namespace se = cvc::state_exec;

namespace {

std::atomic<std::uint64_t> g_messages{0};

// The wire: DSL values as CBOR (value_cbor.h), symbols as themselves, so every data value keeps its
// type between client and relay. No handle table: a host object, like a function, doesn't cross.
const se::cbor_options kWire{nullptr, /*symbols=*/true};

// `v` on the wire. Throws a TypeError when it holds what doesn't cross.
std::string encode_wire(const se::value_t &v) {
  try {
    return se::encode_cbor(v, kWire);
  } catch (const std::invalid_argument &e) {
    throw se::dsl_error("TypeError", std::string("room-call: ") + e.what());
  }
}

// `data` read from the wire, or nil (and `error` set) when it isn't one item of it.
se::value_t decode_wire(std::string_view data, std::string *error = nullptr) {
  try {
    return se::decode_cbor(data, kWire);
  } catch (const std::exception &e) {
    if (error)
      *error = e.what();
    return se::value_t();
  }
}

std::string_view payload_of(const cvc::state_message &m) {
  return std::string_view(reinterpret_cast<const char *>(m.bytes.data()), m.bytes.size());
}

using se::field;

std::string text_of(const se::value_t *v) {
  if (!v)
    return std::string();
  if (const std::string *s = std::get_if<std::string>(&v->v))
    return *s;
  return std::string();
}

double number_of(const se::value_t *v) {
  if (!v)
    return 0.0;
  if (const double *d = std::get_if<double>(&v->v))
    return *d;
  if (const std::int64_t *i = std::get_if<std::int64_t>(&v->v))
    return static_cast<double>(*i);
  return 0.0;
}

cvc::state_message make_message(const std::string &room, const std::string &origin,
                                const std::string &pid, const char *type, const std::string &wire) {
  cvc::state_message m;
  m.cluster_id = "conf";
  m.origin_node_id = origin;
  m.message_id = std::string(std::strcmp(type, kCallType) == 0 ? "call-" : "call-reply-") + pid +
                 "-" + std::to_string(g_messages.fetch_add(1) + 1);
  m.path = call_request_path(room, pid);
  m.ttl_hops = 0;
  m.content_type = type;
  m.bytes.assign(wire.begin(), wire.end());
  return m;
}

// The participants' prefix the bus matches (whole segments: no trailing separator).
std::string participants_prefix(const std::string &room) {
  std::string p = room_participant_path(room, "");
  const std::string sep = cvc::state::SEPARATOR;
  if (p.size() >= sep.size() && p.compare(p.size() - sep.size(), sep.size(), sep) == 0)
    p.resize(p.size() - sep.size());
  return p;
}

} // namespace

std::shared_ptr<const cvc::state_delivery_filter> room_read_scope(std::vector<std::string> prefixes,
                                                                  const std::string &room,
                                                                  const std::string &pid) {
  const std::string own = room_participant_path(room, pid);
  // Every node above the participant's own (conf, conf.rooms, ..., its participants node).
  std::vector<std::string> above;
  for (std::size_t dot = own.find(cvc::state::SEPARATOR); dot != std::string::npos;
       dot = own.find(cvc::state::SEPARATOR, dot + 1))
    above.push_back(own.substr(0, dot));
  prefixes.push_back(own);
  return std::make_shared<cvc::state_prefix_filter>(std::move(prefixes), std::move(above));
}

std::string call_request_path(const std::string &room, const std::string &pid) {
  return room_participant_path(room, pid) + cvc::state::SEPARATOR + "call";
}

se::value_t room_call_request::to_value() const {
  return se::make_dict({{"pid", se::value_t(pid)},
                        {"id", se::value_t(id)},
                        {"fn", se::value_t(fn)},
                        {"args", args.is_nil() ? se::make_list() : args},
                        {"kwargs", kwargs.is_nil() ? se::make_dict() : kwargs},
                        {"key", se::value_t(key)},
                        {"timeout", se::value_t(timeout)}});
}

std::string parse_call(const std::string &pid, std::string_view data, room_call_request &out) {
  std::string error;
  const se::value_t v = decode_wire(data, &error);
  if (!error.empty())
    return "the call is not one CBOR item: " + error;
  if (!std::get_if<se::dict_ptr>(&v.v))
    return "the call is not a map";
  out = room_call_request{};
  out.pid = pid;
  out.id = text_of(field(v, "id"));
  out.fn = text_of(field(v, "fn"));
  if (out.id.empty() || out.id.size() > 128)
    return "the call has no id, or one longer than 128 characters";
  if (out.fn.empty() || out.fn.size() > 256)
    return "the call names no function, or one longer than 256 characters";
  const se::value_t *args = field(v, "args");
  if (args && !args->is_nil() && !std::get_if<se::list_ptr>(&args->v))
    return "the call's args is not a list";
  out.args = args && !args->is_nil() ? *args : se::make_list();
  const se::value_t *kwargs = field(v, "kwargs");
  if (kwargs && !kwargs->is_nil() && !std::get_if<se::dict_ptr>(&kwargs->v))
    return "the call's kwargs is not an object";
  out.kwargs = kwargs && !kwargs->is_nil() ? *kwargs : se::make_dict();
  out.key = text_of(field(v, "key"));
  if (out.key.empty())
    out.key = out.id;
  out.timeout = number_of(field(v, "timeout"));
  if (!(out.timeout >= 0.0))
    out.timeout = 0.0;
  return std::string();
}

namespace {
std::string error_wire(const std::string &id, const std::string &code, const std::string &message,
                       const se::value_t &detail) {
  const se::value_t reply = se::make_dict({{"id", se::value_t(id)},
                                           {"type", se::value_t(std::string("error"))},
                                           {"code", se::value_t(code)},
                                           {"message", se::value_t(message)},
                                           {"detail", detail}});
  try {
    return encode_wire(reply);
  } catch (const std::exception &) { // a detail that doesn't cross: the error goes without it
    return encode_wire(se::make_dict({{"id", se::value_t(id)},
                                      {"type", se::value_t(std::string("error"))},
                                      {"code", se::value_t(code)},
                                      {"message", se::value_t(message)},
                                      {"detail", se::value_t()}}));
  }
}
} // namespace

// ── the relay ──────────────────────────────────────────────────────────────────────────────────

struct room_call_relay::impl {
  impl(cvc::state_cluster_shard &s, std::string r) : shard(s), room(std::move(r)) {}
  cvc::state_cluster_shard &shard;
  std::string room;
  std::vector<cvc::state_transport *> planes;
  std::uint64_t sub = 0;
  std::mutex mu;
  std::vector<std::pair<std::string, std::string>> inbox; // {pid, wire}, from reader threads
  std::atomic<std::uint64_t> refused{0};
  std::size_t result_max = std::size_t(1) << 20;
  stats st;
};

room_call_relay::room_call_relay(cvc::state_cluster_shard &shard, std::string room)
    : _impl(std::make_shared<impl>(shard, std::move(room))) {
  std::weak_ptr<impl> w = _impl;
  // On a transport's reader thread, inside the bus's admit(): queue only.
  _impl->sub = shard.message_bus().subscribe(
      participants_prefix(_impl->room), [w](const cvc::state_message &m) {
        const auto i = w.lock();
        if (!i || m.content_type != kCallType)
          return;
        // A call speaks for the participant whose path it is at, and only it, to the relay alone.
        if (m.origin_node_id.empty() || m.ttl_hops != 0 ||
            m.path != call_request_path(i->room, m.origin_node_id)) {
          i->refused.fetch_add(1, std::memory_order_relaxed);
          return;
        }
        std::lock_guard<std::mutex> lk(i->mu);
        i->inbox.emplace_back(m.origin_node_id, std::string(payload_of(m)));
      });
}

room_call_relay::~room_call_relay() {
  if (_impl->sub)
    _impl->shard.message_bus().unsubscribe(_impl->sub);
}

void room_call_relay::add_transport(cvc::state_transport *tp) {
  if (tp)
    _impl->planes.push_back(tp);
}

void room_call_relay::set_result_max(std::size_t bytes) { _impl->result_max = bytes; }

std::vector<room_call_request> room_call_relay::take() {
  std::vector<std::pair<std::string, std::string>> in;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    in.swap(_impl->inbox);
  }
  std::vector<room_call_request> out;
  for (auto &[pid, wire] : in) {
    room_call_request r;
    if (const std::string e = parse_call(pid, wire, r); !e.empty()) {
      const se::value_t v = decode_wire(wire);
      error(pid, text_of(field(v, "id")), "invalid", e);
      continue;
    }
    ++_impl->st.calls;
    out.push_back(std::move(r));
  }
  return out;
}

std::size_t room_call_relay::send(const std::string &pid, const std::string &wire) {
  const cvc::state_message m =
      make_message(_impl->room, _impl->shard.local_node_id(), pid, kCallReplyType, wire);
  std::size_t n = 0;
  for (cvc::state_transport *tp : _impl->planes)
    n += tp->send_message_to_holder(pid, m);
  return n;
}

std::size_t room_call_relay::result(const std::string &pid, const std::string &id,
                                    const se::value_t &value) {
  std::string wire;
  try {
    wire = encode_wire(se::make_dict(
        {{"id", se::value_t(id)}, {"type", se::value_t(std::string("result"))}, {"value", value}}));
  } catch (const std::exception &e) {
    return error(pid, id, "internal",
                 std::string("the function's value can't be sent (") + e.what() + ")");
  }
  if (_impl->result_max && wire.size() > _impl->result_max)
    return error(pid, id, "too_large",
                 "the function's value is " + std::to_string(wire.size()) +
                     " bytes encoded, over the relay's cap of " +
                     std::to_string(_impl->result_max) + " (--call-result-max)");
  const std::size_t n = send(pid, wire);
  if (n)
    ++_impl->st.replies;
  return n;
}

std::size_t room_call_relay::progress(const std::string &pid, const std::string &id,
                                      double fraction, const std::string &message) {
  return send(pid, encode_wire(se::make_dict({{"id", se::value_t(id)},
                                              {"type", se::value_t(std::string("progress"))},
                                              {"fraction", se::value_t(fraction)},
                                              {"message", se::value_t(message)}})));
}

std::size_t room_call_relay::error(const std::string &pid, const std::string &id,
                                   const std::string &code, const std::string &message,
                                   const se::value_t &detail) {
  const std::size_t n = send(pid, error_wire(id, code, message, detail));
  if (n)
    ++_impl->st.replies;
  return n;
}

room_call_relay::stats room_call_relay::take_stats() {
  stats s = _impl->st;
  s.refused = _impl->refused.exchange(0);
  _impl->st = stats{};
  return s;
}

// ── a client ───────────────────────────────────────────────────────────────────────────────────

struct room_call_client::impl {
  impl(cvc::state_cluster_shard &s, std::string r, std::string p)
      : shard(s), room(std::move(r)), pid(std::move(p)) {}
  cvc::state_cluster_shard &shard;
  std::string room, pid;
  std::uint64_t sub = 0;
  std::atomic<std::uint64_t> next{0};
  struct pending {
    done_fn done;
    progress_fn progress;
  };
  std::mutex mu;
  std::unordered_map<std::string, pending> calls;

  void on_reply(std::string_view wire) {
    const se::value_t v = decode_wire(wire);
    const std::string id = text_of(field(v, "id"));
    const std::string type = text_of(field(v, "type"));
    pending p;
    {
      std::lock_guard<std::mutex> lk(mu);
      const auto it = calls.find(id);
      if (it == calls.end())
        return; // forgotten, or not ours
      if (type == "progress") {
        p.progress = it->second.progress;
      } else {
        p = std::move(it->second);
        calls.erase(it);
      }
    }
    if (type == "progress") {
      if (p.progress)
        p.progress(number_of(field(v, "fraction")), text_of(field(v, "message")));
      return;
    }
    outcome o;
    if (type == "result") {
      o.ok = true;
      if (const se::value_t *val = field(v, "value"))
        o.value = *val;
    } else {
      o.code = text_of(field(v, "code"));
      if (o.code.empty())
        o.code = "internal";
      o.message = text_of(field(v, "message"));
      if (const se::value_t *d = field(v, "detail"))
        o.detail = *d;
    }
    if (p.done)
      p.done(o);
  }
};

room_call_client::room_call_client(cvc::state_cluster_shard &shard, std::string room,
                                   std::string pid)
    : _impl(std::make_shared<impl>(shard, std::move(room), std::move(pid))) {
  std::weak_ptr<impl> w = _impl;
  _impl->sub = shard.message_bus().subscribe(
      call_request_path(_impl->room, _impl->pid), [w](const cvc::state_message &m) {
        const auto i = w.lock();
        if (i && m.content_type == kCallReplyType && m.origin_node_id != i->pid)
          i->on_reply(payload_of(m));
      });
}

room_call_client::~room_call_client() {
  if (_impl->sub)
    _impl->shard.message_bus().unsubscribe(_impl->sub);
  fail_all("the host left its relay");
}

const std::string &room_call_client::room() const { return _impl->room; }
const std::string &room_call_client::pid() const { return _impl->pid; }

std::string room_call_client::call(const std::string &fn, const se::value_t &args,
                                   const se::value_t &kwargs, const std::string &key,
                                   double timeout, done_fn done, progress_fn progress) {
  const std::string id = _impl->pid + "-" + std::to_string(_impl->next.fetch_add(1) + 1);
  // Encoded before the call is kept: an argument that doesn't cross throws to the caller.
  std::string wire =
      encode_wire(se::make_dict({{"id", se::value_t(id)},
                                 {"fn", se::value_t(fn)},
                                 {"args", args.is_nil() ? se::make_list() : args},
                                 {"kwargs", kwargs.is_nil() ? se::make_dict() : kwargs},
                                 {"key", se::value_t(key)},
                                 {"timeout", se::value_t(timeout > 0 ? timeout : 0.0)}}));
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    _impl->calls[id] = impl::pending{done, std::move(progress)};
  }
  cvc::state_message m = make_message(_impl->room, _impl->pid, _impl->pid, kCallType, wire);
  bool sent = false;
  try {
    sent = _impl->shard.send_message(m).status ==
           cvc::state_cluster_shard::send_message_result::status_kind::delivered;
  } catch (const std::exception &) {
    sent = false;
  }
  if (!sent) {
    impl::pending p;
    {
      std::lock_guard<std::mutex> lk(_impl->mu);
      const auto it = _impl->calls.find(id);
      if (it != _impl->calls.end()) {
        p = std::move(it->second);
        _impl->calls.erase(it);
      }
    }
    if (p.done) {
      outcome o;
      o.code = "unavailable";
      o.message = "room-call: the call could not be sent to the relay";
      p.done(o);
    }
  }
  return id;
}

void room_call_client::forget(const std::string &id) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->calls.erase(id);
}

void room_call_client::fail_all(const std::string &why) {
  std::unordered_map<std::string, impl::pending> calls;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    calls.swap(_impl->calls);
  }
  for (auto &[id, p] : calls)
    if (p.done) {
      outcome o;
      o.code = "unavailable";
      o.message = why;
      p.done(o);
    }
}

namespace {
std::mutex g_clients_mu;
std::map<const cvc::app *, std::shared_ptr<room_call_client>> &clients() {
  static std::map<const cvc::app *, std::shared_ptr<room_call_client>> c;
  return c;
}
} // namespace

void room_call_client::install(cvc::app &app, std::shared_ptr<room_call_client> client) {
  std::lock_guard<std::mutex> lk(g_clients_mu);
  if (client)
    clients()[&app] = std::move(client);
  else
    clients().erase(&app);
}

std::shared_ptr<room_call_client> room_call_client::for_app(cvc::app &app) {
  std::lock_guard<std::mutex> lk(g_clients_mu);
  const auto it = clients().find(&app);
  return it == clients().end() ? nullptr : it->second;
}

// ── the DSL ────────────────────────────────────────────────────────────────────────────────────

namespace {

// What one call needs, kept by the work until it settles.
struct call_state {
  std::mutex mu;
  std::condition_variable cv;
  bool done = false;
  room_call_client::outcome result;
};

// The DSL value a call ended with, or the exception it raises.
se::value_t value_of(const room_call_client::outcome &o) {
  if (o.ok)
    return o.value;
  const char *cls = o.code == "forbidden"           ? "PermissionError"
                    : o.code == "deadline_exceeded" ? "TimeoutError"
                                                    : "RuntimeError";
  throw se::dsl_error(cls, "room-call: " + o.code + ": " + o.message, "room-call",
                      {{"code", se::value_t(o.code)}, {"detail", o.detail}});
}

// A call's arguments: the function's name, then its positional arguments, or one dict of keyword
// arguments.
struct call_args {
  std::string fn;
  se::value_t args = se::make_list(), kwargs = se::make_dict();
};
call_args parse_args(const char *verb, std::span<const se::value_t> args) {
  if (args.empty())
    throw se::dsl_error("TypeError", std::string(verb) + ": expected the function's name first");
  const std::string *fn = std::get_if<std::string>(&args[0].v);
  if (!fn || fn->empty())
    throw se::dsl_error("TypeError", std::string(verb) + ": the function's name is a string");
  call_args out;
  out.fn = *fn;
  const std::span<const se::value_t> rest = args.subspan(1);
  if (rest.size() == 1 && std::get_if<se::dict_ptr>(&rest[0].v))
    out.kwargs = rest[0];
  else
    out.args = se::make_list(std::vector<se::value_t>(rest.begin(), rest.end()));
  return out;
}

// The work of one call: started where the process parks (the reply settles it on a reader
// thread), else waited for in one blocking call (init:).
se::async_work call_work(cvc::app &app, se::async_host &host, std::string prefix, call_args a,
                         std::string key, double timeout) {
  const std::string fn = a.fn;
  const auto progress = [&app, prefix, fn](double fraction, const std::string &message) {
    if (prefix.empty())
      return;
    cvc::state &root = cvc::state::instance(app);
    const std::string base = prefix + ".calls." + fn;
    root(base + ".progress").value(std::to_string(fraction));
    root(base + ".message").value(message);
  };
  se::async_work work;
  work.start = [&app, &host, a, key, timeout, progress](const std::shared_ptr<se::async_call> &c) {
    const std::shared_ptr<room_call_client> client = room_call_client::for_app(app);
    if (!client)
      throw se::dsl_error("RuntimeError",
                          "room-call: this document's host has joined no relay (a relay: block, "
                          "or the host's --relay), or lost the one it joined",
                          "room-call", {{"code", se::value_t(std::string("unavailable"))}});
    std::weak_ptr<se::async_call> wc = c;
    const std::string id = client->call(
        a.fn, a.args, a.kwargs, key, timeout,
        [&host, wc](const room_call_client::outcome &o) {
          const std::shared_ptr<se::async_call> call = wc.lock();
          if (!call || call->cancel_requested())
            return;
          bool settled = false;
          try {
            settled = call->settle(value_of(o), nullptr);
          } catch (...) {
            settled = call->settle(se::value_t(), std::current_exception());
          }
          if (settled)
            host.finish(call);
        },
        [wc, progress](double fraction, const std::string &message) {
          progress(fraction, message);
          if (const std::shared_ptr<se::async_call> call = wc.lock())
            call->post(se::make_dict(
                {{"fraction", se::value_t(fraction)}, {"message", se::value_t(message)}}));
        });
    c->on_cancel([client, id] { client->forget(id); });
  };
  work.kernel = [&app, a, key, timeout, progress](se::async_call &) -> se::value_t {
    const std::shared_ptr<room_call_client> client = room_call_client::for_app(app);
    if (!client)
      throw se::dsl_error("RuntimeError",
                          "room-call: this document's host has joined no relay (a relay: block, "
                          "or the host's --relay), or lost the one it joined",
                          "room-call", {{"code", se::value_t(std::string("unavailable"))}});
    auto s = std::make_shared<call_state>();
    const std::string id = client->call(
        a.fn, a.args, a.kwargs, key, timeout,
        [s](const room_call_client::outcome &o) {
          std::lock_guard<std::mutex> lk(s->mu);
          s->result = o;
          s->done = true;
          s->cv.notify_all();
        },
        progress);
    std::unique_lock<std::mutex> lk(s->mu);
    const double wait = timeout > 0 ? timeout + 5.0 : 120.0;
    if (!s->cv.wait_for(lk, std::chrono::duration<double>(wait), [&] { return s->done; })) {
      client->forget(id);
      throw se::dsl_error("TimeoutError", "room-call: " + a.fn + ": no reply from the relay");
    }
    return value_of(s->result);
  };
  return work;
}

} // namespace

void register_room_call_intrinsics(cvc::app &app) {
  static std::mutex mu;
  static std::vector<const cvc::app *> done;
  {
    std::lock_guard<std::mutex> lk(mu);
    for (const cvc::app *p : done)
      if (p == &app)
        return;
    done.push_back(&app);
  }
  app.exec_async_host();
  register_action_intrinsics(
      [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        se::async_host &host = app.exec_async_host();
        const std::string prefix = ictx.root_path;
        se::async_options io;
        io.io = true; // the relay's time, not the program's (as http-request's)
        // (room-call "fn" arg ...) / (room-call-async ...)
        se::register_async_fn(
            env, host, "room-call",
            [&app, &host, prefix](std::span<const se::value_t> args) -> se::async_work {
              call_args a = parse_args("room-call", args);
              return call_work(app, host, prefix, std::move(a), std::string(), 0.0);
            },
            io);
        // (room-call-opts (dict "key" K "timeout" S) "fn" arg ...) / (room-call-opts-async ...)
        se::register_async_fn(
            env, host, "room-call-opts",
            [&app, &host, prefix](std::span<const se::value_t> args) -> se::async_work {
              if (args.empty() || !std::get_if<se::dict_ptr>(&args[0].v))
                throw se::dsl_error(
                    "TypeError",
                    "room-call-opts: expected (dict \"key\" K \"timeout\" S) first, then the "
                    "function's name");
              const std::string key = text_of(field(args[0], "key"));
              const double timeout = number_of(field(args[0], "timeout"));
              call_args a = parse_args("room-call-opts", args.subspan(1));
              return call_work(app, host, prefix, std::move(a), key, timeout > 0 ? timeout : 0.0);
            },
            io);
      });
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

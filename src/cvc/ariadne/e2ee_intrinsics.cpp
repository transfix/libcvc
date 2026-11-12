// Ariadne — the e2ee-* verbs over an E2eeStore. See inc/cvc/ariadne/e2ee_intrinsics.h.

#include <cstdint>
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/e2ee_intrinsics.h>
#include <cvc/core/app.h>
#include <cvc/net/e2ee.h>
#include <cvc/state/state_exec/async_host.h> // register_async_fn
#include <cvc/state/state_exec/builtins.h>   // builtin_json_parse
#include <cvc/state/state_exec/types.h>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace se = cvc::state_exec;

namespace {

[[noreturn]] void misuse(const char *verb, const std::string &why) {
  throw std::runtime_error(std::string(verb) + ": " + why);
}

const se::value_t &arg(std::span<const se::value_t> args, std::size_t i, const char *verb) {
  if (i >= args.size())
    misuse(verb, "missing argument " + std::to_string(i + 1));
  return args[i];
}

std::string string_arg(std::span<const se::value_t> args, std::size_t i, const char *verb,
                       const char *what) {
  const se::value_t &v = arg(args, i, verb);
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  misuse(verb, std::string(what) + " must be a string");
}

std::string handle_arg(std::span<const se::value_t> args, std::size_t i, const char *verb) {
  std::string h = string_arg(args, i, verb, "the handle");
  if (!cvc::net::E2eeStore::valid_handle(h))
    misuse(verb, "'" + h + "' is not a handle (1-64 of A-Z a-z 0-9 . _ -)");
  return h;
}

// A conversation or message id: an int, or a string of one.
std::string id_text(const se::value_t &v) {
  if (const std::int64_t *i = std::get_if<std::int64_t>(&v.v))
    return std::to_string(*i);
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  if (const double *d = std::get_if<double>(&v.v);
      d && *d == static_cast<double>(static_cast<std::int64_t>(*d)))
    return std::to_string(static_cast<std::int64_t>(*d));
  return std::string();
}

std::string conv_arg(std::span<const se::value_t> args, std::size_t i, const char *verb) {
  std::string c = id_text(arg(args, i, verb));
  if (!cvc::net::E2eeStore::valid_conversation(c))
    misuse(verb, "the conversation must be an id (1-64 of A-Z a-z 0-9 _ -)");
  return c;
}

std::optional<std::int64_t> as_int(const se::value_t &v) {
  if (const std::int64_t *i = std::get_if<std::int64_t>(&v.v))
    return *i;
  if (const double *d = std::get_if<double>(&v.v);
      d && *d == static_cast<double>(static_cast<std::int64_t>(*d)))
    return static_cast<std::int64_t>(*d);
  if (const std::string *s = std::get_if<std::string>(&v.v); s && !s->empty()) {
    std::size_t used = 0;
    try {
      const long long n = std::stoll(*s, &used);
      if (used == s->size())
        return n;
    } catch (const std::exception &) {
    }
  }
  return std::nullopt;
}

std::int64_t int_arg(std::span<const se::value_t> args, std::size_t i, const char *verb,
                     const char *what) {
  std::optional<std::int64_t> n = as_int(arg(args, i, verb));
  if (!n)
    misuse(verb, std::string(what) + " must be a whole number");
  return *n;
}

const se::dict_ptr *as_dict(const se::value_t &v) { return std::get_if<se::dict_ptr>(&v.v); }

const se::value_t *field(const se::dict_ptr &d, const char *key) {
  const se::value_t *found = nullptr;
  for (const auto &kv : *d)
    if (kv.first == key)
      found = &kv.second;
  return found;
}

std::optional<std::string> str_field(const se::dict_ptr &d, const char *key) {
  const se::value_t *v = field(d, key);
  if (!v)
    return std::nullopt;
  if (const std::string *s = std::get_if<std::string>(&v->v))
    return *s;
  return std::nullopt;
}

std::optional<std::int64_t> int_field(const se::dict_ptr &d, const char *key) {
  const se::value_t *v = field(d, key);
  return v ? as_int(*v) : std::nullopt;
}

bool bool_field(const se::dict_ptr &d, const char *key) {
  const se::value_t *v = field(d, key);
  return v && v->is_truthy();
}

// A prekey bundle dict (GET /e2ee/keys/{username}).
cvc::net::E2eeBundle bundle_of(const se::value_t &v, const char *verb) {
  const se::dict_ptr *d = as_dict(v);
  if (!d)
    misuse(verb, "the bundle must be a dict (the GET /e2ee/keys/{username} reply)");
  cvc::net::E2eeBundle b;
  b.identity_key_ed25519 = str_field(*d, "identity_key_ed25519").value_or("");
  b.identity_key_x25519 = str_field(*d, "identity_key_x25519").value_or("");
  b.registration_id = int_field(*d, "registration_id").value_or(0);
  const se::value_t *spk = field(*d, "signed_prekey");
  const se::dict_ptr *sd = spk ? as_dict(*spk) : nullptr;
  if (!sd)
    misuse(verb, "the bundle has no signed_prekey");
  b.signed_prekey_id = int_field(*sd, "key_id").value_or(0);
  b.signed_prekey = str_field(*sd, "public_key").value_or("");
  b.signed_prekey_signature = str_field(*sd, "signature").value_or("");
  if (const se::value_t *opk = field(*d, "one_time_prekey"); opk && !opk->is_nil()) {
    const se::dict_ptr *od = as_dict(*opk);
    if (!od)
      misuse(verb, "the bundle's one_time_prekey must be a dict or nil");
    b.one_time_prekey = cvc::net::E2eePreKey{int_field(*od, "key_id").value_or(0),
                                             str_field(*od, "public_key").value_or("")};
  }
  return b;
}

// An encrypted message dict (GET /e2ee/dm/conversations/{id}/messages).
cvc::net::E2eeMessage message_of(const se::value_t &v, const char *verb) {
  const se::dict_ptr *d = as_dict(v);
  if (!d)
    misuse(verb, "the message must be a dict (an entry of GET .../messages)");
  cvc::net::E2eeMessage m;
  m.id = int_field(*d, "id").value_or(0);
  m.sender_id = int_field(*d, "sender_id").value_or(0);
  m.algorithm = str_field(*d, "algorithm").value_or("dr-v1");
  m.envelope = str_field(*d, "envelope");
  m.ciphertext = str_field(*d, "ciphertext");
  m.wrapped_key = str_field(*d, "wrapped_key");
  m.header = str_field(*d, "header");
  m.is_sender = bool_field(*d, "is_sender");
  m.locked = bool_field(*d, "locked");
  return m;
}

// A backup dict (GET /e2ee/backup).
cvc::net::E2eeBackup backup_of(const se::value_t &v, const char *verb) {
  const se::dict_ptr *d = as_dict(v);
  if (!d)
    misuse(verb, "the backup must be a dict (the GET /e2ee/backup reply)");
  cvc::net::E2eeBackup b;
  b.ciphertext = str_field(*d, "ciphertext").value_or("");
  b.kdf = str_field(*d, "kdf").value_or("argon2id");
  b.kdf_salt = str_field(*d, "kdf_salt").value_or("");
  b.nonce = str_field(*d, "nonce").value_or("");
  b.version = int_field(*d, "version").value_or(1);
  if (const se::value_t *p = field(*d, "kdf_params"); p && as_dict(*p)) {
    const se::dict_ptr &pd = *as_dict(*p);
    b.opslimit = int_field(pd, "opslimit").value_or(0);
    b.memlimit = int_field(pd, "memlimit").value_or(0);
    b.alg = str_field(pd, "alg").value_or("argon2id13");
  }
  return b;
}

se::value_t prekeys_value(const std::vector<cvc::net::E2eePreKey> &keys) {
  std::vector<se::value_t> out;
  for (const auto &k : keys)
    out.push_back(se::make_dict({{"key_id", se::value_t(k.key_id)}, {"public_key", k.public_key}}));
  return se::make_list(std::move(out));
}

se::value_t keys_value(const cvc::net::E2eePublicKeys &k) {
  se::value_t spk = se::make_dict({{"key_id", se::value_t(k.signed_prekey_id)},
                                   {"public_key", k.signed_prekey},
                                   {"signature", k.signed_prekey_signature}});
  se::value_t publish = se::make_dict({{"identity_key_ed25519", k.identity_key_ed25519},
                                       {"identity_key_x25519", k.identity_key_x25519},
                                       {"registration_id", se::value_t(k.registration_id)},
                                       {"signed_prekey", spk},
                                       {"one_time_prekeys", prekeys_value(k.one_time_prekeys)}});
  return se::make_dict(
      {{"identity_key_ed25519", k.identity_key_ed25519},
       {"identity_key_x25519", k.identity_key_x25519},
       {"registration_id", se::value_t(k.registration_id)},
       {"fingerprint", k.fingerprint},
       {"one_time_prekeys", se::value_t(static_cast<std::int64_t>(k.one_time_prekeys.size()))},
       {"publish", publish}});
}

se::value_t backup_value(const cvc::net::E2eeBackup &b) {
  return se::make_dict({{"ciphertext", b.ciphertext},
                        {"kdf", b.kdf},
                        {"kdf_salt", b.kdf_salt},
                        {"kdf_params", se::make_dict({{"opslimit", se::value_t(b.opslimit)},
                                                      {"memlimit", se::value_t(b.memlimit)},
                                                      {"alg", b.alg}})},
                        {"nonce", b.nonce},
                        {"version", se::value_t(b.version)}});
}

se::value_t json_value(const std::string &json) {
  const se::value_t text(json);
  return se::builtin_json_parse(std::span<const se::value_t>(&text, 1));
}

se::value_t opt_string(const std::optional<std::string> &s) {
  return s ? se::value_t(*s) : se::nil_value;
}

void register_verbs(const std::shared_ptr<se::environment> &env, se::async_host &host,
                    const std::shared_ptr<cvc::net::E2eeStore> &store,
                    const std::string &unavailable) {
  // (e2ee-status): whether the verbs are here, and why not. Always there once a host asked.
  se::builtins::register_fn(
      env, "e2ee-status", [store, unavailable](std::span<const se::value_t>) -> se::value_t {
        return se::make_dict({{"ok", se::value_t(store != nullptr)},
                              {"persistent", se::value_t(store && store->persistent())},
                              {"error", store ? std::string() : unavailable}});
      });
  if (!store)
    return;
  // Charged as I/O: Argon2id's second is the host's work, not the program's.
  se::async_options io;
  io.io = true;

  se::builtins::register_fn(env, "e2ee-identity",
                            [store](std::span<const se::value_t> args) -> se::value_t {
                              const std::string h = handle_arg(args, 0, "e2ee-identity");
                              std::optional<cvc::net::E2eePublicKeys> k = store->public_keys(h);
                              return k ? keys_value(*k) : se::nil_value;
                            });

  se::builtins::register_fn(
      env, "e2ee-create", [store](std::span<const se::value_t> args) -> se::value_t {
        const std::string h = handle_arg(args, 0, "e2ee-create");
        const std::int64_t n =
            args.size() > 1 ? int_arg(args, 1, "e2ee-create", "the prekey count") : 20;
        if (n < 0 || n > 1000)
          misuse("e2ee-create", "the prekey count is 0 to 1000");
        std::string error;
        std::optional<cvc::net::E2eePublicKeys> k =
            store->create_identity(h, static_cast<int>(n), &error);
        if (!k)
          misuse("e2ee-create", error);
        return keys_value(*k);
      });

  se::builtins::register_fn(
      env, "e2ee-prekeys", [store](std::span<const se::value_t> args) -> se::value_t {
        const std::string h = handle_arg(args, 0, "e2ee-prekeys");
        const std::int64_t n = int_arg(args, 1, "e2ee-prekeys", "the prekey count");
        if (n < 1 || n > 1000)
          misuse("e2ee-prekeys", "the prekey count is 1 to 1000");
        std::string error;
        std::optional<std::vector<cvc::net::E2eePreKey>> keys =
            store->add_prekeys(h, static_cast<int>(n), &error);
        if (!keys)
          misuse("e2ee-prekeys", error);
        return se::make_dict({{"one_time_prekeys", prekeys_value(*keys)}});
      });

  se::builtins::register_fn(env, "e2ee-forget",
                            [store](std::span<const se::value_t> args) -> se::value_t {
                              const std::string h = handle_arg(args, 0, "e2ee-forget");
                              std::string error;
                              const bool had = store->forget(h, &error);
                              if (!error.empty())
                                misuse("e2ee-forget", error);
                              return se::value_t(had);
                            });

  se::builtins::register_fn(env, "e2ee-session?",
                            [store](std::span<const se::value_t> args) -> se::value_t {
                              const std::string h = handle_arg(args, 0, "e2ee-session?");
                              const std::string c = conv_arg(args, 1, "e2ee-session?");
                              return se::value_t(store->has_session(h, c));
                            });

  se::builtins::register_fn(
      env, "e2ee-seal", [store](std::span<const se::value_t> args) -> se::value_t {
        const char *verb = "e2ee-seal";
        const std::string h = handle_arg(args, 0, verb);
        const std::string c = conv_arg(args, 1, verb);
        const std::int64_t peer = int_arg(args, 2, verb, "the peer's user id");
        const std::string text = string_arg(args, 3, verb, "the text");
        std::optional<cvc::net::E2eeBundle> bundle;
        if (args.size() > 4 && !args[4].is_nil())
          bundle = bundle_of(args[4], verb);
        cvc::net::E2eeSealed s = store->seal(h, c, text, bundle);
        se::value_t body = se::nil_value;
        if (s.ok) {
          std::vector<std::pair<std::string, se::value_t>> b{
              {"ciphertext", s.ciphertext},
              {"recipient_keys", se::make_list({se::make_dict({{"recipient", se::value_t(peer)},
                                                               {"wrapped_key", s.wrapped_key},
                                                               {"header", s.header}})})},
              {"algorithm", s.algorithm}};
          if (s.envelope)
            b.emplace_back("envelope", *s.envelope);
          body = se::make_dict(std::move(b));
        }
        return se::make_dict({{"ok", se::value_t(s.ok)},
                              {"need_bundle", se::value_t(s.need_bundle)},
                              {"error", s.error},
                              {"body", body}});
      });

  se::builtins::register_fn(
      env, "e2ee-seal-group", [store](std::span<const se::value_t> args) -> se::value_t {
        const char *verb = "e2ee-seal-group";
        const std::string h = handle_arg(args, 0, verb);
        const std::string c = conv_arg(args, 1, verb);
        const std::string text = string_arg(args, 2, verb, "the text");
        const se::list_ptr *list = std::get_if<se::list_ptr>(&arg(args, 3, verb).v);
        if (!list)
          misuse(verb, "the members must be a list");
        std::vector<cvc::net::E2eeGroupMember> members;
        for (const se::value_t &m : **list) {
          cvc::net::E2eeGroupMember gm;
          if (std::optional<std::int64_t> id = as_int(m)) {
            gm.user_id = *id;
          } else if (const se::dict_ptr *d = as_dict(m)) {
            std::optional<std::int64_t> id = int_field(*d, "user_id");
            if (!id)
              misuse(verb, "a member dict needs user_id");
            gm.user_id = *id;
            if (const se::value_t *b = field(*d, "bundle"); b && !b->is_nil())
              gm.bundle = bundle_of(*b, verb);
          } else {
            misuse(verb, "a member is a user id or (dict \"user_id\" ID \"bundle\" BUNDLE)");
          }
          members.push_back(std::move(gm));
        }
        cvc::net::E2eeGroupSealed s = store->seal_group(h, c, text, members);
        std::vector<se::value_t> need;
        for (std::int64_t id : s.need_bundles)
          need.emplace_back(id);
        se::value_t body = se::nil_value;
        if (s.ok) {
          std::vector<se::value_t> keys;
          for (const auto &k : s.recipient_keys)
            keys.push_back(se::make_dict({{"recipient", se::value_t(k.recipient)},
                                          {"wrapped_key", k.wrapped_key},
                                          {"header", k.header}}));
          body = se::make_dict({{"ciphertext", s.ciphertext},
                                {"recipient_keys", se::make_list(std::move(keys))},
                                {"algorithm", s.algorithm}});
        }
        return se::make_dict({{"ok", se::value_t(s.ok)},
                              {"need_bundles", se::make_list(std::move(need))},
                              {"error", s.error},
                              {"body", body}});
      });

  se::builtins::register_fn(env, "e2ee-sent",
                            [store](std::span<const se::value_t> args) -> se::value_t {
                              const char *verb = "e2ee-sent";
                              const std::string h = handle_arg(args, 0, verb);
                              const std::int64_t id = int_arg(args, 1, verb, "the message id");
                              const std::string text = string_arg(args, 2, verb, "the text");
                              std::string error;
                              if (!store->remember(h, id, text, &error))
                                misuse(verb, error);
                              return se::value_t(true);
                            });

  se::builtins::register_fn(env, "e2ee-open",
                            [store](std::span<const se::value_t> args) -> se::value_t {
                              const char *verb = "e2ee-open";
                              const std::string h = handle_arg(args, 0, verb);
                              const std::string c = conv_arg(args, 1, verb);
                              const cvc::net::E2eeMessage m = message_of(arg(args, 2, verb), verb);
                              const cvc::net::E2eeOpened o = store->open_message(h, c, m);
                              using S = cvc::net::E2eeOpened::Status;
                              const bool ok = o.status == S::ok || o.status == S::cached;
                              return se::make_dict({{"ok", se::value_t(ok)},
                                                    {"status", cvc::net::to_string(o.status)},
                                                    {"text", o.text},
                                                    {"attachments", json_value(o.attachments)},
                                                    {"error", o.error}});
                            });

  se::builtins::register_fn(
      env, "e2ee-fingerprint", [](std::span<const se::value_t> args) -> se::value_t {
        return se::value_t(
            cvc::net::e2ee_fingerprint(string_arg(args, 0, "e2ee-fingerprint", "the key")));
      });

  se::register_async_fn(
      env, host, "e2ee-backup",
      [store](std::span<const se::value_t> args) -> se::async_kernel {
        const std::string h = handle_arg(args, 0, "e2ee-backup");
        const std::string pass = string_arg(args, 1, "e2ee-backup", "the passphrase");
        return [store, h, pass](se::async_call &) -> se::value_t {
          std::string error;
          std::optional<cvc::net::E2eeBackup> b = store->backup(h, pass, &error);
          return se::make_dict({{"ok", se::value_t(b.has_value())},
                                {"error", error},
                                {"body", b ? backup_value(*b) : se::nil_value}});
        };
      },
      io);

  se::register_async_fn(
      env, host, "e2ee-restore",
      [store](std::span<const se::value_t> args) -> se::async_kernel {
        const std::string h = handle_arg(args, 0, "e2ee-restore");
        const cvc::net::E2eeBackup blob = backup_of(arg(args, 1, "e2ee-restore"), "e2ee-restore");
        const std::string pass = string_arg(args, 2, "e2ee-restore", "the passphrase");
        return [store, h, blob, pass](se::async_call &) -> se::value_t {
          std::string error;
          std::optional<cvc::net::E2eePublicKeys> k = store->restore(h, blob, pass, &error);
          return se::make_dict({{"ok", se::value_t(k.has_value())},
                                {"error", error},
                                {"identity", k ? keys_value(*k) : se::nil_value}});
        };
      },
      io);
}

// The app's E2EE host: the store its verbs use, or why there is none. One per app
// (app.slot), so a second enable_e2ee or register_e2ee_intrinsics replaces the store the
// verbs see without registering them twice.
struct E2eeHost {
  static constexpr const char *app_slot = "cvc.ariadne.e2ee_host";
  std::mutex mutex;
  std::shared_ptr<cvc::net::E2eeStore> store;
  std::string path;        // the store's file ("" in memory)
  std::string unavailable; // why there is no store
  bool registered = false;
};

void use_store(cvc::app &app, std::shared_ptr<cvc::net::E2eeStore> store, std::string path,
               std::string unavailable) {
  E2eeHost &h = app.slot<E2eeHost>();
  bool register_now = false;
  {
    std::lock_guard<std::mutex> lock(h.mutex);
    h.store = std::move(store);
    h.path = std::move(path);
    h.unavailable = std::move(unavailable);
    register_now = !h.registered;
    h.registered = true;
  }
  if (!register_now)
    return;
  // Warm the lazy per-app pieces on this thread before a background kernel touches them (as
  // register_net_intrinsics does).
  app.computePool();
  app.exec_scheduler();
  se::async_host &host = app.exec_async_host();
  E2eeHost *hp = &h; // the slot lives as long as the app, and the provider goes with it
  register_action_intrinsics(
      app, [hp, &host](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        std::shared_ptr<cvc::net::E2eeStore> store;
        std::string unavailable;
        {
          std::lock_guard<std::mutex> lock(hp->mutex);
          store = hp->store;
          unavailable = hp->unavailable;
        }
        register_verbs(env, host, store, unavailable);
      });
}

} // namespace

void register_e2ee_intrinsics(cvc::app &app, std::shared_ptr<cvc::net::E2eeStore> store) {
  if (!store)
    return;
  std::string path = store->path();
  use_store(app, std::move(store), std::move(path), std::string());
}

std::string enable_e2ee(cvc::app &app, const std::string &spec) {
  if (spec.empty())
    return std::string();
  std::string why;
  std::string path;
#ifdef __EMSCRIPTEN__
  why = "a browser host keeps no E2EE store yet";
#else
  if (!cvc::net::e2ee_available())
    why = "this host was built without libsodium";
  else if ((path = cvc::net::E2eeStore::resolve_path(spec)).empty())
    why = "no config directory to keep '" + spec +
          "' in (set XDG_CONFIG_HOME or HOME, or give a path)";
#endif
  if (why.empty()) {
    E2eeHost &h = app.slot<E2eeHost>();
    {
      std::lock_guard<std::mutex> lock(h.mutex);
      if (h.store && h.path == path)
        return std::string(); // already open, and this process holds its lock
    }
    cvc::net::E2eeStore::Options o;
    o.path = path;
    auto store = std::make_shared<cvc::net::E2eeStore>(o);
    if (store->open(&why)) {
      use_store(app, std::move(store), path, std::string());
      return std::string();
    }
  }
  use_store(app, nullptr, path, why);
  return "--e2ee-store: " + why;
}

} // namespace ariadne
} // namespace cvc

// cvc::net — TX.WTF's end-to-end encrypted DMs, with the keys in the host. See inc/cvc/net/e2ee.h
// and docs/E2EE.md.
//
// A port of atx-crypto-club/txwtf txwtf/static/js/e2ee.js (TxwtfE2EE) and messaging-e2ee.js
// (TxwtfMessagingE2EE) over libsodium. Each function below names the JavaScript it ports, and does
// what it does in the same order, random draws included: e2ee_test.cpp replays e2ee.js's vectors
// through a seeded stream and compares every byte. Where the port differs it is safer and still
// wire-compatible, and says so:
//   - decrypting works on a copy of the session and keeps it only once the message has opened
//     (e2ee.js advances the ratchet first, so one forged message breaks the session);
//   - headers are checked (a 32-byte dh, whole counters below 2^31) before anything is derived;
//   - skipped keys are bounded per session (Options::max_skipped), as is the plaintext cache;
//   - a backup's Argon2id limits are bounded (Options::max_opslimit, max_memlimit).

#include "json_lite.h"
#include "private_file.h"

#include <algorithm>
#include <cstring>
#include <cvc/net/e2ee.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

#ifdef CVC_HAVE_SODIUM
#include <sodium.h>
#endif

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#if defined(__EMSCRIPTEN__)
#include <sys/random.h> // getentropy
#include <unistd.h>
#endif

namespace cvc {
namespace net {

namespace fs = std::filesystem;
namespace jl = json_lite;

const char *to_string(E2eeOpened::Status s) {
  switch (s) {
  case E2eeOpened::Status::ok:
    return "ok";
  case E2eeOpened::Status::cached:
    return "cached";
  case E2eeOpened::Status::own:
    return "own";
  case E2eeOpened::Status::locked:
    return "locked";
  case E2eeOpened::Status::no_session:
    return "no-session";
  case E2eeOpened::Status::failed:
    break;
  }
  return "failed";
}

namespace {

bool fail(std::string *error, std::string message) {
  if (error)
    *error = std::move(message);
  return false;
}

// Why an operation stopped. Never carries key material: messages are written here, never built
// from what was being decrypted.
struct E2eeError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

[[noreturn]] void raise(const std::string &why) { throw E2eeError(why); }

bool handle_char(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' ||
         c == '_' || c == '-';
}

// --- JSON out, as JSON.stringify writes it -----------------------------------------------------

// Replace bytes that are not UTF-8 with U+FFFD, as a JavaScript string round trip through
// TextEncoder would (it never holds invalid UTF-8).
std::string well_formed_utf8(std::string_view s) {
  std::string out;
  out.reserve(s.size());
  std::size_t i = 0;
  while (i < s.size()) {
    const auto c = static_cast<unsigned char>(s[i]);
    std::size_t n = 0;
    std::uint32_t cp = 0;
    if (c < 0x80) {
      out.push_back(static_cast<char>(c));
      ++i;
      continue;
    } else if ((c & 0xE0) == 0xC0) {
      n = 2;
      cp = c & 0x1F;
    } else if ((c & 0xF0) == 0xE0) {
      n = 3;
      cp = c & 0x0F;
    } else if ((c & 0xF8) == 0xF0) {
      n = 4;
      cp = c & 0x07;
    }
    bool ok = n != 0 && i + n <= s.size();
    for (std::size_t k = 1; ok && k < n; ++k) {
      const auto d = static_cast<unsigned char>(s[i + k]);
      ok = (d & 0xC0) == 0x80;
      cp = (cp << 6) | (d & 0x3F);
    }
    if (ok) {
      const bool overlong =
          (n == 2 && cp < 0x80) || (n == 3 && cp < 0x800) || (n == 4 && cp < 0x10000);
      ok = !overlong && cp <= 0x10FFFF && !(cp >= 0xD800 && cp <= 0xDFFF);
    }
    if (ok) {
      out.append(s.substr(i, n));
      i += n;
    } else {
      out.append("\xEF\xBF\xBD");
      ++i;
    }
  }
  return out;
}

std::string jstr(std::string_view s) {
  const std::string u = well_formed_utf8(s);
  std::string out = "\"";
  static constexpr char hex[] = "0123456789abcdef";
  for (const char ch : u) {
    const auto c = static_cast<unsigned char>(ch);
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\b':
      out += "\\b";
      break;
    case '\f':
      out += "\\f";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (c < 0x20) {
        out += "\\u00";
        out.push_back(hex[c >> 4]);
        out.push_back(hex[c & 0xF]);
      } else {
        out.push_back(ch);
      }
    }
  }
  out.push_back('"');
  return out;
}

std::string jnum(std::int64_t v) { return std::to_string(v); }
std::string jbool(bool b) { return b ? "true" : "false"; }
std::string jopt(const std::optional<std::string> &s) { return s ? jstr(*s) : "null"; }

using Members = std::vector<std::pair<std::string, std::string>>; // name -> rendered value

std::string jobj(const Members &m) {
  std::string out = "{";
  for (std::size_t i = 0; i < m.size(); ++i) {
    if (i)
      out.push_back(',');
    out += jstr(m[i].first);
    out.push_back(':');
    out += m[i].second;
  }
  out.push_back('}');
  return out;
}

// Re-render a parsed value (JSON.stringify of what JSON.parse gave, members in document order).
std::string render(const jl::Value &v) {
  switch (v.kind) {
  case jl::Value::Kind::null:
    return "null";
  case jl::Value::Kind::boolean:
    return jbool(v.b);
  case jl::Value::Kind::number:
    if (v.integral)
      return jnum(v.i);
    {
      std::ostringstream o;
      o.precision(17);
      o << v.num;
      return o.str();
    }
  case jl::Value::Kind::string:
    return jstr(v.str);
  case jl::Value::Kind::array: {
    std::string out = "[";
    for (std::size_t i = 0; i < v.items.size(); ++i) {
      if (i)
        out.push_back(',');
      out += render(v.items[i]);
    }
    return out + "]";
  }
  case jl::Value::Kind::object: {
    Members m;
    for (const auto &kv : v.members)
      m.emplace_back(kv.first, render(kv.second));
    return jobj(m);
  }
  }
  return "null";
}

// --- JSON in --------------------------------------------------------------------------------

jl::Value parse_json(const std::string &text, const char *what) {
  std::string why;
  std::optional<jl::Value> v = jl::parse(text, &why);
  if (!v)
    raise(std::string(what) + " is not JSON");
  return std::move(*v);
}

const jl::Value &member(const jl::Value &o, const char *key, const char *what) {
  const jl::Value *v = o.is_object() ? o.get(key) : nullptr;
  if (!v)
    raise(std::string(what) + " has no " + key);
  return *v;
}

std::string str_member(const jl::Value &o, const char *key, const char *what) {
  const jl::Value &v = member(o, key, what);
  if (!v.is_string())
    raise(std::string(what) + "'s " + key + " is not a string");
  return v.str;
}

std::optional<std::string> opt_str_member(const jl::Value &o, const char *key) {
  const jl::Value *v = o.is_object() ? o.get(key) : nullptr;
  if (!v || v->kind == jl::Value::Kind::null)
    return std::nullopt;
  if (!v->is_string())
    raise(std::string("'") + key + "' is not a string");
  return v->str;
}

std::int64_t int_member(const jl::Value &o, const char *key, const char *what) {
  const jl::Value &v = member(o, key, what);
  if (v.kind != jl::Value::Kind::number || !v.integral)
    raise(std::string(what) + "'s " + key + " is not a whole number");
  return v.i;
}

std::optional<std::int64_t> opt_int_member(const jl::Value &o, const char *key) {
  const jl::Value *v = o.is_object() ? o.get(key) : nullptr;
  if (!v || v->kind == jl::Value::Kind::null)
    return std::nullopt;
  if (v->kind != jl::Value::Kind::number || !v->integral)
    raise(std::string("'") + key + "' is not a whole number");
  return v->i;
}

bool bool_member(const jl::Value &o, const char *key) {
  const jl::Value *v = o.is_object() ? o.get(key) : nullptr;
  return v && v->kind == jl::Value::Kind::boolean && v->b;
}

} // namespace

#ifdef CVC_HAVE_SODIUM

namespace {

// --- bytes and secrets ------------------------------------------------------------------------

using Bytes = std::string; // octets

// Key material: wiped when it goes.
struct Secret {
  std::string b;
  Secret() = default;
  explicit Secret(std::string s) : b(std::move(s)) {}
  explicit Secret(std::size_t n) : b(n, '\0') {}
  Secret(const Secret &) = default;
  Secret(Secret &&o) noexcept : b(std::move(o.b)) { o.wipe(); }
  Secret &operator=(const Secret &o) {
    if (this != &o) {
      wipe();
      b = o.b;
    }
    return *this;
  }
  Secret &operator=(Secret &&o) noexcept {
    if (this != &o) {
      wipe();
      b = std::move(o.b);
      o.wipe();
    }
    return *this;
  }
  ~Secret() { wipe(); }
  void wipe() noexcept {
    if (!b.empty())
      sodium_memzero(b.data(), b.size());
    b.clear();
  }
  unsigned char *data() { return reinterpret_cast<unsigned char *>(b.data()); }
  const unsigned char *data() const { return reinterpret_cast<const unsigned char *>(b.data()); }
  std::size_t size() const { return b.size(); }
  bool empty() const { return b.empty(); }
};

const unsigned char *u8(const std::string &s) {
  return reinterpret_cast<const unsigned char *>(s.data());
}

void ensure_sodium() {
  if (sodium_init() < 0)
    raise("libsodium could not start");
}

// base64, libsodium's ORIGINAL variant (standard alphabet, padded): what e2ee.js's b64/unb64 and
// the server's Python base64 use.
std::string b64(const unsigned char *p, std::size_t n) {
  std::string out(sodium_base64_ENCODED_LEN(n, sodium_base64_VARIANT_ORIGINAL), '\0');
  sodium_bin2base64(out.data(), out.size(), p, n, sodium_base64_VARIANT_ORIGINAL);
  out.resize(std::strlen(out.c_str()));
  return out;
}
std::string b64(const std::string &bytes) { return b64(u8(bytes), bytes.size()); }
std::string b64(const Secret &s) { return b64(s.data(), s.size()); }

std::optional<std::string> unb64_opt(const std::string &text) {
  std::string out(text.size() / 4 * 3 + 3, '\0');
  std::size_t len = 0;
  const char *end = nullptr;
  if (sodium_base642bin(reinterpret_cast<unsigned char *>(out.data()), out.size(), text.data(),
                        text.size(), nullptr, &len, &end, sodium_base64_VARIANT_ORIGINAL) != 0 ||
      end != text.data() + text.size())
    return std::nullopt;
  out.resize(len);
  return out;
}

std::string unb64(const std::string &text, const char *what) {
  std::optional<std::string> b = unb64_opt(text);
  if (!b)
    raise(std::string(what) + " is not base64");
  return std::move(*b);
}

std::string unb64n(const std::string &text, std::size_t n, const char *what) {
  std::string b = unb64(text, what);
  if (b.size() != n)
    raise(std::string(what) + " is not " + std::to_string(n) + " bytes");
  return b;
}

Secret unb64_secret(const std::string &text, std::size_t n, const char *what) {
  Secret s(unb64(text, what));
  if (s.size() != n)
    raise(std::string(what) + " is not " + std::to_string(n) + " bytes");
  return s;
}

// --- randomness -------------------------------------------------------------------------------

class SystemRandom final : public E2eeRandom {
public:
  void fill(unsigned char *out, std::size_t n) override {
#if defined(__EMSCRIPTEN__)
    // libsodium's own source here is set up per thread (randombytes_stir); getentropy works on
    // every thread, a pthread's included.
    while (n > 0) {
      const std::size_t k = n > 256 ? 256 : n;
      if (::getentropy(out, k) != 0)
        raise("no randomness (getentropy)");
      out += k;
      n -= k;
    }
#else
    randombytes_buf(out, n);
#endif
  }
  std::uint32_t uniform(std::uint32_t upper) override {
#if defined(__EMSCRIPTEN__)
    // Rejection sampling, as randombytes_uniform does.
    const std::uint32_t min = static_cast<std::uint32_t>(-upper) % upper;
    std::uint32_t r = 0;
    do {
      fill(reinterpret_cast<unsigned char *>(&r), sizeof r);
    } while (r < min);
    return r % upper;
#else
    return randombytes_uniform(upper);
#endif
  }
#if defined(__EMSCRIPTEN__)
  bool seeds_keypairs() const override { return true; }
#endif
};

class SeededRandom final : public E2eeRandom {
public:
  explicit SeededRandom(std::string seed) : seed_(std::move(seed)) {}
  void fill(unsigned char *out, std::size_t n) override {
    while (n > 0) {
      if (pos_ == sizeof block_)
        refill();
      const std::size_t k = std::min(n, sizeof block_ - pos_);
      std::memcpy(out, block_ + pos_, k);
      pos_ += k;
      out += k;
      n -= k;
    }
  }
  std::uint32_t uniform(std::uint32_t upper) override {
    unsigned char b[4];
    fill(b, 4);
    const std::uint32_t v =
        static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
        (static_cast<std::uint32_t>(b[2]) << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
    return v % upper;
  }
  bool seeds_keypairs() const override { return true; }

private:
  void refill() {
    unsigned char idx[8];
    std::uint64_t c = counter_++;
    for (unsigned char &x : idx) {
      x = static_cast<unsigned char>(c & 0xFF);
      c >>= 8;
    }
    crypto_generichash(block_, sizeof block_, idx, sizeof idx, u8(seed_), seed_.size());
    pos_ = 0;
  }
  std::string seed_;
  std::uint64_t counter_ = 0;
  unsigned char block_[64] = {};
  std::size_t pos_ = sizeof block_;
};

Secret random_secret(E2eeRandom &r, std::size_t n) {
  Secret s(n);
  r.fill(s.data(), n);
  return s;
}

Bytes random_bytes(E2eeRandom &r, std::size_t n) {
  Bytes b(n, '\0');
  r.fill(reinterpret_cast<unsigned char *>(b.data()), n);
  return b;
}

// --- primitives -------------------------------------------------------------------------------

struct KeyPair {
  Bytes pub;
  Secret priv;
};

// s.crypto_box_keypair() (X25519).
KeyPair box_keypair(E2eeRandom &r) {
  KeyPair kp;
  kp.pub.assign(crypto_box_PUBLICKEYBYTES, '\0');
  kp.priv = Secret(crypto_box_SECRETKEYBYTES);
  auto *pk = reinterpret_cast<unsigned char *>(kp.pub.data());
  if (r.seeds_keypairs()) {
    Secret seed = random_secret(r, crypto_box_SEEDBYTES);
    crypto_box_seed_keypair(pk, kp.priv.data(), seed.data());
  } else {
    crypto_box_keypair(pk, kp.priv.data());
  }
  return kp;
}

// s.crypto_sign_keypair() (Ed25519; the secret is 64 bytes, seed then public key).
KeyPair sign_keypair(E2eeRandom &r) {
  KeyPair kp;
  kp.pub.assign(crypto_sign_PUBLICKEYBYTES, '\0');
  kp.priv = Secret(crypto_sign_SECRETKEYBYTES);
  auto *pk = reinterpret_cast<unsigned char *>(kp.pub.data());
  if (r.seeds_keypairs()) {
    Secret seed = random_secret(r, crypto_sign_SEEDBYTES);
    crypto_sign_seed_keypair(pk, kp.priv.data(), seed.data());
  } else {
    crypto_sign_keypair(pk, kp.priv.data());
  }
  return kp;
}

// _dh: crypto_scalarmult, which refuses a low-order public key (libsodium.js throws there too).
Secret dh(const Secret &priv, const Bytes &pub) {
  if (priv.size() != crypto_scalarmult_SCALARBYTES || pub.size() != crypto_scalarmult_BYTES)
    raise("a key is the wrong size");
  Secret out(crypto_scalarmult_BYTES);
  if (crypto_scalarmult(out.data(), priv.data(), u8(pub)) != 0)
    raise("a key exchange failed (a low-order public key)");
  return out;
}

Secret generichash(std::size_t outlen, const unsigned char *in, std::size_t inlen,
                   const unsigned char *key, std::size_t keylen) {
  Secret out(outlen);
  crypto_generichash(out.data(), outlen, in, inlen, key, keylen);
  return out;
}

// _kdfRK(rk, dhOut) -> [newRootKey, chainKey]: BLAKE2b keyed by rk, 64 bytes split 32/32.
std::pair<Secret, Secret> kdf_rk(const Secret &rk, const Secret &dh_out) {
  Secret out = generichash(64, dh_out.data(), dh_out.size(), rk.data(), rk.size());
  return {Secret(out.b.substr(0, 32)), Secret(out.b.substr(32, 32))};
}

// _kdfCK(ck) -> [nextChainKey, messageKey].
std::pair<Secret, Secret> kdf_ck(const Secret &ck) {
  const unsigned char one = 0x01, two = 0x02;
  Secret mk = generichash(32, &one, 1, ck.data(), ck.size());
  Secret next = generichash(32, &two, 1, ck.data(), ck.size());
  return {std::move(next), std::move(mk)};
}

Bytes aead_encrypt(const Bytes &pt, const Bytes &ad, const Bytes &nonce, const Secret &key) {
  Bytes ct(pt.size() + crypto_aead_xchacha20poly1305_ietf_ABYTES, '\0');
  unsigned long long clen = 0;
  crypto_aead_xchacha20poly1305_ietf_encrypt(reinterpret_cast<unsigned char *>(ct.data()), &clen,
                                             u8(pt), pt.size(), ad.empty() ? nullptr : u8(ad),
                                             ad.size(), nullptr, u8(nonce), key.data());
  ct.resize(static_cast<std::size_t>(clen));
  return ct;
}

std::optional<Secret> aead_decrypt(const Bytes &ct, const Bytes &ad, const Bytes &nonce,
                                   const Secret &key) {
  if (ct.size() < crypto_aead_xchacha20poly1305_ietf_ABYTES || key.size() != 32 ||
      nonce.size() != crypto_aead_xchacha20poly1305_ietf_NPUBBYTES)
    return std::nullopt;
  Secret pt(ct.size() - crypto_aead_xchacha20poly1305_ietf_ABYTES);
  unsigned long long plen = 0;
  if (crypto_aead_xchacha20poly1305_ietf_decrypt(pt.data(), &plen, nullptr, u8(ct), ct.size(),
                                                 ad.empty() ? nullptr : u8(ad), ad.size(),
                                                 u8(nonce), key.data()) != 0)
    return std::nullopt;
  pt.b.resize(static_cast<std::size_t>(plen));
  return pt;
}

// b64(nonce | ciphertext), the form of every body and wrapped key.
std::string seal_box(const Bytes &pt, const Bytes &ad, const Secret &key, E2eeRandom &r) {
  const Bytes nonce = random_bytes(r, crypto_aead_xchacha20poly1305_ietf_NPUBBYTES);
  return b64(nonce + aead_encrypt(pt, ad, nonce, key));
}

std::optional<Secret> open_box(const std::string &box_b64, const Bytes &ad, const Secret &key) {
  std::optional<std::string> raw = unb64_opt(box_b64);
  const std::size_t nlen = crypto_aead_xchacha20poly1305_ietf_NPUBBYTES;
  if (!raw || raw->size() < nlen)
    return std::nullopt;
  return aead_decrypt(raw->substr(nlen), ad, raw->substr(0, nlen), key);
}

// --- the model ------------------------------------------------------------------------------

struct SignedPreKey {
  std::int64_t key_id = 1;
  KeyPair kp;
  Bytes signature;
};

struct Identity {
  std::int64_t registration_id = 0;
  KeyPair sign;
  KeyPair dh;
  SignedPreKey spk;
  std::map<std::int64_t, KeyPair> opks; // unused one-time prekeys by id
};

struct Ratchet {
  KeyPair dhs;
  Bytes dhr; // empty: none yet
  Secret rk;
  std::optional<Secret> cks;
  std::optional<Secret> ckr;
  std::int64_t ns = 0, nr = 0, pn = 0;
  std::vector<std::pair<std::string, Secret>>
      skipped; // "<b64 dh>:<n>" -> message key, oldest first
};

struct Session {
  std::string role; // "initiator" | "responder"
  std::optional<std::string> envelope;
  bool peer_replied = false;
  Ratchet state;
};

struct Cached {
  std::string text;
  std::string attachments = "[]"; // JSON
};

struct Account {
  std::optional<Identity> identity;
  std::int64_t next_opk_id = 1;
  std::map<std::string, Session> sessions; // by conversation ("10", or "20:3" in a group)
  std::map<std::int64_t, Cached> plaintext;
};

constexpr std::int64_t kMaxCounter = 0x7FFFFFFF;
constexpr std::int64_t kMaxSkip = 1000; // e2ee.js's MAX_SKIP
constexpr char kX3dhInfo[] = "txwtf-e2ee-x3dh-v1";

// --- e2ee.js: identity and prekeys ------------------------------------------------------------

// generateIdentity + generateSignedPreKey(sign.priv, 1) + generateOneTimePreKeys(1, n).
Identity generate_identity(E2eeRandom &r, int opks) {
  Identity id;
  id.sign = sign_keypair(r);
  id.dh = box_keypair(r);
  id.registration_id = static_cast<std::int64_t>(r.uniform(16380)) + 1;
  id.spk.key_id = 1;
  id.spk.kp = box_keypair(r);
  id.spk.signature.assign(crypto_sign_BYTES, '\0');
  crypto_sign_detached(reinterpret_cast<unsigned char *>(id.spk.signature.data()), nullptr,
                       u8(id.spk.kp.pub), id.spk.kp.pub.size(), id.sign.priv.data());
  for (int i = 0; i < opks; ++i)
    id.opks.emplace(1 + i, box_keypair(r));
  return id;
}

bool verify_signed_prekey(const Bytes &spk_pub, const Bytes &sig, const Bytes &ik_sign_pub) {
  return sig.size() == crypto_sign_BYTES && ik_sign_pub.size() == crypto_sign_PUBLICKEYBYTES &&
         crypto_sign_verify_detached(u8(sig), u8(spk_pub), spk_pub.size(), u8(ik_sign_pub)) == 0;
}

E2eePublicKeys public_keys_of(const Identity &id) {
  E2eePublicKeys p;
  p.identity_key_ed25519 = b64(id.sign.pub);
  p.identity_key_x25519 = b64(id.dh.pub);
  p.registration_id = id.registration_id;
  p.signed_prekey_id = id.spk.key_id;
  p.signed_prekey = b64(id.spk.kp.pub);
  p.signed_prekey_signature = b64(id.spk.signature);
  for (const auto &[kid, kp] : id.opks)
    p.one_time_prekeys.push_back({kid, b64(kp.pub)});
  p.fingerprint = e2ee_fingerprint(p.identity_key_ed25519);
  return p;
}

// --- e2ee.js: X3DH ----------------------------------------------------------------------------

struct Initiated {
  Secret shared_key;
  std::string init_header; // JSON
  Bytes their_ratchet_pub;
};

// x3dhInitiate(myIdentity, bundle).
Initiated x3dh_initiate(const Identity &me, const E2eeBundle &bundle, E2eeRandom &r) {
  const Bytes ikb_sign = unb64n(bundle.identity_key_ed25519, 32, "the bundle's identity key");
  const Bytes ikb_dh = unb64n(bundle.identity_key_x25519, 32, "the bundle's X25519 identity key");
  const Bytes spkb = unb64n(bundle.signed_prekey, 32, "the bundle's signed prekey");
  const Bytes sig = unb64n(bundle.signed_prekey_signature, 64, "the bundle's prekey signature");
  if (!verify_signed_prekey(spkb, sig, ikb_sign))
    raise("X3DH: signed prekey signature verification failed");
  std::optional<Bytes> opkb;
  if (bundle.one_time_prekey)
    opkb = unb64n(bundle.one_time_prekey->public_key, 32, "the bundle's one-time prekey");

  const KeyPair ek = box_keypair(r);
  Secret parts;
  parts.b += dh(me.dh.priv, spkb).b;
  parts.b += dh(ek.priv, ikb_dh).b;
  parts.b += dh(ek.priv, spkb).b;
  if (opkb)
    parts.b += dh(ek.priv, *opkb).b;
  Initiated out;
  out.shared_key = generichash(32, parts.data(), parts.size(), u8(kX3dhInfo), sizeof kX3dhInfo - 1);
  out.init_header = jobj({
      {"identity_key_ed25519", jstr(b64(me.sign.pub))},
      {"identity_key_x25519", jstr(b64(me.dh.pub))},
      {"ephemeral_key", jstr(b64(ek.pub))},
      {"spk_id", jnum(bundle.signed_prekey_id)},
      {"opk_id", bundle.one_time_prekey ? jnum(bundle.one_time_prekey->key_id) : "null"},
  });
  out.their_ratchet_pub = spkb;
  return out;
}

// x3dhRespond(myIdentity, mySpk, myOpkPriv, initHeader).
Secret x3dh_respond(const Identity &me, const KeyPair *opk, const jl::Value &header) {
  const Bytes ika_dh = unb64n(str_member(header, "identity_key_x25519", "the envelope"), 32,
                              "the envelope's identity key");
  const Bytes eka =
      unb64n(str_member(header, "ephemeral_key", "the envelope"), 32, "the envelope's key");
  Secret parts;
  parts.b += dh(me.spk.kp.priv, ika_dh).b;
  parts.b += dh(me.dh.priv, eka).b;
  parts.b += dh(me.spk.kp.priv, eka).b;
  if (opt_int_member(header, "opk_id")) {
    if (!opk)
      raise("this message used a one-time prekey this device does not hold (it was made on "
            "another device, or already used)");
    parts.b += dh(opk->priv, eka).b;
  }
  return generichash(32, parts.data(), parts.size(), u8(kX3dhInfo), sizeof kX3dhInfo - 1);
}

// --- e2ee.js: the Double Ratchet --------------------------------------------------------------

// ratchetInitAlice(sharedKey, theirRatchetPub).
Ratchet ratchet_init_alice(const Secret &sk, const Bytes &their, E2eeRandom &r) {
  Ratchet st;
  st.dhs = box_keypair(r);
  auto [rk, cks] = kdf_rk(sk, dh(st.dhs.priv, their));
  st.dhr = their;
  st.rk = std::move(rk);
  st.cks = std::move(cks);
  return st;
}

// ratchetInitBob(sharedKey, myRatchetKeypair): his signed prekey pair.
Ratchet ratchet_init_bob(const Secret &sk, const KeyPair &spk) {
  Ratchet st;
  st.dhs = spk;
  st.rk = sk;
  return st;
}

struct Header {
  std::string dh; // base64, as it came
  std::int64_t pn = 0;
  std::int64_t n = 0;
};

// _headerBytes: JSON.stringify({dh, pn, n}).
std::string header_json(const Header &h) {
  return jobj({{"dh", jstr(h.dh)}, {"pn", jnum(h.pn)}, {"n", jnum(h.n)}});
}

Header parse_header(const jl::Value &v) {
  Header h;
  h.dh = str_member(v, "dh", "the ratchet header");
  h.pn = int_member(v, "pn", "the ratchet header");
  h.n = int_member(v, "n", "the ratchet header");
  if (h.pn < 0 || h.n < 0 || h.pn > kMaxCounter || h.n > kMaxCounter)
    raise("the ratchet header's counters are out of range");
  unb64n(h.dh, 32, "the ratchet header's key");
  return h;
}

Header parse_header_b64(const std::string &header_b64) {
  return parse_header(parse_json(unb64(header_b64, "the ratchet header"), "the ratchet header"));
}

struct Encrypted {
  Header header;
  std::string ciphertext; // b64(nonce | ct)
};

// ratchetEncrypt(state, plaintextBytes, associatedData).
Encrypted ratchet_encrypt(Ratchet &st, const Bytes &pt, const Bytes &ad, E2eeRandom &r) {
  if (!st.cks)
    raise("this session cannot send before it has received");
  auto [cks, mk] = kdf_ck(*st.cks);
  st.cks = std::move(cks);
  Encrypted out;
  out.header = {b64(st.dhs.pub), st.pn, st.ns};
  st.ns += 1;
  out.ciphertext = seal_box(pt, ad + header_json(out.header), mk, r);
  return out;
}

void remember_skipped(Ratchet &st, std::string id, Secret mk, std::size_t max_skipped) {
  st.skipped.emplace_back(std::move(id), std::move(mk));
  if (st.skipped.size() > max_skipped)
    st.skipped.erase(st.skipped.begin(), st.skipped.begin() + static_cast<std::ptrdiff_t>(
                                                                  st.skipped.size() - max_skipped));
}

// _skipMessageKeys(state, until).
void skip_message_keys(Ratchet &st, std::int64_t until, std::size_t max_skipped) {
  if (!st.ckr)
    return;
  if (until - st.nr > kMaxSkip)
    raise("ratchet: too many skipped messages");
  const std::string dhr = b64(st.dhr);
  while (st.nr < until) {
    auto [ckr, mk] = kdf_ck(*st.ckr);
    st.ckr = std::move(ckr);
    remember_skipped(st, dhr + ":" + std::to_string(st.nr), std::move(mk), max_skipped);
    st.nr += 1;
  }
}

// _dhRatchet(state, header).
void dh_ratchet(Ratchet &st, const Header &h, E2eeRandom &r) {
  st.pn = st.ns;
  st.ns = 0;
  st.nr = 0;
  st.dhr = unb64n(h.dh, 32, "the ratchet header's key");
  auto [rk, ckr] = kdf_rk(st.rk, dh(st.dhs.priv, st.dhr));
  st.rk = std::move(rk);
  st.ckr = std::move(ckr);
  st.dhs = box_keypair(r);
  auto [rk2, cks] = kdf_rk(st.rk, dh(st.dhs.priv, st.dhr));
  st.rk = std::move(rk2);
  st.cks = std::move(cks);
}

// ratchetDecrypt(state, header, ciphertextB64, associatedData), on `st`, which the caller keeps
// only when this returns a value.
std::optional<Secret> ratchet_decrypt(Ratchet &st, const Header &h, const std::string &ct,
                                      const Bytes &ad, E2eeRandom &r, std::size_t max_skipped) {
  const Bytes full_ad = ad + header_json(h);
  const std::string skip_id = h.dh + ":" + std::to_string(h.n);
  for (auto it = st.skipped.begin(); it != st.skipped.end(); ++it) {
    if (it->first == skip_id) {
      std::optional<Secret> pt = open_box(ct, full_ad, it->second);
      if (pt)
        st.skipped.erase(it);
      return pt;
    }
  }
  const bool is_new = st.dhr.empty() || b64(st.dhr) != h.dh;
  if (is_new) {
    skip_message_keys(st, h.pn, max_skipped);
    dh_ratchet(st, h, r);
  }
  skip_message_keys(st, h.n, max_skipped);
  if (!st.ckr)
    raise("the session has no receiving chain");
  auto [ckr, mk] = kdf_ck(*st.ckr);
  st.ckr = std::move(ckr);
  st.nr += 1;
  return open_box(ct, full_ad, mk);
}

// --- messaging-e2ee.js: the payload -----------------------------------------------------------

// _encodePayload(string): {v: 1, text, attachments: []}.
std::string encode_payload(const std::string &text) {
  return jobj({{"v", "1"}, {"text", jstr(text)}, {"attachments", "[]"}});
}

// _decodePayload(plaintext).
Cached decode_payload(const std::string &plaintext) {
  Cached out;
  std::optional<jl::Value> env = jl::parse(plaintext);
  if (env && env->is_object() && env->get("text")) {
    const jl::Value &t = *env->get("text");
    if (t.is_string())
      out.text = t.str;
    else if (!(t.kind == jl::Value::Kind::null || (t.kind == jl::Value::Kind::boolean && !t.b) ||
               (t.kind == jl::Value::Kind::number && t.num == 0)))
      out.text = render(t);
    const jl::Value *a = env->get("attachments");
    if (a && a->kind == jl::Value::Kind::array)
      out.attachments = render(*a);
    return out;
  }
  out.text = well_formed_utf8(plaintext);
  return out;
}

// --- serialisation: messaging-e2ee.js's store layout ------------------------------------------

std::string key_pair_json(const KeyPair &kp) {
  return jobj({{"pub", jstr(b64(kp.pub))}, {"priv", jstr(b64(kp.priv))}});
}

// _serializeIdentity(mem).
std::string identity_json(const Identity &id) {
  Members opks;
  for (const auto &[kid, kp] : id.opks)
    opks.emplace_back(std::to_string(kid), key_pair_json(kp));
  return jobj({
      {"v", "1"},
      {"registration_id", jnum(id.registration_id)},
      {"sign_pub", jstr(b64(id.sign.pub))},
      {"sign_priv", jstr(b64(id.sign.priv))},
      {"dh_pub", jstr(b64(id.dh.pub))},
      {"dh_priv", jstr(b64(id.dh.priv))},
      {"spk", jobj({{"key_id", jnum(id.spk.key_id)},
                    {"pub", jstr(b64(id.spk.kp.pub))},
                    {"priv", jstr(b64(id.spk.kp.priv))},
                    {"signature", jstr(b64(id.spk.signature))}})},
      {"opks", jobj(opks)},
  });
}

KeyPair parse_key_pair(const jl::Value &v, std::size_t pub_len, std::size_t priv_len,
                       const char *what) {
  KeyPair kp;
  kp.pub = unb64n(str_member(v, "pub", what), pub_len, what);
  kp.priv = unb64_secret(str_member(v, "priv", what), priv_len, what);
  return kp;
}

// _deserializeIdentity(raw).
Identity parse_identity(const jl::Value &v) {
  Identity id;
  id.registration_id = int_member(v, "registration_id", "the identity");
  id.sign.pub = unb64n(str_member(v, "sign_pub", "the identity"), 32, "the identity's sign_pub");
  id.sign.priv =
      unb64_secret(str_member(v, "sign_priv", "the identity"), 64, "the identity's sign_priv");
  id.dh.pub = unb64n(str_member(v, "dh_pub", "the identity"), 32, "the identity's dh_pub");
  id.dh.priv = unb64_secret(str_member(v, "dh_priv", "the identity"), 32, "the identity's dh_priv");
  const jl::Value &spk = member(v, "spk", "the identity");
  id.spk.key_id = int_member(spk, "key_id", "the signed prekey");
  id.spk.kp = parse_key_pair(spk, 32, 32, "the signed prekey");
  id.spk.signature =
      unb64n(str_member(spk, "signature", "the signed prekey"), 64, "the prekey signature");
  if (const jl::Value *opks = v.get("opks"); opks && opks->is_object()) {
    for (const auto &kv : opks->members) {
      char *end = nullptr;
      const long long kid = std::strtoll(kv.first.c_str(), &end, 10);
      if (kv.first.empty() || *end != '\0')
        raise("a one-time prekey id is not a number");
      id.opks[kid] = parse_key_pair(kv.second, 32, 32, "a one-time prekey");
    }
  }
  return id;
}

// _serializeState(st).
std::string ratchet_json(const Ratchet &st) {
  Members skipped;
  for (const auto &[k, mk] : st.skipped)
    skipped.emplace_back(k, jstr(b64(mk)));
  return jobj({
      {"DHs", key_pair_json(st.dhs)},
      {"DHr", st.dhr.empty() ? "null" : jstr(b64(st.dhr))},
      {"RK", jstr(b64(st.rk))},
      {"CKs", st.cks ? jstr(b64(*st.cks)) : "null"},
      {"CKr", st.ckr ? jstr(b64(*st.ckr)) : "null"},
      {"Ns", jnum(st.ns)},
      {"Nr", jnum(st.nr)},
      {"PN", jnum(st.pn)},
      {"skipped", jobj(skipped)},
  });
}

Ratchet parse_ratchet(const jl::Value &v) {
  Ratchet st;
  st.dhs = parse_key_pair(member(v, "DHs", "the ratchet"), 32, 32, "the ratchet's DHs");
  if (std::optional<std::string> dhr = opt_str_member(v, "DHr"))
    st.dhr = unb64n(*dhr, 32, "the ratchet's DHr");
  st.rk = unb64_secret(str_member(v, "RK", "the ratchet"), 32, "the ratchet's RK");
  if (std::optional<std::string> cks = opt_str_member(v, "CKs"))
    st.cks = unb64_secret(*cks, 32, "the ratchet's CKs");
  if (std::optional<std::string> ckr = opt_str_member(v, "CKr"))
    st.ckr = unb64_secret(*ckr, 32, "the ratchet's CKr");
  st.ns = int_member(v, "Ns", "the ratchet");
  st.nr = int_member(v, "Nr", "the ratchet");
  st.pn = int_member(v, "PN", "the ratchet");
  if (const jl::Value *sk = v.get("skipped"); sk && sk->is_object())
    for (const auto &kv : sk->members) {
      if (!kv.second.is_string())
        raise("a skipped key is not a string");
      st.skipped.emplace_back(kv.first, unb64_secret(kv.second.str, 32, "a skipped key"));
    }
  return st;
}

// _serializeSession(sess).
std::string session_json(const Session &s) {
  return jobj({{"v", "1"},
               {"role", jstr(s.role)},
               {"envelope", jopt(s.envelope)},
               {"peerReplied", jbool(s.peer_replied)},
               {"state", ratchet_json(s.state)}});
}

Session parse_session(const jl::Value &v) {
  Session s;
  s.role = str_member(v, "role", "the session");
  if (s.role != "initiator" && s.role != "responder")
    raise("the session's role is neither initiator nor responder");
  s.envelope = opt_str_member(v, "envelope");
  s.peer_replied = bool_member(v, "peerReplied");
  s.state = parse_ratchet(member(v, "state", "the session"));
  return s;
}

std::string cached_json(const Cached &c) {
  return jobj({{"text", jstr(c.text)}, {"attachments", c.attachments}});
}

// The account as messaging-e2ee.js's store holds it: key -> value.
std::string account_state_json(const Account &a) {
  Members m;
  if (a.identity)
    m.emplace_back("identity", identity_json(*a.identity));
  for (const auto &[ref, s] : a.sessions)
    m.emplace_back("session:" + ref, session_json(s));
  for (const auto &[id, c] : a.plaintext)
    m.emplace_back("pt:" + std::to_string(id), cached_json(c));
  return jobj(m);
}

Account parse_account_state(const jl::Value &v) {
  if (!v.is_object())
    raise("the state is not an object");
  Account a;
  std::int64_t max_opk = 0;
  for (const auto &kv : v.members) {
    const std::string &k = kv.first;
    if (k == "identity") {
      a.identity = parse_identity(kv.second);
      if (!a.identity->opks.empty())
        max_opk = std::max(max_opk, a.identity->opks.rbegin()->first);
    } else if (k.rfind("session:", 0) == 0) {
      a.sessions[k.substr(8)] = parse_session(kv.second);
    } else if (k.rfind("pt:", 0) == 0) {
      char *end = nullptr;
      const long long id = std::strtoll(k.c_str() + 3, &end, 10);
      if (*end != '\0')
        raise("a cached message id is not a number");
      Cached c;
      if (const jl::Value *t = kv.second.get("text"); t && t->is_string())
        c.text = t->str;
      if (const jl::Value *at = kv.second.get("attachments");
          at && at->kind == jl::Value::Kind::array)
        c.attachments = render(*at);
      a.plaintext[id] = std::move(c);
    }
  }
  a.next_opk_id = max_opk + 1;
  return a;
}

// e2ee.js's exportSecretBundle(identity, spk).
std::string secret_bundle_json(const Identity &id) {
  return jobj({
      {"v", "1"},
      {"registration_id", jnum(id.registration_id)},
      {"sign_priv", jstr(b64(id.sign.priv))},
      {"sign_pub", jstr(b64(id.sign.pub))},
      {"dh_priv", jstr(b64(id.dh.priv))},
      {"dh_pub", jstr(b64(id.dh.pub))},
      {"spk", jobj({{"key_id", jnum(id.spk.key_id)},
                    {"priv", jstr(b64(id.spk.kp.priv))},
                    {"pub", jstr(b64(id.spk.kp.pub))},
                    {"signature", jstr(b64(id.spk.signature))}})},
  });
}

// importSecretBundle, checked: every public half matches its secret, and the prekey's signature.
Identity parse_secret_bundle(const jl::Value &v) {
  Identity id;
  id.registration_id = int_member(v, "registration_id", "the backup");
  id.sign.pub = unb64n(str_member(v, "sign_pub", "the backup"), 32, "the backup's sign_pub");
  id.sign.priv =
      unb64_secret(str_member(v, "sign_priv", "the backup"), 64, "the backup's sign_priv");
  id.dh.pub = unb64n(str_member(v, "dh_pub", "the backup"), 32, "the backup's dh_pub");
  id.dh.priv = unb64_secret(str_member(v, "dh_priv", "the backup"), 32, "the backup's dh_priv");
  const jl::Value *spk = v.get("spk");
  if (!spk || !spk->is_object())
    raise("the backup has no signed prekey");
  id.spk.key_id = int_member(*spk, "key_id", "the backup's signed prekey");
  id.spk.kp = parse_key_pair(*spk, 32, 32, "the backup's signed prekey");
  id.spk.signature = unb64n(str_member(*spk, "signature", "the backup's signed prekey"), 64,
                            "the backup's prekey signature");
  unsigned char pk[32];
  crypto_sign_ed25519_sk_to_pk(pk, id.sign.priv.data());
  if (std::memcmp(pk, id.sign.pub.data(), 32) != 0)
    raise("the backup's signing key does not match its public key");
  crypto_scalarmult_base(pk, id.dh.priv.data());
  if (std::memcmp(pk, id.dh.pub.data(), 32) != 0)
    raise("the backup's X25519 key does not match its public key");
  crypto_scalarmult_base(pk, id.spk.kp.priv.data());
  if (std::memcmp(pk, id.spk.kp.pub.data(), 32) != 0)
    raise("the backup's signed prekey does not match its public key");
  if (!verify_signed_prekey(id.spk.kp.pub, id.spk.signature, id.sign.pub))
    raise("the backup's signed prekey signature does not verify");
  return id;
}

// The whole store file.
constexpr char kStoreWhat[] = "the E2EE store";

std::string store_json(const std::map<std::string, Account> &accounts) {
  Members handles;
  for (const auto &[h, a] : accounts)
    handles.emplace_back(
        h, jobj({{"next_opk_id", jnum(a.next_opk_id)}, {"state", account_state_json(a)}}));
  return jobj({{"v", "1"},
               {"about", jstr("cvc E2EE store: secrets, owner-only; written by the host")},
               {"handles", jobj(handles)}}) +
         "\n";
}

// (Unused in a browser, where no store file opens yet.)
[[maybe_unused]] std::map<std::string, Account> parse_store(const std::string &text) {
  const jl::Value doc = parse_json(text, "the E2EE store");
  if (int_member(doc, "v", "the E2EE store") != 1)
    raise("the E2EE store is not version 1");
  std::map<std::string, Account> out;
  const jl::Value &hs = member(doc, "handles", "the E2EE store");
  if (!hs.is_object())
    raise("the E2EE store's handles are not an object");
  for (const auto &kv : hs.members) {
    if (!E2eeStore::valid_handle(kv.first))
      raise("the E2EE store names a malformed handle");
    Account a = parse_account_state(member(kv.second, "state", "a handle"));
    if (std::optional<std::int64_t> next = opt_int_member(kv.second, "next_opk_id"))
      a.next_opk_id = std::max(a.next_opk_id, *next);
    out[kv.first] = std::move(a);
  }
  return out;
}

} // namespace

#endif // CVC_HAVE_SODIUM

// --- the store --------------------------------------------------------------------------------

struct E2eeStore::Impl {
#ifdef CVC_HAVE_SODIUM
  std::map<std::string, Account> accounts;
#endif
  bool opened = false;
#if defined(_WIN32)
  HANDLE lock = INVALID_HANDLE_VALUE;
#elif !defined(__EMSCRIPTEN__)
  int lock = -1;
#endif

  void unlock() {
#if defined(_WIN32)
    if (lock != INVALID_HANDLE_VALUE)
      ::CloseHandle(lock);
    lock = INVALID_HANDLE_VALUE;
#elif !defined(__EMSCRIPTEN__)
    if (lock >= 0)
      ::close(lock);
    lock = -1;
#endif
  }
};

bool e2ee_available() {
#ifdef CVC_HAVE_SODIUM
  return sodium_init() >= 0;
#else
  return false;
#endif
}

std::shared_ptr<E2eeRandom> e2ee_system_random() {
#ifdef CVC_HAVE_SODIUM
  return std::make_shared<SystemRandom>();
#else
  return nullptr;
#endif
}

std::shared_ptr<E2eeRandom> e2ee_seeded_random(const std::string &seed) {
#ifdef CVC_HAVE_SODIUM
  ensure_sodium();
  return std::make_shared<SeededRandom>(seed);
#else
  (void)seed;
  return nullptr;
#endif
}

std::string e2ee_fingerprint(const std::string &key_b64) {
#ifdef CVC_HAVE_SODIUM
  if (sodium_init() < 0)
    return {};
  std::optional<std::string> key = unb64_opt(key_b64);
  if (!key || key->empty())
    return {};
  unsigned char h[20];
  crypto_generichash(h, sizeof h, u8(*key), key->size(), nullptr, 0);
  char hex[41];
  sodium_bin2hex(hex, sizeof hex, h, sizeof h);
  std::string out;
  for (int i = 0; i < 40; ++i) {
    if (i && i % 5 == 0)
      out.push_back(' ');
    out.push_back(hex[i]);
  }
  return out;
#else
  (void)key_b64;
  return {};
#endif
}

E2eeStore::E2eeStore(Options options)
    : options_(std::move(options)), impl_(std::make_unique<Impl>()) {
#ifdef CVC_HAVE_SODIUM
  if (!options_.random)
    options_.random = e2ee_system_random();
  if (options_.max_skipped == 0)
    options_.max_skipped = 1;
#endif
}

E2eeStore::~E2eeStore() { impl_->unlock(); }

bool E2eeStore::valid_handle(const std::string &handle) {
  return !handle.empty() && handle.size() <= 64 &&
         std::all_of(handle.begin(), handle.end(), handle_char);
}

bool E2eeStore::valid_conversation(const std::string &conv) {
  return !conv.empty() && conv.size() <= 64 && std::all_of(conv.begin(), conv.end(), [](char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' ||
           c == '-';
  });
}

std::string E2eeStore::resolve_path(const std::string &spec) {
#if defined(__EMSCRIPTEN__)
  (void)spec;
  return std::string();
#else
  if (!valid_handle(spec) || spec == "." || spec == "..")
    return spec;
  const fs::path dir = private_file::config_dir();
  if (dir.empty())
    return std::string();
  return (dir / "e2ee" / spec).string();
#endif
}

#ifdef CVC_HAVE_SODIUM

namespace {

void check_handle(const std::string &h) {
  if (!E2eeStore::valid_handle(h))
    raise("'" + h + "' is not an E2EE handle (1-64 of A-Z a-z 0-9 . _ -)");
}

void check_conversation(const std::string &c) {
  if (!E2eeStore::valid_conversation(c))
    raise("'" + c + "' is not a conversation id (1-64 of A-Z a-z 0-9 _ -)");
}

std::string open_failure() { return "the message does not open with this session"; }

} // namespace

// Run `fn` on a copy of `handle`'s account; keep the copy, and write the file, only when it
// returns true. `fn` throws E2eeError to stop.
#define CVC_E2EE_LOCKED std::lock_guard<std::mutex> guard(mutex_)

namespace {
template <class Fn>
bool mutate(std::map<std::string, Account> &accounts, const std::string &handle,
            const std::string &path, std::string *error, Fn &&fn) {
  auto it = accounts.find(handle);
  Account next = it == accounts.end() ? Account{} : it->second;
  if (!fn(next))
    return false;
  if (!path.empty()) {
    std::map<std::string, Account> all = accounts;
    all[handle] = next;
    if (!private_file::write(fs::path(path), store_json(all), kStoreWhat, error))
      return false;
  }
  accounts[handle] = std::move(next);
  return true;
}
} // namespace

#endif

bool E2eeStore::open(std::string *error) {
#ifndef CVC_HAVE_SODIUM
  return fail(error, "this build has no E2EE (libsodium was not found)");
#else
  CVC_E2EE_LOCKED;
  if (impl_->opened)
    return true;
  if (sodium_init() < 0)
    return fail(error, "libsodium could not start");
  if (options_.path.empty()) {
    impl_->opened = true;
    return true;
  }
#if defined(__EMSCRIPTEN__)
  return fail(error, "a browser host keeps no E2EE store file yet");
#else
  const fs::path path(options_.path);
  std::error_code ec;
  if (!path.parent_path().empty() && !fs::exists(path.parent_path(), ec)) {
    fs::create_directories(path.parent_path(), ec);
#if !defined(_WIN32)
    ::chmod(path.parent_path().c_str(), 0700);
#endif
  }
  fs::path lock_path = path;
  lock_path += ".lock";
#if defined(_WIN32)
  impl_->lock = ::CreateFileW(lock_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                              OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (impl_->lock == INVALID_HANDLE_VALUE)
    return fail(error,
                std::string(kStoreWhat) + " '" + options_.path + "' is in use by another process");
#else
  impl_->lock = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
  if (impl_->lock < 0)
    return fail(error, std::string("cannot lock ") + kStoreWhat + " '" + options_.path + "'");
  if (::flock(impl_->lock, LOCK_EX | LOCK_NB) != 0) {
    impl_->unlock();
    return fail(error,
                std::string(kStoreWhat) + " '" + options_.path + "' is in use by another process");
  }
#endif
  if (fs::exists(path, ec)) {
    if (!private_file::check_owner_only(path, kStoreWhat, error)) {
      impl_->unlock();
      return false;
    }
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    if (!in && !in.eof()) {
      impl_->unlock();
      return fail(error, std::string("cannot read ") + kStoreWhat + " '" + options_.path + "'");
    }
    try {
      impl_->accounts = parse_store(text.str());
    } catch (const E2eeError &e) {
      impl_->unlock();
      return fail(error, std::string(e.what()) + " ('" + options_.path + "')");
    }
  }
  impl_->opened = true;
  return true;
#endif
#endif
}

#ifdef CVC_HAVE_SODIUM
#define CVC_E2EE_OPENED(err)                                                                       \
  if (!impl_->opened)                                                                              \
    return fail(err, "the E2EE store is not open");
#endif

std::vector<std::string> E2eeStore::handles() const {
  std::vector<std::string> out;
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  for (const auto &[h, a] : impl_->accounts)
    if (a.identity)
      out.push_back(h);
#endif
  return out;
}

bool E2eeStore::has_identity(const std::string &handle) const {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  auto it = impl_->accounts.find(handle);
  return it != impl_->accounts.end() && it->second.identity.has_value();
#else
  (void)handle;
  return false;
#endif
}

std::optional<E2eePublicKeys> E2eeStore::public_keys(const std::string &handle) const {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  auto it = impl_->accounts.find(handle);
  if (it == impl_->accounts.end() || !it->second.identity)
    return std::nullopt;
  return public_keys_of(*it->second.identity);
#else
  (void)handle;
  return std::nullopt;
#endif
}

std::optional<E2eePublicKeys> E2eeStore::create_identity(const std::string &handle,
                                                         int one_time_prekeys, std::string *error) {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  if (!impl_->opened) {
    fail(error, "the E2EE store is not open");
    return std::nullopt;
  }
  std::optional<E2eePublicKeys> out;
  try {
    check_handle(handle);
    if (one_time_prekeys < 0 || one_time_prekeys > 1000)
      raise("one-time prekeys: 0 to 1000");
    if (!mutate(impl_->accounts, handle, options_.path, error, [&](Account &a) {
          if (a.identity)
            raise("'" + handle + "' has an identity already (forget it first)");
          a = Account{};
          a.identity = generate_identity(*options_.random, one_time_prekeys);
          a.next_opk_id = one_time_prekeys + 1;
          out = public_keys_of(*a.identity);
          return true;
        }))
      return std::nullopt;
  } catch (const E2eeError &e) {
    fail(error, e.what());
    return std::nullopt;
  }
  return out;
#else
  (void)handle;
  (void)one_time_prekeys;
  fail(error, "this build has no E2EE (libsodium was not found)");
  return std::nullopt;
#endif
}

std::optional<std::vector<E2eePreKey>> E2eeStore::add_prekeys(const std::string &handle, int count,
                                                              std::string *error) {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  std::optional<std::vector<E2eePreKey>> out;
  try {
    check_handle(handle);
    if (!impl_->opened)
      raise("the E2EE store is not open");
    if (count < 1 || count > 1000)
      raise("one-time prekeys: 1 to 1000");
    if (!mutate(impl_->accounts, handle, options_.path, error, [&](Account &a) {
          if (!a.identity)
            raise("'" + handle + "' has no identity");
          std::vector<E2eePreKey> made;
          for (int i = 0; i < count; ++i) {
            const std::int64_t kid = a.next_opk_id++;
            KeyPair kp = box_keypair(*options_.random);
            made.push_back({kid, b64(kp.pub)});
            a.identity->opks[kid] = std::move(kp);
          }
          out = std::move(made);
          return true;
        }))
      return std::nullopt;
  } catch (const E2eeError &e) {
    fail(error, e.what());
    return std::nullopt;
  }
  return out;
#else
  (void)handle;
  (void)count;
  fail(error, "this build has no E2EE (libsodium was not found)");
  return std::nullopt;
#endif
}

bool E2eeStore::forget(const std::string &handle, std::string *error) {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  CVC_E2EE_OPENED(error);
  auto it = impl_->accounts.find(handle);
  if (it == impl_->accounts.end())
    return false;
  std::map<std::string, Account> all = impl_->accounts;
  all.erase(handle);
  if (!options_.path.empty() &&
      !private_file::write(fs::path(options_.path), store_json(all), kStoreWhat, error))
    return false;
  impl_->accounts = std::move(all);
  return true;
#else
  (void)handle;
  return fail(error, "this build has no E2EE (libsodium was not found)");
#endif
}

bool E2eeStore::has_session(const std::string &handle, const std::string &conv) const {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  auto it = impl_->accounts.find(handle);
  return it != impl_->accounts.end() && it->second.sessions.count(conv) != 0;
#else
  (void)handle;
  (void)conv;
  return false;
#endif
}

E2eeSealed E2eeStore::seal(const std::string &handle, const std::string &conv,
                           const std::string &text, const std::optional<E2eeBundle> &bundle) {
  E2eeSealed out;
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  try {
    check_handle(handle);
    check_conversation(conv);
    if (!impl_->opened)
      raise("the E2EE store is not open");
    std::string werr;
    const bool kept = mutate(impl_->accounts, handle, options_.path, &werr, [&](Account &a) {
      if (!a.identity)
        raise("'" + handle + "' has no identity");
      E2eeRandom &r = *options_.random;
      auto it = a.sessions.find(conv);
      if (it == a.sessions.end()) {
        if (!bundle) {
          out.need_bundle = true;
          return false;
        }
        // encryptAndSend: x3dhInitiate, ratchetInitAlice, the envelope.
        Initiated init = x3dh_initiate(*a.identity, *bundle, r);
        Session s;
        s.role = "initiator";
        s.state = ratchet_init_alice(init.shared_key, init.their_ratchet_pub, r);
        s.envelope = b64(init.init_header);
        it = a.sessions.emplace(conv, std::move(s)).first;
      }
      Session &s = it->second;
      // sealMessage(state, plaintextStr).
      const Secret ck = random_secret(r, 32);
      const std::string body = seal_box(encode_payload(text), Bytes(), ck, r);
      Encrypted wrap = ratchet_encrypt(s.state, ck.b, body, r);
      out.ciphertext = body;
      out.wrapped_key = wrap.ciphertext;
      out.header = b64(header_json(wrap.header));
      if (s.role == "initiator" && !s.peer_replied)
        out.envelope = s.envelope;
      out.algorithm = "dr-v1";
      return true;
    });
    if (!kept) {
      if (!out.need_bundle)
        out.error = werr.empty() ? "could not seal" : werr;
      out.ciphertext.clear();
      out.wrapped_key.clear();
      out.header.clear();
      out.envelope.reset();
      return out;
    }
    out.ok = true;
  } catch (const E2eeError &e) {
    out = E2eeSealed{};
    out.error = e.what();
  }
#else
  (void)handle;
  (void)conv;
  (void)text;
  (void)bundle;
  out.error = "this build has no E2EE (libsodium was not found)";
#endif
  return out;
}

E2eeGroupSealed E2eeStore::seal_group(const std::string &handle, const std::string &conv,
                                      const std::string &text,
                                      const std::vector<E2eeGroupMember> &members) {
  E2eeGroupSealed out;
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  try {
    check_handle(handle);
    check_conversation(conv);
    if (!impl_->opened)
      raise("the E2EE store is not open");
    if (members.empty())
      raise("a group message needs at least one recipient");
    std::string werr;
    const bool kept = mutate(impl_->accounts, handle, options_.path, &werr, [&](Account &a) {
      if (!a.identity)
        raise("'" + handle + "' has no identity");
      for (const E2eeGroupMember &m : members)
        if (!a.sessions.count(conv + ":" + std::to_string(m.user_id)) && !m.bundle)
          out.need_bundles.push_back(m.user_id);
      if (!out.need_bundles.empty())
        return false;
      E2eeRandom &r = *options_.random;
      // encryptGroupBody(plaintextStr).
      const Secret ck = random_secret(r, 32);
      const std::string body = seal_box(encode_payload(text), Bytes(), ck, r);
      for (const E2eeGroupMember &m : members) {
        const std::string ref = conv + ":" + std::to_string(m.user_id);
        auto it = a.sessions.find(ref);
        if (it == a.sessions.end()) {
          Initiated init = x3dh_initiate(*a.identity, *m.bundle, r);
          Session s;
          s.role = "initiator";
          s.state = ratchet_init_alice(init.shared_key, init.their_ratchet_pub, r);
          s.envelope = b64(init.init_header);
          it = a.sessions.emplace(ref, std::move(s)).first;
        }
        Session &s = it->second;
        // wrapGroupKey(state, ckB64, body).
        Encrypted wrap = ratchet_encrypt(s.state, ck.b, body, r);
        const std::string group_header =
            jobj({{"h", jstr(b64(header_json(wrap.header)))},
                  {"x3dh", s.role == "initiator" && !s.peer_replied ? jopt(s.envelope) : "null"}});
        out.recipient_keys.push_back({m.user_id, wrap.ciphertext, b64(group_header)});
      }
      out.ciphertext = body;
      out.algorithm = "grp-dr-v1";
      return true;
    });
    if (!kept) {
      if (out.need_bundles.empty())
        out.error = werr.empty() ? "could not seal" : werr;
      out.recipient_keys.clear();
      out.ciphertext.clear();
      return out;
    }
    out.ok = true;
  } catch (const E2eeError &e) {
    out = E2eeGroupSealed{};
    out.error = e.what();
  }
#else
  (void)handle;
  (void)conv;
  (void)text;
  (void)members;
  out.error = "this build has no E2EE (libsodium was not found)";
#endif
  return out;
}

bool E2eeStore::remember(const std::string &handle, std::int64_t message_id,
                         const std::string &text, std::string *error) {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  try {
    check_handle(handle);
    if (!impl_->opened)
      raise("the E2EE store is not open");
    return mutate(impl_->accounts, handle, options_.path, error, [&](Account &a) {
      if (!a.identity)
        raise("'" + handle + "' has no identity");
      a.plaintext[message_id] = Cached{well_formed_utf8(text), "[]"};
      while (a.plaintext.size() > options_.max_cached)
        a.plaintext.erase(a.plaintext.begin());
      return true;
    });
  } catch (const E2eeError &e) {
    return fail(error, e.what());
  }
#else
  (void)handle;
  (void)message_id;
  (void)text;
  return fail(error, "this build has no E2EE (libsodium was not found)");
#endif
}

E2eeOpened E2eeStore::open_message(const std::string &handle, const std::string &conv,
                                   const E2eeMessage &message) {
  E2eeOpened out;
#ifdef CVC_HAVE_SODIUM
  using S = E2eeOpened::Status;
  CVC_E2EE_LOCKED;
  try {
    check_handle(handle);
    check_conversation(conv);
    if (!impl_->opened)
      raise("the E2EE store is not open");
    auto ait = impl_->accounts.find(handle);
    if (ait == impl_->accounts.end() || !ait->second.identity)
      raise("'" + handle + "' has no identity");
    const Account &cur = ait->second;
    // A message opened before, or our own cached send: never through the ratchet again.
    if (message.id != 0) {
      auto c = cur.plaintext.find(message.id);
      if (c != cur.plaintext.end()) {
        out.status = S::cached;
        out.text = c->second.text;
        out.attachments = c->second.attachments;
        return out;
      }
    }
    if (message.is_sender) {
      out.status = S::own;
      return out;
    }
    const bool group = message.algorithm == "grp-dr-v1";
    if (!group && message.algorithm != "dr-v1")
      raise("the message's algorithm is '" + message.algorithm + "', not dr-v1 or grp-dr-v1");
    if (message.locked || !message.ciphertext || !message.wrapped_key ||
        (group && !message.header)) {
      out.status = S::locked;
      return out;
    }
    std::string ref = conv;
    std::string inner_header_b64;
    std::optional<std::string> envelope;
    if (group) {
      // decryptIncomingGroup: the header is {h, x3dh}, the session the pair's.
      ref = conv + ":" + std::to_string(message.sender_id);
      const jl::Value gh =
          parse_json(unb64(*message.header, "the group header"), "the group header");
      inner_header_b64 = str_member(gh, "h", "the group header");
      envelope = opt_str_member(gh, "x3dh");
    } else {
      if (!message.header)
        raise("the message has no ratchet header");
      inner_header_b64 = *message.header;
      envelope = message.envelope;
    }
    const Header header = parse_header_b64(inner_header_b64);

    std::string werr;
    std::optional<Cached> opened;
    bool no_session = false;
    std::string why;
    const bool kept = mutate(impl_->accounts, handle, options_.path, &werr, [&](Account &a) {
      auto sit = a.sessions.find(ref);
      Session fresh;
      Session *s = nullptr;
      std::optional<std::int64_t> used_opk;
      if (sit != a.sessions.end()) {
        s = &sit->second;
      } else {
        if (!envelope) {
          no_session = true;
          return false;
        }
        // Bootstrap as the responder (Bob) from the X3DH envelope.
        const jl::Value init =
            parse_json(unb64(*envelope, "the X3DH envelope"), "the X3DH envelope");
        const KeyPair *opk = nullptr;
        if (std::optional<std::int64_t> id = opt_int_member(init, "opk_id")) {
          auto o = a.identity->opks.find(*id);
          if (o != a.identity->opks.end()) {
            opk = &o->second;
            used_opk = *id;
          }
        }
        const Secret sk = x3dh_respond(*a.identity, opk, init);
        fresh.role = "responder";
        fresh.state = ratchet_init_bob(sk, a.identity->spk.kp);
        s = &fresh;
      }
      // openMessage / unwrapGroupKey + decryptGroupBody, on a copy of the ratchet.
      Ratchet st = s->state;
      std::optional<Secret> ck =
          ratchet_decrypt(st, header, *message.wrapped_key, *message.ciphertext, *options_.random,
                          options_.max_skipped);
      if (!ck || ck->size() != 32) {
        why = open_failure();
        return false;
      }
      std::optional<Secret> pt = open_box(*message.ciphertext, Bytes(), *ck);
      if (!pt) {
        why = "the message's body does not open with its key";
        return false;
      }
      s->state = std::move(st);
      if (s->role == "initiator")
        s->peer_replied = true;
      if (s == &fresh)
        a.sessions.emplace(ref, std::move(fresh));
      if (used_opk)
        a.identity->opks.erase(*used_opk);              // a one-time prekey is used once
      opened = decode_payload(well_formed_utf8(pt->b)); // as TextDecoder reads it
      if (message.id != 0) {
        a.plaintext[message.id] = *opened;
        while (a.plaintext.size() > options_.max_cached)
          a.plaintext.erase(a.plaintext.begin());
      }
      return true;
    });
    if (!kept) {
      if (no_session) {
        out.status = S::no_session;
        out.error = "no session for this conversation here, and the message carries no X3DH "
                    "envelope (its keys are on another device)";
      } else {
        out.status = S::failed;
        out.error = !why.empty() ? why : (werr.empty() ? open_failure() : werr);
      }
      return out;
    }
    out.status = S::ok;
    out.text = opened->text;
    out.attachments = opened->attachments;
  } catch (const E2eeError &e) {
    out = E2eeOpened{};
    out.status = E2eeOpened::Status::failed;
    out.error = e.what();
  }
#else
  (void)handle;
  (void)conv;
  (void)message;
  out.error = "this build has no E2EE (libsodium was not found)";
#endif
  return out;
}

std::optional<E2eeBackup> E2eeStore::backup(const std::string &handle,
                                            const std::string &passphrase, std::string *error) {
#ifdef CVC_HAVE_SODIUM
  try {
    check_handle(handle);
    std::string bundle;
    Bytes salt, nonce;
    {
      CVC_E2EE_LOCKED;
      if (!impl_->opened)
        raise("the E2EE store is not open");
      auto it = impl_->accounts.find(handle);
      if (it == impl_->accounts.end() || !it->second.identity)
        raise("'" + handle + "' has no identity");
      bundle = secret_bundle_json(*it->second.identity);
      // wrapPrivateBundle's draws: the salt, then (after the key) the nonce.
      salt = random_bytes(*options_.random, crypto_pwhash_SALTBYTES);
      nonce = random_bytes(*options_.random, crypto_secretbox_NONCEBYTES);
    }
    Secret secret_bundle(std::move(bundle));
    if (passphrase.empty())
      raise("a backup needs a passphrase");
    // Argon2id outside the lock: it takes about a second.
    const unsigned long long ops = crypto_pwhash_OPSLIMIT_MODERATE;
    const std::size_t mem = crypto_pwhash_MEMLIMIT_MODERATE;
    Secret key(crypto_secretbox_KEYBYTES);
    if (crypto_pwhash(key.data(), key.size(), passphrase.data(), passphrase.size(), u8(salt), ops,
                      mem, crypto_pwhash_ALG_ARGON2ID13) != 0)
      raise("Argon2id ran out of memory");
    Bytes ct(secret_bundle.size() + crypto_secretbox_MACBYTES, '\0');
    crypto_secretbox_easy(reinterpret_cast<unsigned char *>(ct.data()), secret_bundle.data(),
                          secret_bundle.size(), u8(nonce), key.data());
    E2eeBackup out;
    out.ciphertext = b64(ct);
    out.kdf_salt = b64(salt);
    out.opslimit = static_cast<std::int64_t>(ops);
    out.memlimit = static_cast<std::int64_t>(mem);
    out.nonce = b64(nonce);
    return out;
  } catch (const E2eeError &e) {
    fail(error, e.what());
    return std::nullopt;
  }
#else
  (void)handle;
  (void)passphrase;
  fail(error, "this build has no E2EE (libsodium was not found)");
  return std::nullopt;
#endif
}

std::optional<E2eePublicKeys> E2eeStore::restore(const std::string &handle, const E2eeBackup &blob,
                                                 const std::string &passphrase,
                                                 std::string *error) {
#ifdef CVC_HAVE_SODIUM
  try {
    check_handle(handle);
    {
      CVC_E2EE_LOCKED;
      if (!impl_->opened)
        raise("the E2EE store is not open");
      auto it = impl_->accounts.find(handle);
      if (it != impl_->accounts.end() && it->second.identity)
        raise("'" + handle + "' has an identity already (forget it first)");
    }
    if (blob.kdf != "argon2id" || (blob.alg != "argon2id13" && !blob.alg.empty()))
      raise("the backup is not an Argon2id one");
    if (blob.version != 1)
      raise("the backup is not version 1");
    const std::int64_t ops = blob.opslimit ? blob.opslimit : crypto_pwhash_OPSLIMIT_MODERATE;
    const std::int64_t mem = blob.memlimit ? blob.memlimit : crypto_pwhash_MEMLIMIT_MODERATE;
    if (ops < static_cast<std::int64_t>(crypto_pwhash_OPSLIMIT_MIN) || ops > options_.max_opslimit)
      raise("the backup's Argon2id time limit is out of range");
    if (mem < static_cast<std::int64_t>(crypto_pwhash_MEMLIMIT_MIN) || mem > options_.max_memlimit)
      raise("the backup's Argon2id memory limit is out of range");
    const Bytes salt = unb64n(blob.kdf_salt, crypto_pwhash_SALTBYTES, "the backup's salt");
    const Bytes nonce = unb64n(blob.nonce, crypto_secretbox_NONCEBYTES, "the backup's nonce");
    const Bytes ct = unb64(blob.ciphertext, "the backup");
    if (ct.size() < crypto_secretbox_MACBYTES)
      raise("the backup is too short");
    Secret key(crypto_secretbox_KEYBYTES);
    if (crypto_pwhash(key.data(), key.size(), passphrase.data(), passphrase.size(), u8(salt),
                      static_cast<unsigned long long>(ops), static_cast<std::size_t>(mem),
                      crypto_pwhash_ALG_ARGON2ID13) != 0)
      raise("Argon2id ran out of memory");
    Secret pt(ct.size() - crypto_secretbox_MACBYTES);
    if (crypto_secretbox_open_easy(pt.data(), u8(ct), ct.size(), u8(nonce), key.data()) != 0)
      raise("backup: incorrect passphrase or corrupt blob");
    Identity id = parse_secret_bundle(parse_json(pt.b, "the backup's contents"));
    std::optional<E2eePublicKeys> out;
    CVC_E2EE_LOCKED;
    if (!mutate(impl_->accounts, handle, options_.path, error, [&](Account &a) {
          if (a.identity)
            raise("'" + handle + "' has an identity already (forget it first)");
          a = Account{};
          a.identity = std::move(id);
          a.next_opk_id = 100001; // past what another device numbered from 1
          out = public_keys_of(*a.identity);
          return true;
        }))
      return std::nullopt;
    return out;
  } catch (const E2eeError &e) {
    fail(error, e.what());
    return std::nullopt;
  }
#else
  (void)handle;
  (void)blob;
  (void)passphrase;
  fail(error, "this build has no E2EE (libsodium was not found)");
  return std::nullopt;
#endif
}

std::string E2eeStore::export_state(const std::string &handle) const {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  auto it = impl_->accounts.find(handle);
  if (it == impl_->accounts.end())
    return "{}";
  return account_state_json(it->second);
#else
  (void)handle;
  return "{}";
#endif
}

bool E2eeStore::import_state(const std::string &handle, const std::string &json,
                             std::string *error) {
#ifdef CVC_HAVE_SODIUM
  CVC_E2EE_LOCKED;
  try {
    check_handle(handle);
    if (!impl_->opened)
      raise("the E2EE store is not open");
    Account parsed = parse_account_state(parse_json(json, "the state"));
    return mutate(impl_->accounts, handle, options_.path, error, [&](Account &a) {
      if (a.identity)
        raise("'" + handle + "' has an identity already (forget it first)");
      a = std::move(parsed);
      return true;
    });
  } catch (const E2eeError &e) {
    return fail(error, e.what());
  }
#else
  (void)handle;
  (void)json;
  return fail(error, "this build has no E2EE (libsodium was not found)");
#endif
}

} // namespace net
} // namespace cvc

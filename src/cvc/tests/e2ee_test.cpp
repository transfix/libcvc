// cvc::net::E2eeStore against TX.WTF's own client crypto (docs/E2EE.md).
//
// data/e2ee/txwtf_vectors.json is what tools/e2ee/txwtf_vectors.js wrote by running txwtf's
// e2ee.js and messaging-e2ee.js (libsodium-wrappers-sumo 0.7.15) for three users through a
// stand-in server, with every random draw from a seeded stream. The replay below runs the same
// three users through E2eeStore with the same streams and asserts:
//   - every published bundle, sealed body, wrapped key, ratchet header and X3DH envelope equals
//     e2ee.js's, byte for byte (so what the store seals, the web client opens, and the reverse);
//   - every message e2ee.js sealed opens here, out of order, across ratchet turns, without a
//     one-time prekey, and in a group;
//   - the stores end holding what e2ee.js's stores hold (identity, sessions, cached plaintext);
//   - a passphrase backup e2ee.js wrapped restores here, and one wrapped here is e2ee.js's.
// Then what the port does differently: a tampered or forged message changes nothing (e2ee.js's
// ratchet advances before it authenticates), the window of skipped keys, the file (owner-only,
// locked to one process, there after a restart), and no error quoting a key.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cvc/net/e2ee.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/types.h>
#include <deque>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#if !defined(_WIN32)
#include <sys/stat.h>
#endif

namespace {

namespace fs = std::filesystem;
namespace se = cvc::state_exec;
using cvc::net::E2eeBackup;
using cvc::net::E2eeBundle;
using cvc::net::E2eeMessage;
using cvc::net::E2eeOpened;
using cvc::net::E2eeStore;

const fs::path kData = fs::path(__FILE__).parent_path() / "data" / "e2ee";

// --- the vectors, through the DSL's own JSON reader -------------------------------------------

se::value_t parse(const std::string &text) {
  const se::value_t t(text);
  return se::builtin_json_parse(std::span<const se::value_t>(&t, 1));
}

std::string encode(const se::value_t &v) {
  const se::value_t r = se::builtin_json_encode(std::span<const se::value_t>(&v, 1));
  return std::get<std::string>(r.v);
}

se::value_t load_vectors() {
  std::ifstream in(kData / "txwtf_vectors.json", std::ios::binary);
  std::ostringstream s;
  s << in.rdbuf();
  return parse(s.str());
}

const se::value_t &at(const se::value_t &v, const std::string &key) {
  static const se::value_t nil;
  if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v))
    for (const auto &kv : **d)
      if (kv.first == key)
        return kv.second;
  return nil;
}
std::string str(const se::value_t &v) {
  const std::string *s = std::get_if<std::string>(&v.v);
  return s ? *s : std::string();
}
std::int64_t num(const se::value_t &v) {
  const std::int64_t *i = std::get_if<std::int64_t>(&v.v);
  return i ? *i : -1;
}
const std::vector<se::value_t> &items(const se::value_t &v) {
  static const std::vector<se::value_t> none;
  const se::list_ptr *l = std::get_if<se::list_ptr>(&v.v);
  return l ? **l : none;
}
std::optional<std::string> opt_str(const se::value_t &v) {
  if (v.is_nil())
    return std::nullopt;
  return str(v);
}

// A JSON value written with every object's keys sorted, so two stores compare whatever order
// they keep their entries in.
std::string canonical(const se::value_t &v) {
  if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v)) {
    std::map<std::string, std::string> m;
    for (const auto &kv : **d)
      m[kv.first] = canonical(kv.second);
    std::string out = "{";
    for (const auto &[k, val] : m)
      out += (out.size() > 1 ? "," : "") + encode(se::value_t(k)) + ":" + val;
    return out + "}";
  }
  if (const se::list_ptr *l = std::get_if<se::list_ptr>(&v.v)) {
    std::string out = "[";
    for (const se::value_t &x : **l)
      out += (out.size() > 1 ? "," : "") + canonical(x);
    return out + "]";
  }
  return encode(v);
}

E2eeBundle bundle_of(const se::value_t &b) {
  E2eeBundle out;
  out.identity_key_ed25519 = str(at(b, "identity_key_ed25519"));
  out.identity_key_x25519 = str(at(b, "identity_key_x25519"));
  out.registration_id = num(at(b, "registration_id"));
  const se::value_t &spk = at(b, "signed_prekey");
  out.signed_prekey_id = num(at(spk, "key_id"));
  out.signed_prekey = str(at(spk, "public_key"));
  out.signed_prekey_signature = str(at(spk, "signature"));
  const se::value_t &opk = at(b, "one_time_prekey");
  if (!opk.is_nil())
    out.one_time_prekey = cvc::net::E2eePreKey{num(at(opk, "key_id")), str(at(opk, "public_key"))};
  return out;
}

E2eeMessage message_of(const se::value_t &m) {
  E2eeMessage out;
  out.id = num(at(m, "id"));
  out.sender_id = num(at(m, "sender_id"));
  out.algorithm = str(at(m, "algorithm"));
  out.envelope = opt_str(at(m, "envelope"));
  out.ciphertext = opt_str(at(m, "ciphertext"));
  out.wrapped_key = opt_str(at(m, "wrapped_key"));
  out.header = opt_str(at(m, "header"));
  out.is_sender = at(m, "is_sender").is_truthy();
  out.locked = at(m, "locked").is_truthy();
  return out;
}

E2eeBackup backup_of(const se::value_t &b) {
  E2eeBackup out;
  out.ciphertext = str(at(b, "ciphertext"));
  out.kdf = str(at(b, "kdf"));
  out.kdf_salt = str(at(b, "kdf_salt"));
  out.nonce = str(at(b, "nonce"));
  out.version = num(at(b, "version"));
  const se::value_t &p = at(b, "kdf_params");
  out.opslimit = num(at(p, "opslimit"));
  out.memlimit = num(at(p, "memlimit"));
  out.alg = str(at(p, "alg"));
  return out;
}

std::string b64_decode_for_seed(const std::string &b64) {
  // Seeds are base64 of 32 bytes; decode with a tiny local table (the store's own decoder is
  // private to it).
  static const std::string alphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  int bits = 0, acc = 0;
  for (char c : b64) {
    if (c == '=')
      break;
    const auto p = alphabet.find(c);
    if (p == std::string::npos)
      continue;
    acc = (acc << 6) | static_cast<int>(p);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((acc >> bits) & 0xFF));
    }
  }
  return out;
}

std::unique_ptr<E2eeStore> seeded_store(const se::value_t &doc, const std::string &who,
                                        const std::string &path = std::string()) {
  E2eeStore::Options o;
  o.path = path;
  o.random = cvc::net::e2ee_seeded_random(b64_decode_for_seed(str(at(at(doc, "seeds"), who))));
  auto s = std::make_unique<E2eeStore>(o);
  std::string why;
  EXPECT_TRUE(s->open(&why)) << why;
  return s;
}

// The base64 secrets a state holds (every *priv, RK, CK, skipped key), for "no error quotes one".
void collect_secrets(const se::value_t &v, const std::string &key, std::vector<std::string> &out) {
  if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v)) {
    for (const auto &kv : **d)
      collect_secrets(kv.second, kv.first, out);
    return;
  }
  if (const std::string *s = std::get_if<std::string>(&v.v)) {
    const bool secret = key == "priv" || key.find("_priv") != std::string::npos || key == "RK" ||
                        key == "CKs" || key == "CKr" || key.find(':') != std::string::npos;
    if (secret && s->size() >= 40)
      out.push_back(*s);
  }
}

// The replay ------------------------------------------------------------------------------------

struct Replay {
  se::value_t doc;
  std::map<std::string, std::unique_ptr<E2eeStore>> stores;
  std::map<std::string, std::deque<E2eeBundle>> bundles; // fetched, not used yet, by fetcher
  std::vector<std::string> errors;                       // every error the stores gave
  int sends = 0, opens = 0, tampers = 0;

  void publish(const se::value_t &step) {
    const std::string who = str(at(step, "who"));
    stores[who] = seeded_store(doc, who);
    std::string why;
    auto keys =
        stores[who]->create_identity(who, static_cast<int>(num(at(step, "opk_count"))), &why);
    ASSERT_TRUE(keys) << why;
    const se::value_t &p = at(step, "payload");
    EXPECT_EQ(keys->identity_key_ed25519, str(at(p, "identity_key_ed25519"))) << who;
    EXPECT_EQ(keys->identity_key_x25519, str(at(p, "identity_key_x25519"))) << who;
    EXPECT_EQ(keys->registration_id, num(at(p, "registration_id"))) << who;
    const se::value_t &spk = at(p, "signed_prekey");
    EXPECT_EQ(keys->signed_prekey_id, num(at(spk, "key_id")));
    EXPECT_EQ(keys->signed_prekey, str(at(spk, "public_key")));
    EXPECT_EQ(keys->signed_prekey_signature, str(at(spk, "signature")));
    const auto &opks = items(at(p, "one_time_prekeys"));
    ASSERT_EQ(keys->one_time_prekeys.size(), opks.size());
    for (std::size_t i = 0; i < opks.size(); ++i) {
      EXPECT_EQ(keys->one_time_prekeys[i].key_id, num(at(opks[i], "key_id")));
      EXPECT_EQ(keys->one_time_prekeys[i].public_key, str(at(opks[i], "public_key")));
    }
  }

  void send(const se::value_t &step) {
    const std::string who = str(at(step, "who"));
    const std::string conv = std::to_string(num(at(step, "conv")));
    const std::string text = str(at(step, "text"));
    const se::value_t &body = at(step, "body");
    E2eeStore &s = *stores[who];
    const auto &keys = items(at(body, "recipient_keys"));
    if (!at(step, "group").is_truthy()) {
      std::optional<E2eeBundle> b;
      if (!s.has_session(who, conv)) {
        // The first send asks for a bundle, and changes nothing until it has one.
        EXPECT_TRUE(s.seal(who, conv, text).need_bundle);
        ASSERT_FALSE(bundles[who].empty());
        b = bundles[who].front();
        bundles[who].pop_front();
      }
      const cvc::net::E2eeSealed sealed = s.seal(who, conv, text, b);
      ASSERT_TRUE(sealed.ok) << sealed.error;
      EXPECT_EQ(sealed.algorithm, str(at(body, "algorithm")));
      EXPECT_EQ(sealed.ciphertext, str(at(body, "ciphertext"))) << who << " send " << text;
      ASSERT_EQ(keys.size(), 1u);
      EXPECT_EQ(sealed.wrapped_key, str(at(keys[0], "wrapped_key")));
      EXPECT_EQ(sealed.header, str(at(keys[0], "header")));
      EXPECT_EQ(sealed.envelope, opt_str(at(body, "envelope")));
    } else {
      std::vector<cvc::net::E2eeGroupMember> members;
      for (const se::value_t &k : keys) {
        cvc::net::E2eeGroupMember m;
        m.user_id = num(at(k, "recipient"));
        if (!s.has_session(who, conv + ":" + std::to_string(m.user_id))) {
          EXPECT_FALSE(bundles[who].empty());
          m.bundle = bundles[who].front();
          bundles[who].pop_front();
        }
        members.push_back(m);
      }
      // Without the bundles it names who needs one.
      std::vector<cvc::net::E2eeGroupMember> bare = members;
      for (auto &m : bare)
        m.bundle.reset();
      const auto asked = s.seal_group(who, conv, text, bare);
      for (const auto &m : members) {
        if (m.bundle) {
          EXPECT_NE(std::find(asked.need_bundles.begin(), asked.need_bundles.end(), m.user_id),
                    asked.need_bundles.end());
        }
      }
      const cvc::net::E2eeGroupSealed sealed = s.seal_group(who, conv, text, members);
      ASSERT_TRUE(sealed.ok) << sealed.error;
      EXPECT_EQ(sealed.algorithm, "grp-dr-v1");
      EXPECT_EQ(sealed.ciphertext, str(at(body, "ciphertext")));
      ASSERT_EQ(sealed.recipient_keys.size(), keys.size());
      for (std::size_t i = 0; i < keys.size(); ++i) {
        EXPECT_EQ(sealed.recipient_keys[i].recipient, num(at(keys[i], "recipient")));
        EXPECT_EQ(sealed.recipient_keys[i].wrapped_key, str(at(keys[i], "wrapped_key")));
        EXPECT_EQ(sealed.recipient_keys[i].header, str(at(keys[i], "header")));
      }
    }
    std::string why;
    EXPECT_TRUE(s.remember(who, num(at(step, "id")), text, &why)) << why;
    ++sends;
  }

  void open(const se::value_t &step) {
    const std::string who = str(at(step, "who"));
    const std::string conv = std::to_string(num(at(step, "conv")));
    E2eeStore &s = *stores[who];
    const se::value_t &tamper = at(doc, "tamper");
    if (num(at(step, "id")) == num(at(tamper, "genuine_id")) && who == str(at(tamper, "who"))) {
      // Before the genuine message opens: its tampered copies fail, and change nothing.
      for (const char *k : {"wrapped_key_flipped", "body_flipped"}) {
        const std::string before = s.export_state(who);
        const E2eeOpened o = s.open_message(who, conv, message_of(at(tamper, k)));
        EXPECT_EQ(o.status, E2eeOpened::Status::failed) << k;
        errors.push_back(o.error);
        EXPECT_EQ(s.export_state(who), before) << k << ": a failed open changed the session";
        ++tampers;
      }
    }
    const E2eeOpened o = s.open_message(who, conv, message_of(at(step, "message")));
    ASSERT_EQ(o.status, E2eeOpened::Status::ok)
        << who << " #" << num(at(step, "id")) << ": " << o.error;
    EXPECT_EQ(o.text, str(at(step, "text")));
    EXPECT_EQ(o.attachments, "[]");
    // Opened once, it comes from the cache, never through the ratchet again.
    const E2eeOpened again = s.open_message(who, conv, message_of(at(step, "message")));
    EXPECT_EQ(again.status, E2eeOpened::Status::cached);
    EXPECT_EQ(again.text, o.text);
    ++opens;
  }

  void run() {
    for (const se::value_t &step : items(at(doc, "steps"))) {
      const std::string op = str(at(step, "op"));
      if (op == "publish")
        publish(step);
      else if (op == "bundle")
        bundles[str(at(step, "who"))].push_back(bundle_of(at(step, "bundle")));
      else if (op == "send")
        send(step);
      else if (op == "open")
        open(step);
      if (::testing::Test::HasFatalFailure())
        return;
    }
  }
};

bool have_e2ee() { return cvc::net::e2ee_available(); }

TEST(E2eeVectors, TheTranscriptOfTxwtfsClientReplaysByteForByte) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  Replay r;
  r.doc = load_vectors();
  ASSERT_FALSE(items(at(r.doc, "steps")).empty()) << "no vectors at " << kData;
  r.run();
  EXPECT_EQ(r.sends, 10);
  EXPECT_EQ(r.opens, 12);
  EXPECT_EQ(r.tampers, 2);
  // Each store now holds what e2ee.js's store held: identity (its used one-time prekeys gone),
  // every session's ratchet, every cached plaintext.
  for (const auto &[who, store] : r.stores)
    EXPECT_EQ(canonical(parse(store->export_state(who))),
              canonical(at(at(r.doc, "final_stores"), who)))
        << who;
  // No error quotes a secret.
  std::vector<std::string> secrets;
  for (const auto &[who, store] : r.stores)
    collect_secrets(parse(store->export_state(who)), "", secrets);
  ASSERT_GT(secrets.size(), 20u);
  for (const std::string &e : r.errors)
    for (const std::string &k : secrets)
      EXPECT_EQ(e.find(k), std::string::npos) << e;
}

// e2ee.js's state imported (the layout messaging-e2ee.js keeps in IndexedDB) carries on: bob's
// final store, loaded into a fresh one, opens a new message alice's final store seals.
TEST(E2eeVectors, AStateFromTxwtfsStoreCarriesOn) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  const se::value_t doc = load_vectors();
  const se::value_t &finals = at(doc, "final_stores");
  E2eeStore alice({}), bob({});
  ASSERT_TRUE(alice.open());
  ASSERT_TRUE(bob.open());
  std::string why;
  ASSERT_TRUE(alice.import_state("alice", encode(at(finals, "alice")), &why)) << why;
  ASSERT_TRUE(bob.import_state("bob", encode(at(finals, "bob")), &why)) << why;
  EXPECT_FALSE(bob.import_state("bob", encode(at(finals, "bob")), &why)) << "imported twice";
  ASSERT_TRUE(alice.has_session("alice", "10"));
  const auto sealed = alice.seal("alice", "10", "carrying on");
  ASSERT_TRUE(sealed.ok) << sealed.error;
  EXPECT_FALSE(sealed.envelope) << "bob has replied; no envelope any more";
  E2eeMessage m;
  m.id = 900;
  m.sender_id = 1;
  m.ciphertext = sealed.ciphertext;
  m.wrapped_key = sealed.wrapped_key;
  m.header = sealed.header;
  const E2eeOpened o = bob.open_message("bob", "10", m);
  EXPECT_EQ(o.status, E2eeOpened::Status::ok) << o.error;
  EXPECT_EQ(o.text, "carrying on");
}

// A forged message with a fresh ratchet key (what an untrusted server can inject) fails and
// leaves the session as it was, so the next genuine message still opens. e2ee.js would have run a
// DH ratchet step before failing, and every later message of that page load would fail with it.
TEST(E2eeStore, AForgedRatchetKeyChangesNothing) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  E2eeStore a({}), b({});
  ASSERT_TRUE(a.open());
  ASSERT_TRUE(b.open());
  auto ka = a.create_identity("a", 1);
  auto kb = b.create_identity("b", 1);
  ASSERT_TRUE(ka && kb);
  E2eeBundle bundle{kb->identity_key_ed25519,    kb->identity_key_x25519,
                    kb->registration_id,         kb->signed_prekey_id,
                    kb->signed_prekey,           kb->signed_prekey_signature,
                    kb->one_time_prekeys.front()};
  auto first = a.seal("a", "5", "one", bundle);
  ASSERT_TRUE(first.ok) << first.error;
  E2eeMessage m;
  m.id = 1;
  m.sender_id = 1;
  m.envelope = first.envelope;
  m.ciphertext = first.ciphertext;
  m.wrapped_key = first.wrapped_key;
  m.header = first.header;
  ASSERT_EQ(b.open_message("b", "5", m).status, E2eeOpened::Status::ok);

  const std::string before = b.export_state("b");
  // {"dh": <32 bytes nobody holds>, "pn": 0, "n": 0}, base64'd as the header is.
  E2eeMessage forged = m;
  forged.id = 2;
  forged.header =
      "eyJkaCI6IkFBRUNBd1FGQmdjSUNRb0xEQTBPRHhBUkVoTVVGUllYR0JrYUd4d2RIaDg9IiwicG4iOjAsIm4iOjB9";
  const E2eeOpened f = b.open_message("b", "5", forged);
  EXPECT_EQ(f.status, E2eeOpened::Status::failed);
  EXPECT_EQ(b.export_state("b"), before);

  auto second = a.seal("a", "5", "two");
  ASSERT_TRUE(second.ok);
  m.id = 3;
  m.ciphertext = second.ciphertext;
  m.wrapped_key = second.wrapped_key;
  m.header = second.header;
  const E2eeOpened o = b.open_message("b", "5", m);
  EXPECT_EQ(o.status, E2eeOpened::Status::ok) << o.error;
  EXPECT_EQ(o.text, "two");
}

// Sessions are per device: a message whose session another device holds (no envelope here)
// says so; one whose envelope names a one-time prekey this device never had says that.
TEST(E2eeStore, MessagesForAnotherDeviceSaySo) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  E2eeStore a({}), b({}), b2({});
  ASSERT_TRUE(a.open() && b.open() && b2.open());
  auto kb = b.create_identity("b", 2);
  ASSERT_TRUE(a.create_identity("a", 0) && kb);
  E2eeBundle bundle{kb->identity_key_ed25519,    kb->identity_key_x25519,
                    kb->registration_id,         kb->signed_prekey_id,
                    kb->signed_prekey,           kb->signed_prekey_signature,
                    kb->one_time_prekeys.front()};
  auto s = a.seal("a", "9", "hello", bundle);
  ASSERT_TRUE(s.ok);
  E2eeMessage m;
  m.id = 11;
  m.sender_id = 1;
  m.ciphertext = s.ciphertext;
  m.wrapped_key = s.wrapped_key;
  m.header = s.header;
  // Without the envelope: no session here.
  ASSERT_TRUE(b2.create_identity("b", 0));
  EXPECT_EQ(b2.open_message("b", "9", m).status, E2eeOpened::Status::no_session);
  // With it, on a device with the right identity (b's backup, say) but not that one-time key.
  m.envelope = s.envelope;
  std::string why;
  auto blob = b.backup("b", "pw", &why);
  ASSERT_TRUE(blob) << why;
  E2eeStore b3({});
  ASSERT_TRUE(b3.open());
  ASSERT_TRUE(b3.restore("b", *blob, "pw", &why)) << why;
  const E2eeOpened o = b3.open_message("b", "9", m);
  EXPECT_EQ(o.status, E2eeOpened::Status::failed);
  EXPECT_NE(o.error.find("one-time prekey"), std::string::npos) << o.error;
  // The device that published it opens it.
  EXPECT_EQ(b.open_message("b", "9", m).status, E2eeOpened::Status::ok);
  // Our own send comes back as ours, or from the cache once remembered.
  m.is_sender = true;
  m.id = 12;
  EXPECT_EQ(a.open_message("a", "9", m).status, E2eeOpened::Status::own);
  ASSERT_TRUE(a.remember("a", 12, "hello"));
  EXPECT_EQ(a.open_message("a", "9", m).status, E2eeOpened::Status::cached);
}

// More than e2ee.js's MAX_SKIP (1000) messages ahead is refused, and the session survives it.
TEST(E2eeStore, TooFarAheadIsRefused) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  E2eeStore a({}), b({});
  ASSERT_TRUE(a.open() && b.open());
  auto kb = b.create_identity("b", 0);
  ASSERT_TRUE(a.create_identity("a", 0) && kb);
  E2eeBundle bundle{kb->identity_key_ed25519,
                    kb->identity_key_x25519,
                    kb->registration_id,
                    kb->signed_prekey_id,
                    kb->signed_prekey,
                    kb->signed_prekey_signature,
                    std::nullopt};
  std::vector<cvc::net::E2eeSealed> sent;
  sent.push_back(a.seal("a", "1", "m0", bundle));
  for (int i = 1; i < 1003; ++i)
    sent.push_back(a.seal("a", "1", "m" + std::to_string(i)));
  auto msg = [&](int i) {
    E2eeMessage m;
    m.id = 1000 + i;
    m.envelope = sent[0].envelope;
    m.ciphertext = sent[i].ciphertext;
    m.wrapped_key = sent[i].wrapped_key;
    m.header = sent[i].header;
    return m;
  };
  ASSERT_EQ(b.open_message("b", "1", msg(0)).status, E2eeOpened::Status::ok);
  const E2eeOpened far = b.open_message("b", "1", msg(1002));
  EXPECT_EQ(far.status, E2eeOpened::Status::failed);
  EXPECT_NE(far.error.find("too many skipped"), std::string::npos) << far.error;
  EXPECT_EQ(b.open_message("b", "1", msg(1)).status, E2eeOpened::Status::ok);
  EXPECT_EQ(b.open_message("b", "1", msg(1000)).status, E2eeOpened::Status::ok);
  EXPECT_EQ(b.open_message("b", "1", msg(500)).status, E2eeOpened::Status::ok) << "a skipped key";
}

// A backup e2ee.js wrapped (Argon2id at MODERATE) restores here; one wrapped here from the same
// stream is the same blob; a wrong passphrase, or limits past the store's, are refused.
TEST(E2eeVectors, BackupsAreTxwtfs) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  const se::value_t doc = load_vectors();
  const se::value_t &bk = at(doc, "backup");
  const E2eeBackup blob = backup_of(at(bk, "blob"));
  const std::string pass = str(at(bk, "passphrase"));
  E2eeStore s({});
  ASSERT_TRUE(s.open());
  std::string why;
  EXPECT_FALSE(s.restore("bob", blob, pass + "x", &why));
  EXPECT_NE(why.find("incorrect passphrase"), std::string::npos) << why;
  auto keys = s.restore("bob", blob, pass, &why);
  ASSERT_TRUE(keys) << why;
  const se::value_t &bundle = at(bk, "bundle");
  EXPECT_EQ(keys->identity_key_ed25519, str(at(bundle, "sign_pub")));
  EXPECT_EQ(keys->identity_key_x25519, str(at(bundle, "dh_pub")));
  EXPECT_EQ(keys->registration_id, num(at(bundle, "registration_id")));
  EXPECT_TRUE(keys->one_time_prekeys.empty());
  auto more = s.add_prekeys("bob", 2, &why);
  ASSERT_TRUE(more) << why;
  EXPECT_EQ(more->front().key_id, 100001) << "numbered past another device's";

  E2eeStore::Options o;
  o.random = cvc::net::e2ee_seeded_random(b64_decode_for_seed(str(at(at(doc, "seeds"), "backup"))));
  E2eeStore w(o);
  ASSERT_TRUE(w.open());
  ASSERT_TRUE(w.import_state("bob", encode(at(at(doc, "final_stores"), "bob")), &why)) << why;
  auto mine = w.backup("bob", pass, &why);
  ASSERT_TRUE(mine) << why;
  EXPECT_EQ(mine->ciphertext, blob.ciphertext);
  EXPECT_EQ(mine->kdf_salt, blob.kdf_salt);
  EXPECT_EQ(mine->nonce, blob.nonce);
  EXPECT_EQ(mine->opslimit, blob.opslimit);
  EXPECT_EQ(mine->memlimit, blob.memlimit);

  E2eeStore::Options tight;
  tight.max_memlimit = 64 << 20;
  E2eeStore t(tight);
  ASSERT_TRUE(t.open());
  EXPECT_FALSE(t.restore("bob", blob, pass, &why));
  EXPECT_NE(why.find("memory limit"), std::string::npos) << why;
}

// The file: there after a restart, owner-only, and locked to one store at a time.
TEST(E2eeStore, TheFileKeepsTheKeysForOneProcess) {
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  const fs::path dir =
      fs::temp_directory_path() /
      ("cvc_e2ee_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
  fs::remove_all(dir);
  const fs::path file = dir / "keys" / "store";
  std::string why;
  std::string ed;
  {
    E2eeStore::Options o;
    o.path = file.string();
    E2eeStore s(o);
    ASSERT_TRUE(s.open(&why)) << why;
    auto k = s.create_identity("txwtf.4", 3, &why);
    ASSERT_TRUE(k) << why;
    ed = k->identity_key_ed25519;
    ASSERT_TRUE(s.remember("txwtf.4", 77, "kept", &why)) << why;

    E2eeStore::Options o2;
    o2.path = file.string();
    E2eeStore other(o2);
    EXPECT_FALSE(other.open(&why));
    EXPECT_NE(why.find("in use by another process"), std::string::npos) << why;
#if !defined(_WIN32)
    struct stat st {};
    ASSERT_EQ(::stat(file.c_str(), &st), 0);
    EXPECT_EQ(st.st_mode & 0777, 0600);
    ASSERT_EQ(::stat(file.parent_path().c_str(), &st), 0);
    EXPECT_EQ(st.st_mode & 0777, 0700);
#endif
  }
  E2eeStore::Options o;
  o.path = file.string();
  E2eeStore again(o);
  ASSERT_TRUE(again.open(&why)) << why;
  auto k = again.public_keys("txwtf.4");
  ASSERT_TRUE(k);
  EXPECT_EQ(k->identity_key_ed25519, ed);
  EXPECT_EQ(k->one_time_prekeys.size(), 3u);
  E2eeMessage m;
  m.id = 77;
  m.is_sender = true;
  const E2eeOpened o77 = again.open_message("txwtf.4", "1", m);
  EXPECT_EQ(o77.status, E2eeOpened::Status::cached);
  EXPECT_EQ(o77.text, "kept");
  EXPECT_TRUE(again.forget("txwtf.4", &why)) << why;
  EXPECT_FALSE(again.has_identity("txwtf.4"));
#if !defined(_WIN32)
  // Open to others: refused, as the credential store refuses.
  ::chmod(file.c_str(), 0644);
  E2eeStore::Options o3;
  o3.path = (dir / "keys" / "other").string();
  { std::ofstream(o3.path) << "{}"; }
  ::chmod(o3.path.c_str(), 0644);
  E2eeStore open_to_all(o3);
  EXPECT_FALSE(open_to_all.open(&why));
  EXPECT_NE(why.find("owner-only"), std::string::npos) << why;
#endif
  std::error_code ec;
  fs::remove_all(dir, ec);
}

TEST(E2eeStore, NamesAndFingerprints) {
  EXPECT_TRUE(E2eeStore::valid_handle("txwtf.dev-tx-wtf_4"));
  EXPECT_FALSE(E2eeStore::valid_handle("a b"));
  EXPECT_FALSE(E2eeStore::valid_handle(""));
  EXPECT_TRUE(E2eeStore::valid_conversation("12"));
  EXPECT_FALSE(E2eeStore::valid_conversation("1:2"));
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  const std::string fp = cvc::net::e2ee_fingerprint("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8=");
  EXPECT_EQ(fp.size(), 47u) << fp; // 40 hex digits in 8 groups of 5
  EXPECT_EQ(cvc::net::e2ee_fingerprint("not base64!"), "");
}

// Live interop with TX.WTF's client, the other way round from the vectors and with the system's
// randomness: tools/e2ee/interop.sh runs this with CVC_E2EE_INTEROP_DIR set, then
// tools/e2ee/txwtf_interop.js (e2ee.js, as the web page runs it), then this again.
//   1. No reply yet: carl (this store) seals three messages for jill, whose keys this store also
//      made; native.json carries them and jill's state in messaging-e2ee.js's layout.
//   2. e2ee.js loads jill, opens carl's messages and replies twice: js_reply.json.
//   3. This opens jill's replies with carl's store as step 1 left it (carl.json).
// Without the variable it is skipped.
TEST(E2eeInterop, WithTxwtfsClientThroughFiles) {
  const char *dir_env = std::getenv("CVC_E2EE_INTEROP_DIR");
  if (!dir_env || !*dir_env)
    GTEST_SKIP() << "set CVC_E2EE_INTEROP_DIR (tools/e2ee/interop.sh)";
  if (!have_e2ee())
    GTEST_SKIP() << "libcvc built without libsodium";
  const fs::path dir(dir_env);
  const std::vector<std::string> texts = {"hello from C++", "second, with ünïcödé and 🔒",
                                          "third: \"quotes\" \\ and a\nnewline"};
  std::string why;
  if (!fs::exists(dir / "js_reply.json")) {
    E2eeStore carl({}), jill({});
    ASSERT_TRUE(carl.open() && jill.open());
    ASSERT_TRUE(carl.create_identity("carl", 0, &why)) << why;
    auto kj = jill.create_identity("jill", 5, &why);
    ASSERT_TRUE(kj) << why;
    E2eeBundle b{kj->identity_key_ed25519,    kj->identity_key_x25519, kj->registration_id,
                 kj->signed_prekey_id,        kj->signed_prekey,       kj->signed_prekey_signature,
                 kj->one_time_prekeys.front()};
    std::string msgs = "[";
    for (std::size_t i = 0; i < texts.size(); ++i) {
      auto s =
          carl.seal("carl", "77", texts[i], i == 0 ? std::optional<E2eeBundle>(b) : std::nullopt);
      ASSERT_TRUE(s.ok) << s.error;
      const std::int64_t id = 501 + static_cast<std::int64_t>(i);
      ASSERT_TRUE(carl.remember("carl", id, texts[i]));
      se::value_t m =
          se::make_dict({{"id", se::value_t(id)},
                         {"conversation_id", se::value_t(std::int64_t(77))},
                         {"sender_id", se::value_t(std::int64_t(1))},
                         {"algorithm", s.algorithm},
                         {"envelope", s.envelope ? se::value_t(*s.envelope) : se::nil_value},
                         {"is_sender", se::value_t(false)},
                         {"locked", se::value_t(false)},
                         {"ciphertext", s.ciphertext},
                         {"wrapped_key", s.wrapped_key},
                         {"header", s.header}});
      msgs += (i ? "," : "") + encode(m);
    }
    msgs += "]";
    std::vector<se::value_t> expect;
    for (const auto &t : texts)
      expect.emplace_back(t);
    const std::string native = "{\"conv\":77,\"jill_state\":" + jill.export_state("jill") +
                               ",\"messages\":" + msgs +
                               ",\"expect\":" + encode(se::make_list(expect)) + "}";
    fs::create_directories(dir);
    std::ofstream(dir / "native.json", std::ios::binary) << native;
    std::ofstream(dir / "carl.json", std::ios::binary) << carl.export_state("carl");
    std::printf("wrote %s: %zu messages sealed here, for e2ee.js to open\n",
                (dir / "native.json").string().c_str(), texts.size());
    return;
  }
  std::ifstream in(dir / "js_reply.json", std::ios::binary),
      cin(dir / "carl.json", std::ios::binary);
  std::ostringstream reply, carl_state;
  reply << in.rdbuf();
  carl_state << cin.rdbuf();
  const se::value_t r = parse(reply.str());
  E2eeStore carl({});
  ASSERT_TRUE(carl.open());
  ASSERT_TRUE(carl.import_state("carl", carl_state.str(), &why)) << why;
  const auto &opened = items(at(r, "opened"));
  ASSERT_EQ(opened.size(), texts.size()) << "e2ee.js did not open every message";
  for (std::size_t i = 0; i < texts.size(); ++i)
    EXPECT_EQ(str(opened[i]), texts[i]) << "e2ee.js opened message " << i << " wrong";
  const auto &replies = items(at(r, "replies"));
  ASSERT_FALSE(replies.empty());
  for (const se::value_t &m : replies) {
    const E2eeOpened o = carl.open_message("carl", "77", message_of(at(m, "message")));
    EXPECT_EQ(o.status, E2eeOpened::Status::ok) << o.error;
    EXPECT_EQ(o.text, str(at(m, "text")));
    std::printf("opened e2ee.js's reply #%lld: %s\n",
                static_cast<long long>(num(at(at(m, "message"), "id"))), o.text.c_str());
  }
}

} // namespace

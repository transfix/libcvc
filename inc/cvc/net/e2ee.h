#ifndef CVC_NET_E2EE_H
#define CVC_NET_E2EE_H

// cvc::net — end-to-end encrypted direct messages, with the keys kept by the host (docs/E2EE.md,
// docs/ARIADNE.md §10.8).
//
// An E2eeStore is TX.WTF's client crypto (atx-crypto-club/txwtf txwtf/static/js/e2ee.js and
// messaging-e2ee.js) ported to C++ over libsodium, byte for byte: X3DH to set a session up, a
// Double Ratchet per message (X25519, BLAKE2b, XChaCha20-Poly1305), a random content key per
// message wrapped through the ratchet, and an Argon2id passphrase backup of the identity. What it
// seals, TX.WTF's web client opens, and the reverse; src/cvc/tests/e2ee_test.cpp replays vectors
// e2ee.js wrote (tools/e2ee/txwtf_vectors.js) and compares every byte.
//
// The store holds every secret: identities, prekeys, ratchet sessions, and a cache of plaintext
// it already opened (a ratchet key opens a message once). Its API takes and gives public keys,
// ciphertext and plaintext only, so a document using it through the e2ee-* verbs
// (cvc/ariadne/e2ee_intrinsics.h) never holds a key, and no error quotes one.
//
// Everything is per handle: a name like a credential's (1-64 of [A-Za-z0-9._-]), one identity
// each, with its sessions by conversation. With a file, the store keeps it all across runs:
// owner-only (0600 in a 0700 directory on POSIX; a Windows profile directory is the user's own),
// replaced whole on every change, before the call that changed it returns, and locked to one
// process (a second store on the same file does not open). Without a file it lives in memory.
//
// Built without libsodium (CVC_HAVE_SODIUM), e2ee_available() is false and every operation fails
// saying so.
//
// Thread-safe.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
namespace net {

// Whether this build has the E2EE code (libsodium was found).
bool e2ee_available();

// Where the store's randomness comes from: the system's (libsodium's randombytes) by default. A
// test hands in e2ee_seeded_random to replay e2ee.js's vectors.
class E2eeRandom {
public:
  virtual ~E2eeRandom() = default;
  virtual void fill(unsigned char *out, std::size_t n) = 0;
  // A uniform value below `upper` (> 0).
  virtual std::uint32_t uniform(std::uint32_t upper) = 0;
  // Whether key pairs come from 32-byte seeds this stream draws (crypto_*_seed_keypair), as the
  // vectors' stream does; otherwise libsodium makes them (crypto_*_keypair).
  virtual bool seeds_keypairs() const { return false; }
};
std::shared_ptr<E2eeRandom> e2ee_system_random();
// The vectors' stream (tools/e2ee/txwtf_vectors.js): 64-byte blocks BLAKE2b-512(key = seed,
// message = block index, 8 bytes little-endian), in order; uniform(n) is the next 4 bytes as a
// little-endian uint32, mod n; key pairs from the next 32 bytes. For tests: never for real keys.
std::shared_ptr<E2eeRandom> e2ee_seeded_random(const std::string &seed);

// A one-time prekey's public half (base64, libsodium's ORIGINAL variant, as everywhere here).
struct E2eePreKey {
  std::int64_t key_id = 0;
  std::string public_key;
};

// A peer's X3DH prekey bundle, as GET /e2ee/keys/{username} returns it.
struct E2eeBundle {
  std::string identity_key_ed25519;
  std::string identity_key_x25519;
  std::int64_t registration_id = 0;
  std::int64_t signed_prekey_id = 0;
  std::string signed_prekey;           // public key
  std::string signed_prekey_signature; // by identity_key_ed25519
  std::optional<E2eePreKey> one_time_prekey;
};

// A handle's public keys: the POST /e2ee/keys body, with the unused one-time prekeys.
struct E2eePublicKeys {
  std::string identity_key_ed25519;
  std::string identity_key_x25519;
  std::int64_t registration_id = 0;
  std::int64_t signed_prekey_id = 0;
  std::string signed_prekey;
  std::string signed_prekey_signature;
  std::vector<E2eePreKey> one_time_prekeys;
  std::string fingerprint; // e2ee_fingerprint(identity_key_ed25519)
};

// One encrypted message as GET /e2ee/dm/conversations/{id}/messages lists it for the caller.
struct E2eeMessage {
  std::int64_t id = 0; // 0: none (nothing is cached for it)
  std::int64_t sender_id = 0;
  std::string algorithm = "dr-v1"; // or "grp-dr-v1"
  std::optional<std::string> envelope;
  std::optional<std::string> ciphertext;
  std::optional<std::string> wrapped_key;
  std::optional<std::string> header;
  bool is_sender = false;
  bool locked = false;
};

// What seal() makes: the POST /e2ee/dm/conversations/{id}/messages fields for a 1:1 message
// (recipient_keys = [{recipient, wrapped_key, header}]).
struct E2eeSealed {
  bool ok = false;
  bool need_bundle = false; // no session yet: call again with the peer's bundle
  std::string error;
  std::string ciphertext;
  std::string wrapped_key;
  std::string header;
  std::optional<std::string> envelope; // the X3DH envelope, until the peer has replied
  std::string algorithm;               // "dr-v1"
};

struct E2eeRecipientKey {
  std::int64_t recipient = 0;
  std::string wrapped_key;
  std::string header;
};

// A group member to wrap for: its user id, and its bundle when the store has no session with it.
struct E2eeGroupMember {
  std::int64_t user_id = 0;
  std::optional<E2eeBundle> bundle;
};

// What seal_group() makes: one body and a wrapped key per member ("grp-dr-v1").
struct E2eeGroupSealed {
  bool ok = false;
  std::vector<std::int64_t> need_bundles; // members with no session and no bundle given
  std::string error;
  std::string ciphertext;
  std::vector<E2eeRecipientKey> recipient_keys;
  std::string algorithm; // "grp-dr-v1"
};

// What open() made of a message.
struct E2eeOpened {
  enum class Status {
    ok,         // opened now
    cached,     // opened before (or our own send, kept by remember())
    own,        // our own send, not kept here: nothing to open it with
    locked,     // paid and not unlocked, or no ciphertext or key for us
    no_session, // no session for it and no envelope: its keys are on another device
    failed,     // it does not open (tampered, out of the session's window, keys we lack)
  };
  Status status = Status::failed;
  std::string text;
  std::string attachments = "[]"; // the payload's attachments, as JSON
  std::string error;              // why, for failed and no_session; never quotes a key
};
const char *to_string(E2eeOpened::Status s);

// A passphrase backup: the PUT /e2ee/backup body, as GET /e2ee/backup returns it.
struct E2eeBackup {
  std::string ciphertext;
  std::string kdf = "argon2id";
  std::string kdf_salt;
  std::int64_t opslimit = 0;
  std::int64_t memlimit = 0;
  std::string alg = "argon2id13";
  std::string nonce;
  std::int64_t version = 1;
};

// A fingerprint people can compare: 40 hex digits in groups of 5 (BLAKE2b-160 of the decoded
// key). "" when `key_b64` is not base64.
std::string e2ee_fingerprint(const std::string &key_b64);

class E2eeStore {
public:
  struct Options {
    std::string path;                    // "" keeps everything in memory
    std::shared_ptr<E2eeRandom> random;  // null: e2ee_system_random()
    std::size_t max_cached = 10000;      // plaintext cache entries per handle (oldest go first)
    std::size_t max_skipped = 2000;      // skipped message keys per session (oldest go first)
    std::int64_t max_opslimit = 10;      // the most a backup may ask Argon2id for
    std::int64_t max_memlimit = 1 << 30; // ... in bytes
  };

  explicit E2eeStore(Options options);
  ~E2eeStore();
  E2eeStore(const E2eeStore &) = delete;
  E2eeStore &operator=(const E2eeStore &) = delete;

  // Read the file, and lock it to this process until the store goes. A missing file holds
  // nothing. False, with `error` set and nothing loaded, when another process holds the lock,
  // the file cannot be read or is malformed, or (on POSIX) it belongs to another user or is open
  // to others. A memory store opens at once.
  bool open(std::string *error = nullptr);
  const std::string &path() const { return options_.path; }
  bool persistent() const { return !options_.path.empty(); }

  // A handle (credential-style name: 1-64 of [A-Za-z0-9._-]); a conversation (1-64 of
  // [A-Za-z0-9_-], a TX.WTF conversation id).
  static bool valid_handle(const std::string &handle);
  static bool valid_conversation(const std::string &conv);
  // Where --e2ee-store SPEC puts the file: a plain name (valid_handle) is a file of that name in
  // the user's config directory (%LOCALAPPDATA%\cvc\e2ee on Windows, else $XDG_CONFIG_HOME/cvc/e2ee
  // or ~/.config/cvc/e2ee); anything else is a path. "" when no config directory can be named
  // (or in a browser).
  static std::string resolve_path(const std::string &spec);

  std::vector<std::string> handles() const;
  bool has_identity(const std::string &handle) const;
  // The handle's public keys, or nullopt when it has no identity.
  std::optional<E2eePublicKeys> public_keys(const std::string &handle) const;
  // Make an identity for `handle` with `one_time_prekeys` prekeys (e2ee.js's generateIdentity,
  // generateSignedPreKey(1), generateOneTimePreKeys(1, N)). Fails when it has one already.
  std::optional<E2eePublicKeys> create_identity(const std::string &handle, int one_time_prekeys,
                                                std::string *error = nullptr);
  // `count` more one-time prekeys, numbered after the last one made: the POST /e2ee/keys/one-time
  // body.
  std::optional<std::vector<E2eePreKey>> add_prekeys(const std::string &handle, int count,
                                                     std::string *error = nullptr);
  // Drop the handle's identity, sessions and cached plaintext. True if it had any.
  bool forget(const std::string &handle, std::string *error = nullptr);

  bool has_session(const std::string &handle, const std::string &conv) const;
  // Seal `text` for the 1:1 conversation `conv` (messaging-e2ee.js's encryptAndSend). With no
  // session yet it needs the peer's bundle: without one the result has need_bundle.
  E2eeSealed seal(const std::string &handle, const std::string &conv, const std::string &text,
                  const std::optional<E2eeBundle> &bundle = std::nullopt);
  // Seal `text` once and wrap its key for each member (encryptAndSendGroup).
  E2eeGroupSealed seal_group(const std::string &handle, const std::string &conv,
                             const std::string &text, const std::vector<E2eeGroupMember> &members);
  // Keep the plaintext of our own send under the id the server gave it.
  bool remember(const std::string &handle, std::int64_t message_id, const std::string &text,
                std::string *error = nullptr);
  // Open one message of `conv` (decryptIncoming / decryptIncomingGroup). Messages of a
  // conversation must come in the order they were sent, as for e2ee.js.
  E2eeOpened open_message(const std::string &handle, const std::string &conv,
                          const E2eeMessage &message);

  // A passphrase backup of the handle's identity and signed prekey (wrapPrivateBundle of
  // exportSecretBundle, Argon2id at libsodium's MODERATE limits). Slow by design: about a second
  // and 256 MiB.
  std::optional<E2eeBackup> backup(const std::string &handle, const std::string &passphrase,
                                   std::string *error = nullptr);
  // Take the identity from a backup for a handle that has none (unwrapPrivateBundle). One-time
  // prekeys are not in a backup; those made afterwards are numbered from 100001.
  std::optional<E2eePublicKeys> restore(const std::string &handle, const E2eeBackup &blob,
                                        const std::string &passphrase,
                                        std::string *error = nullptr);

  // The handle's state in messaging-e2ee.js's key-store layout ("identity", "session:<ref>",
  // "pt:<id>" -> value), as one JSON object: what the tests compare with e2ee.js's store. Holds
  // the handle's secrets; never for a document.
  std::string export_state(const std::string &handle) const;
  // Load such a state for a handle with no identity (e2ee.js's store, for the tests).
  bool import_state(const std::string &handle, const std::string &json,
                    std::string *error = nullptr);

  struct Impl;

private:
  Options options_;
  mutable std::mutex mutex_;
  std::unique_ptr<Impl> impl_;
};

} // namespace net
} // namespace cvc

#endif // CVC_NET_E2EE_H

# End-to-end encrypted DMs with the keys in the host

`cvc::net::E2eeStore` (`inc/cvc/net/e2ee.h`) is TX.WTF's client crypto ported to C++ over
libsodium, and the `e2ee-*` Ariadne verbs (`inc/cvc/ariadne/e2ee_intrinsics.h`, ARIADNE.md §10.8)
put it in front of documents.

- **The source.** TX.WTF is atx-crypto-club/txwtf, and its client crypto is
  `txwtf/static/js/e2ee.js` and `messaging-e2ee.js`.
- **Who holds the keys.** A document such as nicotina, a TX.WTF client written in Ariadne, seals and
  opens a conversation's messages by naming an account's handle. The host keeps every key.
- **What it is built on.** libsodium comes from the cvcpkg `libsodium` package (ISC).
- **Without libsodium.** `e2ee_available()` is false and the verbs are absent.

## What it is

TX.WTF's scheme, byte for byte:

- **Identity.** Each account has an Ed25519 key, an X25519 key, a registration id, a signed
  prekey, and one-time prekeys.
- **Session setup.** X3DH, from the peer's bundle.
- **Per message.** A Double Ratchet: X25519 for the DH steps, BLAKE2b for the root and chain KDFs,
  XChaCha20-Poly1305 for the AEAD.
- **Body and key.** Each message's body is encrypted once under a random content key. That key is
  wrapped through the ratchet, with the body as associated data.
- **Groups (`grp-dr-v1`).** The same content key is wrapped through each member's pairwise session.
- **Backup.** An Argon2id passphrase backup of the identity and signed prekey, sealed with
  `crypto_secretbox`.

| what | e2ee.js | here |
|---|---|---|
| base64 | libsodium ORIGINAL (standard alphabet, padded) | the same, everywhere |
| X3DH shared key | `generichash(32, DH1‖DH2‖DH3[‖DH4], key="txwtf-e2ee-x3dh-v1")` | `x3dh_initiate` / `x3dh_respond` |
| KDF_RK(rk, dh) | `generichash(64, dh, key=rk)`, split 32/32 | `kdf_rk` |
| KDF_CK(ck) | mk = `generichash(32, 0x01, key=ck)`, ck' = `generichash(32, 0x02, key=ck)` | `kdf_ck` |
| ratchet header, AD | `JSON.stringify({dh, pn, n})`; AD = caller's ‖ header | `header_json`, written in that key order |
| sealed box | b64(nonce24 ‖ XChaCha20-Poly1305) | `seal_box` / `open_box` |
| payload | `{"v":1,"text":...,"attachments":[]}` | `encode_payload`; open takes any JSON with `text`, else the bytes as text |
| X3DH envelope | b64(JSON `{identity_key_ed25519, identity_key_x25519, ephemeral_key, spk_id, opk_id}`), sent until the peer replies | the same |
| sessions | by conversation id; a group's by `<conv>:<sender id>` | the same |
| skipped keys | at most 1000 ahead, keyed `"<b64 dh>:<n>"` | the same, and at most `max_skipped` (2000) kept per session |
| backup | Argon2id at MODERATE, then secretbox of `JSON.stringify(exportSecretBundle)` | the same; restore caps the blob's limits |

The store keeps its state in messaging-e2ee.js's key-store layout (`identity`, `session:<ref>`,
`pt:<message id>`), which is what makes the tests below possible.

## Where it is safer, still on the same wire

- **Decrypting is transactional.** It works on a copy of the session and keeps it only once the
  message has opened. e2ee.js runs the DH ratchet before the AEAD has authenticated anything. So a
  forged message carrying a fresh `dh` breaks its session, and the server is untrusted by TX.WTF's
  own threat model. Every later genuine message in that page load then fails too. On each reload the
  forged message is replayed first, so the conversation stays unreadable.
- **A skipped key is spent only when it worked.** e2ee.js deletes it before trying it.
- **Headers are checked before anything is derived.** The `dh` must be 32 bytes, and the counters
  whole numbers below 2^31.
- **What it keeps is bounded.** Skipped keys and cached plaintext have limits, and so do a backup's
  Argon2id limits (`Options::max_opslimit`, `max_memlimit`). A hostile server could otherwise ask
  for any amount of memory.
- **Errors never quote key material.**

The first two are worth reporting to txwtf, but nothing here depends on that.

## Key storage, per host

| host | the keys | |
|---|---|---|
| `ari` (FTXUI, tvision, over SSH), `arigl` native | `--e2ee-store SPEC` | A plain name is `<config>/cvc/e2ee/NAME`, as `--credential-store` resolves one: `%LOCALAPPDATA%\cvc` on Windows, else `$XDG_CONFIG_HOME/cvc` or `~/.config/cvc`. Anything else is a path. |
| `arisrv`, server documents | never | Keys on a relay would defeat the encryption. |
| `arigl-web` | not yet | See below. |

The file:
- **Owner-only:** 0600 in a 0700 directory on POSIX, and refused if another user owns it or others
  can read it. On Windows it is in the user's profile.
- **Written whole on every change,** to a temporary file renamed over the old one, before the call
  that changed it returns. A message is never handed to the document from a ratchet state that
  isn't on disk.
- **Locked to one process** (`<file>.lock`, through flock or LockFileEx). A second host on the same
  file gets no verbs, and `(e2ee-status)` says "in use by another process". Two writers would fork
  the ratchet.
- **Its contents** are secrets in plaintext under those permissions, as TX.WTF's own IndexedDB store
  keeps them in a browser. Sealing the file with an OS key is a follow-up: DPAPI, Keychain, or
  libsecret.

The store is the host's object, not process-wide. `enable_net` opens it from
`NetHostConfig::e2ee_store` and keeps it in the app (`app.slot`), and the verbs' provider reads it
from there.

## In a browser

The crypto compiles for wasm and wasm-mt, against the libsodium bundles. There, randomness comes
from `getentropy()`, because libsodium's own JavaScript source is set up only on the thread that
called `randombytes_stir()`. On a pthread it throws `Module.getRandomValue is not a function`.

The store itself does not open in a browser yet, so `(has? "e2ee-seal")` is false there:
- **Why not in memory.** The ratchet would die with the page. The next send would then start an
  X3DH that the other side's existing session ignores, and that breaks their copy.
- **The next step.** Mount IDBFS at the store's directory and sync it after each write. The keys
  would then live in the page origin's IndexedDB, as tx.wtf's own page keeps them.

## Limits that come from TX.WTF's design

- **Sessions are per device.** The ratchet is not in the backup. A conversation whose session
  another device holds says so here: status `no-session`, and `own` for our sends from there. A
  document must not start a new session in it, because the other side would ignore that one and
  break.
- **One identity per account.** Publishing a new one wipes the account's prekeys at the instance,
  and the other device stops being reachable. A document compares `(e2ee-identity H)` with
  `GET /e2ee/keys/me`, then restores from the backup or asks before making new keys.
- **A restored identity lacks the other device's one-time prekeys.** A message whose envelope names
  one fails, and says which kind of key is missing.

## Tests

- **`e2ee_test` (`E2eeVectors.*`).** It replays `src/cvc/tests/data/e2ee/txwtf_vectors.json`.
  - **Where it comes from.** `tools/e2ee/txwtf_vectors.js` ran txwtf's unmodified e2ee.js and
    messaging-e2ee.js (libsodium-wrappers-sumo 0.7.15) for three users through a stand-in server.
    Every random draw came from a seeded BLAKE2b stream.
  - **What it covers.** Ten sends, twelve opens (out of order, across ratchet turns, with and
    without a one-time prekey, a group each way), two tampered copies, and a backup.
  - **What it asserts.** The store, drawing from the same streams, publishes the same bundles and
    seals the same bytes (bodies, wrapped keys, headers, envelopes, the backup blob). It opens
    every message, and ends holding what e2ee.js's stores hold.
- **`e2ee_test` (`E2eeStore.*`).**
  - a forged ratchet key changes nothing;
  - messages for another device say so;
  - too far ahead is refused;
  - the file is owner-only, locked, and kept across a restart.
- **`ariadne_e2ee_test`.**
  - the verbs, end to end in a document, with no key in the state tree;
  - a backup and a restore through the async verbs;
  - a store in use leaves the verbs out, and `(e2ee-status)` says why.
- **`tools/e2ee/interop.sh` (live, with the system's randomness).**
  1. This store seals three messages.
  2. e2ee.js opens them and replies twice (`tools/e2ee/txwtf_interop.js`).
  3. This store opens the replies.

#ifndef CVC_ARIADNE_E2EE_INTRINSICS_H
#define CVC_ARIADNE_E2EE_INTRINSICS_H

// Ariadne — end-to-end encrypted DMs, with the keys in the host (docs/ARIADNE.md §10.8,
// docs/E2EE.md).
//
// The verbs drive an E2eeStore (cvc/net/e2ee.h): TX.WTF's X3DH + Double Ratchet, byte-compatible
// with its web client. A document names a handle (like a credential, 1-64 of [A-Za-z0-9._-]) and a
// conversation, hands over what the server sent, and gets back what to send. It never holds a key:
// every dict below carries public keys, ciphertext and plaintext only. The verbs make no requests;
// the document sends what they give it with http-request (auth: names the account's token).
//
//   (e2ee-identity H)                the handle's public keys, or nil when it has no identity:
//                                    {identity_key_ed25519 identity_key_x25519 registration_id
//                                     fingerprint one_time_prekeys publish}, `publish` being the
//                                    POST /e2ee/keys body
//   (e2ee-create H [N])              make its identity, with N one-time prekeys (20); the same
//   dict.
//                                    An error when it has one already
//   (e2ee-prekeys H N)               N more one-time prekeys: the POST /e2ee/keys/one-time body
//   (e2ee-forget H)                  drop its identity, sessions and cached plaintext; #t if any
//   (e2ee-session? H CONV)           whether it holds a session for conversation CONV
//   (e2ee-seal H CONV PEER TEXT [BUNDLE])
//                                    seal TEXT for 1:1 conversation CONV with user id PEER:
//                                    {ok need_bundle error body}, `body` being the POST
//                                    /e2ee/dm/conversations/{CONV}/messages body. The first message
//                                    of a session needs BUNDLE, the GET /e2ee/keys/{username}
//                                    reply: without it, need_bundle is #t and nothing changed
//   (e2ee-seal-group H CONV TEXT MEMBERS)
//                                    seal TEXT once for a group, a key per member: MEMBERS a list
//                                    of user ids or (dict "user_id" ID "bundle" BUNDLE);
//                                    {ok need_bundles error body}, need_bundles the ids that have
//                                    no session here and came without a bundle
//   (e2ee-sent H MSGID TEXT)         keep the plaintext of our own send under the id the server
//                                    gave it (a sender cannot open what it sent)
//   (e2ee-open H CONV MESSAGE)       open one entry of GET /e2ee/dm/conversations/{CONV}/messages:
//                                    {ok status text attachments error}; status is ok, cached,
//                                    own (ours, not kept here), locked, no-session (its keys are
//                                    on another device) or failed. Open a conversation's messages
//                                    in the order they were sent (ascending id)
//   (e2ee-backup H PASSPHRASE)       the PUT /e2ee/backup body (Argon2id, ~1 s): {ok error body}
//   (e2ee-restore H BLOB PASSPHRASE) take a handle's identity from a GET /e2ee/backup reply:
//                                    {ok error identity}
//   (e2ee-fingerprint KEY)           a key people can compare (40 hex digits in groups of 5)
//   (e2ee-status)                    {ok persistent error}: whether the verbs above are here (a
//                                    host asked for them, but the store may not open: another
//                                    process holds it, no libsodium, a browser), and whether
//                                    the keys outlive this run
//
// e2ee-backup and e2ee-restore are async functions (they park the program, and have -async
// forms); the rest are plain. Use them in a program lane (on:, on_tick:, ...).

#include <memory>
#include <string>

namespace cvc {
class app;
namespace net {
class E2eeStore; // cvc/net/e2ee.h
} // namespace net
namespace ariadne {

// Register the verbs against `app` for `store` (opened). The store is kept in the app (its slot,
// cvc/core/app.h), so it lives as long as the app does. The first call appends to the app's
// action-intrinsic providers, so make it at setup, before load_*; a later call (or enable_e2ee)
// only swaps the store programs built afterwards use. Null `store`: nothing.
void register_e2ee_intrinsics(cvc::app &app, std::shared_ptr<cvc::net::E2eeStore> store);

// What --e2ee-store SPEC does (enable_net, net_host.h): open the store SPEC names
// (E2eeStore::resolve_path), lock it to this process, and register the verbs for it. When it
// cannot (no libsodium, a browser, another process holds it, an unreadable file), the verbs are
// left out and (e2ee-status) says why, which is also what this returns ("" when it opened). The
// host runs on either way. Again with the same file, nothing changes.
std::string enable_e2ee(cvc::app &app, const std::string &spec);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_E2EE_INTRINSICS_H

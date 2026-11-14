// The e2ee-* verbs (cvc/ariadne/e2ee_intrinsics.h): a document seals and opens TX.WTF DMs by
// handle, and nothing it can reach (its variables, the state tree, an error) ever holds a key.

#include <chrono>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h> // clear_action_intrinsics
#include <cvc/ariadne/e2ee_intrinsics.h>
#include <cvc/core/app.h>
#include <cvc/net/e2ee.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
using cvc::ariadne::AppRuntime;

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// Every value in the state tree under `root`, as text.
std::string all_values(cvc::state &node) {
  std::string out = node.value() + "\n";
  for (const std::string &c : node.children())
    out += c + "=" + all_values(node(c));
  return out;
}

// Run `program` as an action and drain until it ends (async verbs take a while: Argon2id).
std::string run(AppRuntime &art, const std::string &program, int seconds = 60) {
  bool ended = false;
  std::string how = "unset";
  art.run_program(program, [&](const std::string &h) {
    ended = true;
    how = h;
  });
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  while (!ended && std::chrono::steady_clock::now() < until) {
    art.drain();
    if (!ended)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return ended ? how : "timed out";
}

// The base64 secrets a store holds for `handle`: every value in its state that is not public.
std::vector<std::string> secrets_of(const cvc::net::E2eeStore &s, const std::string &handle) {
  std::vector<std::string> out;
  const std::string st = s.export_state(handle);
  for (const char *key : {"\"priv\":\"", "_priv\":\"", "\"RK\":\"", "\"CKs\":\"", "\"CKr\":\""}) {
    for (std::size_t at = st.find(key); at != std::string::npos; at = st.find(key, at + 1)) {
      const std::size_t b = at + std::string(key).size();
      out.push_back(st.substr(b, st.find('"', b) - b));
    }
  }
  return out;
}

// 3.x's action-intrinsic providers are the process's (4.0 gives each app its own): a test drops
// them before its apps go, as the other 3.x Ariadne tests do. Declared after the apps, so it runs
// first.
struct ClearProviders {
  ~ClearProviders() { cvc::ariadne::clear_action_intrinsics(); }
};

TEST(AriadneE2ee, ADocumentSealsAndOpensWithoutHoldingAKey) {
  if (!cvc::net::e2ee_available())
    GTEST_SKIP() << "libcvc built without libsodium";
  cvc::app app;
  ClearProviders clear_providers;
  auto store = std::make_shared<cvc::net::E2eeStore>(cvc::net::E2eeStore::Options{});
  ASSERT_TRUE(store->open());
  cvc::ariadne::register_e2ee_intrinsics(app, store);
  AppRuntime art(app, "ui.dm");

  // Two accounts in one host: alice (user 1) writes to bob (user 2) in conversation 42.
  EXPECT_EQ(run(art, R"((begin
    (state-set "status" (get-attr (e2ee-status) "ok"))
    (state-set "none" (if (e2ee-identity "alice") "some" "nil"))
    (set a (e2ee-create "alice" 2))
    (set b (e2ee-create "bob" 2))
    (state-set "fp" (get-attr a "fingerprint"))
    (state-set "opks" (get-attr b "one_time_prekeys"))
    (state-set "again" (try (e2ee-create "alice") (catch (e) (get-attr e "message"))))
    ; the first seal asks for bob's bundle, as GET /e2ee/keys/bob would give it
    (set s0 (e2ee-seal "alice" 42 2 "hi bob"))
    (state-set "need" (get-attr s0 "need_bundle"))
    (set pub (get-attr b "publish"))
    (set bundle (dict "identity_key_ed25519" (get-attr pub "identity_key_ed25519")
                      "identity_key_x25519" (get-attr pub "identity_key_x25519")
                      "registration_id" (get-attr pub "registration_id")
                      "signed_prekey" (get-attr pub "signed_prekey")
                      "one_time_prekey" (car (get-attr pub "one_time_prekeys"))))
    (set s (e2ee-seal "alice" 42 2 "hi bob" bundle))
    (state-set "sealed" (get-attr s "ok"))
    (set body (get-attr s "body"))
    (state-set "algorithm" (get-attr body "algorithm"))
    (state-set "recipient" (get-attr (car (get-attr body "recipient_keys")) "recipient"))
    (e2ee-sent "alice" 501 "hi bob")
    ; bob opens it as GET .../messages lists it
    (set k (car (get-attr body "recipient_keys")))
    (set m (dict "id" 501 "sender_id" 1 "algorithm" "dr-v1" "envelope" (get-attr body "envelope")
                 "ciphertext" (get-attr body "ciphertext") "wrapped_key" (get-attr k "wrapped_key")
                 "header" (get-attr k "header") "is_sender" #f "locked" #f))
    (set o (e2ee-open "bob" 42 m))
    (state-set "opened" (get-attr o "status"))
    (state-set "text" (get-attr o "text"))
    (state-set "again2" (get-attr (e2ee-open "bob" 42 m) "status"))
    (state-set "mine" (get-attr (e2ee-open "alice" 42 (dict "id" 501 "is_sender" #t)) "text"))
    (state-set "session" (e2ee-session? "bob" 42))
    ; tampered: refused, and says why without a byte of key
    (set tm (dict "id" 502 "sender_id" 1 "algorithm" "dr-v1" "ciphertext" (get-attr body "ciphertext")
                 "wrapped_key" "AAAA" "header" (get-attr k "header")))
    (state-set "tampered" (get-attr (e2ee-open "bob" 42 tm) "status"))
    (state-set "why" (get-attr (e2ee-open "bob" 42 tm) "error"))
    (state-set "prekeys" (length (get-attr (e2ee-prekeys "bob" 3) "one_time_prekeys")))
    (state-set "done" 1)))"),
            "");
  EXPECT_EQ(val(app, "ui.dm.status"), "true");
  EXPECT_EQ(val(app, "ui.dm.none"), "nil");
  EXPECT_EQ(val(app, "ui.dm.fp").size(), 47u) << val(app, "ui.dm.fp");
  EXPECT_EQ(val(app, "ui.dm.opks"), "2");
  EXPECT_NE(val(app, "ui.dm.again").find("has an identity already"), std::string::npos)
      << val(app, "ui.dm.again");
  EXPECT_EQ(val(app, "ui.dm.need"), "true");
  EXPECT_EQ(val(app, "ui.dm.sealed"), "true");
  EXPECT_EQ(val(app, "ui.dm.algorithm"), "dr-v1");
  EXPECT_EQ(val(app, "ui.dm.recipient"), "2");
  EXPECT_EQ(val(app, "ui.dm.opened"), "ok");
  EXPECT_EQ(val(app, "ui.dm.text"), "hi bob");
  EXPECT_EQ(val(app, "ui.dm.again2"), "cached");
  EXPECT_EQ(val(app, "ui.dm.mine"), "hi bob");
  EXPECT_EQ(val(app, "ui.dm.session"), "true");
  EXPECT_EQ(val(app, "ui.dm.tampered"), "failed");
  EXPECT_EQ(val(app, "ui.dm.prekeys"), "3");

  const std::string state = all_values(cvc::state::instance(app));
  std::vector<std::string> secrets = secrets_of(*store, "alice");
  for (const std::string &s : secrets_of(*store, "bob"))
    secrets.push_back(s);
  ASSERT_GE(secrets.size(), 8u);
  for (const std::string &s : secrets)
    EXPECT_EQ(state.find(s), std::string::npos) << "a secret reached the state tree";
}

// A passphrase backup through the async verbs: the PUT /e2ee/backup body, restored under another
// handle (another device's store, here the same one).
TEST(AriadneE2ee, BackupAndRestoreThroughTheVerbs) {
  if (!cvc::net::e2ee_available())
    GTEST_SKIP() << "libcvc built without libsodium";
  cvc::app app;
  ClearProviders clear_providers;
  auto store = std::make_shared<cvc::net::E2eeStore>(cvc::net::E2eeStore::Options{});
  ASSERT_TRUE(store->open());
  cvc::ariadne::register_e2ee_intrinsics(app, store);
  AppRuntime art(app, "ui.dm");
  EXPECT_EQ(run(art, R"((begin
    (set a (e2ee-create "me" 1))
    (set b (e2ee-backup "me" "a long passphrase"))
    (state-set "ok" (get-attr b "ok"))
    (set body (get-attr b "body"))
    (state-set "kdf" (get-attr body "kdf"))
    (state-set "alg" (get-attr (get-attr body "kdf_params") "alg"))
    (state-set "wrong" (get-attr (e2ee-restore "other" body "nope") "error"))
    (set r (e2ee-restore "other" body "a long passphrase"))
    (state-set "restored" (get-attr r "ok"))
    (state-set "same" (= (get-attr (get-attr r "identity") "identity_key_ed25519")
                         (get-attr a "identity_key_ed25519")))
    (state-set "done" 1)))"),
            "");
  EXPECT_EQ(val(app, "ui.dm.ok"), "true");
  EXPECT_EQ(val(app, "ui.dm.kdf"), "argon2id");
  EXPECT_EQ(val(app, "ui.dm.alg"), "argon2id13");
  EXPECT_NE(val(app, "ui.dm.wrong").find("incorrect passphrase"), std::string::npos)
      << val(app, "ui.dm.wrong");
  EXPECT_EQ(val(app, "ui.dm.restored"), "true");
  EXPECT_EQ(val(app, "ui.dm.same"), "true");
}

// --e2ee-store's path: a second process (here a second app) on the same file gets no verbs, and
// (e2ee-status) says why; the first keeps working.
TEST(AriadneE2ee, AStoreInUseLeavesTheVerbsOutAndSaysWhy) {
  if (!cvc::net::e2ee_available())
    GTEST_SKIP() << "libcvc built without libsodium";
  const fs::path dir = fs::temp_directory_path() / "ariadne_e2ee_test_lock";
  fs::remove_all(dir);
  const std::string spec = (dir / "store").string();
  cvc::app one, two;
  ClearProviders clear_providers;
  EXPECT_EQ(cvc::ariadne::enable_e2ee(one, spec), "");
  EXPECT_EQ(cvc::ariadne::enable_e2ee(one, spec), "") << "again with the same file";
  const std::string busy = cvc::ariadne::enable_e2ee(two, spec);
  EXPECT_NE(busy.find("in use by another process"), std::string::npos) << busy;
  {
    AppRuntime art(two, "ui.x");
    EXPECT_EQ(run(art, R"((begin (state-set "ok" (get-attr (e2ee-status) "ok"))
                                (state-set "why" (get-attr (e2ee-status) "error"))
                                (state-set "has" (has? "e2ee-seal")) (state-set "done" 1)))"),
              "");
    EXPECT_EQ(val(two, "ui.x.ok"), "false");
    EXPECT_NE(val(two, "ui.x.why").find("in use"), std::string::npos) << val(two, "ui.x.why");
    EXPECT_EQ(val(two, "ui.x.has"), "false");
  }
  {
    AppRuntime art(one, "ui.y");
    EXPECT_EQ(run(art, R"((begin (state-set "persistent" (get-attr (e2ee-status) "persistent"))
                                (e2ee-create "h" 0) (state-set "has" (has? "e2ee-seal"))
                                (state-set "done" 1)))"),
              "");
    EXPECT_EQ(val(one, "ui.y.persistent"), "true");
    EXPECT_EQ(val(one, "ui.y.has"), "true");
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

} // namespace

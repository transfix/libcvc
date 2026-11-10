// Ariadne's registries are each app's own (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3, P0a step 2):
// what one app registers, another app never sees, an app's entries go with it, and the order apps
// register and run in changes no answer. Each registry P0a moves onto the app adds its cases here.
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/conference/room_calls.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne;
namespace se = cvc::state_exec;

namespace {

// A provider that binds (NAME) to return VALUE.
ActionIntrinsicProvider verb(const std::string &name, const std::string &value) {
  return [name, value](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
    se::builtins::register_fn(env, name,
                              [value](std::span<const se::value_t>) { return se::value_t(value); });
  };
}

// What (NAME) gives in an init: program on `app`, or "error" when the program fails (the name is
// unbound there).
std::string call(cvc::app &app, const std::string &name) {
  std::vector<std::string> errs;
  if (!run_init(app, "t", "(state-set \"out\" (" + name + "))", &errs))
    return "error";
  return cvc::state::instance(app)("t.out").value();
}

// --- action intrinsics ---------------------------------------------------------------------

TEST(AriadneRegistries, ActionIntrinsicsAreEachAppsOwn) {
  cvc::app a, b;
  register_action_intrinsics(a, verb("who", "a"));
  register_action_intrinsics(b, verb("who", "b"));
  register_action_intrinsics(a, verb("only-a", "yes"));
  Runtime ra(a, ""), rb(b, "");
  EXPECT_TRUE(ra.has_function("only-a"));
  EXPECT_FALSE(rb.has_function("only-a")) << "b's documents see a's provider";
  EXPECT_EQ(call(a, "who"), "a");
  EXPECT_EQ(call(b, "who"), "b");
  EXPECT_EQ(call(a, "only-a"), "yes");
  EXPECT_EQ(call(b, "only-a"), "error");

  clear_action_intrinsics(a);
  EXPECT_FALSE(ra.has_function("only-a"));
  EXPECT_EQ(call(a, "who"), "error");
  EXPECT_EQ(call(b, "who"), "b") << "clearing a's providers took b's";
  EXPECT_TRUE(rb.has_function("who"));
}

TEST(AriadneRegistries, AnAppsProvidersGoWithIt) {
  // The provider captures its app. Once that app is gone no other app's program reaches it, not
  // even an app made at the address it had.
  cvc::app b;
  {
    cvc::app a;
    register_action_intrinsics(
        a, [&a](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
          se::builtins::register_fn(env, "a-value", [&a](std::span<const se::value_t>) {
            return se::value_t(cvc::state::instance(a)("x").value());
          });
        });
    EXPECT_EQ(call(a, "a-value"), "");
  }
  cvc::app c;
  EXPECT_EQ(call(b, "a-value"), "error");
  EXPECT_EQ(call(c, "a-value"), "error");
}

TEST(AriadneRegistries, AProviderUnderAKeyIsKeptOnce) {
  cvc::app app;
  int built = 0;
  const auto counting = [&built](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
    ++built;
    se::builtins::register_fn(
        env, "k", [](std::span<const se::value_t>) { return se::value_t(std::string("k")); });
  };
  register_action_intrinsics(app, verb("first", "1"));
  for (int i = 0; i < 3; ++i) // a host that starts three times on one app
    register_action_intrinsics(app, "test.keyed", counting);
  register_action_intrinsics(app, verb("plain", "p")); // no key: appends
  ASSERT_EQ(call(app, "k"), "k");
  EXPECT_EQ(built, 1) << "one program env ran the keyed provider " << built << " times";

  // Again under the same key: it replaces the earlier one, in its place (after "first", before
  // "plain"), so a later provider still overrides it.
  register_action_intrinsics(app, "test.keyed", verb("k", "new"));
  register_action_intrinsics(app, verb("plain", "p2"));
  EXPECT_EQ(call(app, "k"), "new");
  EXPECT_EQ(call(app, "plain"), "p2");
  EXPECT_EQ(call(app, "first"), "1");
}

TEST(AriadneRegistries, ActionIntrinsicAnswersDoNotDependOnOrder) {
  // Two apps registering and running in either order, with or without a third app made and
  // destroyed in between, give the same answers.
  const auto answers = [](bool a_first, bool churn) {
    cvc::app a, b;
    const auto reg_a = [&a] { register_action_intrinsics(a, verb("who", "a")); };
    const auto reg_b = [&b] {
      register_action_intrinsics(b, verb("who", "b"));
      register_action_intrinsics(b, verb("extra", "x"));
    };
    if (a_first) {
      reg_a();
      reg_b();
    } else {
      reg_b();
      reg_a();
    }
    if (churn) {
      cvc::app tmp;
      register_action_intrinsics(tmp, verb("who", "tmp"));
      register_action_intrinsics(tmp, "test.extra", verb("extra", "tmp"));
      EXPECT_EQ(call(tmp, "who"), "tmp");
    }
    std::map<std::string, std::string> out;
    if (a_first) {
      out["who a"] = call(a, "who");
      out["extra a"] = call(a, "extra");
    }
    out["who b"] = call(b, "who");
    out["extra b"] = call(b, "extra");
    if (!a_first) {
      out["who a"] = call(a, "who");
      out["extra a"] = call(a, "extra");
    }
    return out;
  };
  const std::map<std::string, std::string> want = {
      {"who a", "a"}, {"extra a", "error"}, {"who b", "b"}, {"extra b", "x"}};
  EXPECT_EQ(answers(true, false), want);
  EXPECT_EQ(answers(false, false), want);
  EXPECT_EQ(answers(true, true), want);
  EXPECT_EQ(answers(false, true), want);
}

TEST(AriadneRegistries, AppsOnOtherThreadsKeepTheirOwnProviders) {
  constexpr int kApps = 4;
  std::vector<std::string> got(kApps);
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &got] {
      cvc::app app;
      for (int round = 0; round < 3; ++round)
        register_action_intrinsics(app, "test.who", verb("who", std::to_string(i)));
      got[i] = call(app, "who");
    });
  for (std::thread &t : threads)
    t.join();
  for (int i = 0; i < kApps; ++i)
    EXPECT_EQ(got[i], std::to_string(i));
}

// --- the room-call client a host installs ---------------------------------------------------

using conference::room_call_client;

TEST(AriadneRegistries, ARoomCallClientIsItsAppsOwn) {
  cvc::app host;
  cvc::state_cluster_shard shard(host, "conf", "node");
  const auto ca = std::make_shared<room_call_client>(shard, "room", "a");
  const auto cb = std::make_shared<room_call_client>(shard, "room", "b");
  // Installed in either order, each app finds its own and no other's.
  for (const bool aFirst : {true, false}) {
    cvc::app a, b;
    EXPECT_EQ(room_call_client::for_app(a), nullptr);
    if (aFirst) {
      room_call_client::install(a, ca);
      EXPECT_EQ(room_call_client::for_app(b), nullptr);
      room_call_client::install(b, cb);
    } else {
      room_call_client::install(b, cb);
      EXPECT_EQ(room_call_client::for_app(a), nullptr);
      room_call_client::install(a, ca);
    }
    EXPECT_EQ(room_call_client::for_app(a), ca);
    EXPECT_EQ(room_call_client::for_app(b), cb);
    room_call_client::install(a, nullptr);
    EXPECT_EQ(room_call_client::for_app(a), nullptr);
    EXPECT_EQ(room_call_client::for_app(b), cb) << "taking a's away took b's";
  }
}

TEST(AriadneRegistries, AnInstalledRoomCallClientGoesWithItsApp) {
  cvc::app host;
  cvc::state_cluster_shard shard(host, "conf", "node");
  std::weak_ptr<room_call_client> kept;
  {
    cvc::app gone;
    auto client = std::make_shared<room_call_client>(shard, "room", "pid");
    kept = client;
    room_call_client::install(gone, std::move(client));
    EXPECT_FALSE(kept.expired());
  }
  EXPECT_TRUE(kept.expired()) << "the app held its client past its own end";
  cvc::app after;
  EXPECT_EQ(room_call_client::for_app(after), nullptr);
}

} // namespace

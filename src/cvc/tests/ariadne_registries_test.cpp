// Ariadne's registries are each app's own (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3, P0a step 2):
// what one app registers, another app never sees, an app's entries go with it, and the order apps
// register and run in changes no answer. Each registry P0a moves onto the app adds its cases here.
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/conference/room_calls.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/script_engine.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_state.h>
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

// --- the document registries: widget types, .ari blocks, data decoders ----------------------

// Whether `warnings` has one containing `needle`.
bool warned(const std::vector<std::string> &warnings, const std::string &needle) {
  for (const std::string &w : warnings)
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

// A document that uses widget type `widget` and block `block`, and asks for decoder `decoder`.
std::string document_using(const std::string &widget, const std::string &block,
                           const std::string &decoder) {
  return "meta: { min_libcvc: \"0.0.0\" }\n"
         "windows: [ { window: W, children: [ { type: " +
         widget + " } ] } ]\n" + block + ": { name: here }\n" +
         "data:\n  d: { uri: \"file:x.bin\", as: " + decoder + " }\n";
}

// What `app` makes of document_using(widget, block, decoder): which of the three it knew.
std::string what_app_knows(cvc::app &app, const std::string &widget, const std::string &block,
                           const std::string &decoder) {
  const LoadResult r = load_string(app, document_using(widget, block, decoder));
  if (!r.ok)
    return "load failed: " + r.error;
  std::string out;
  out += warned(r.warnings, "widget type '" + widget + "'") ? "widget? " : "widget ";
  bool kept = false;
  for (const auto &kv : r.extras)
    kept = kept || kv.first == block;
  out += kept ? "block " : "block? ";
  out += warned(r.warnings, "as: " + decoder) ? "decoder?" : "decoder";
  return out;
}

void register_all(cvc::app &app, const std::string &tag) {
  register_widget_type(app, "w_" + tag, [](const Widget &, const WidgetEmitContext &) {});
  register_ari_block(app, "b_" + tag, [tag](const Value &content, LoadResult &out) {
    out.extras.emplace_back("b_" + tag, content);
  });
  register_data_decoder(app, "d_" + tag, [](DataDecode &) {});
}

TEST(AriadneRegistries, DocumentRegistriesAreEachAppsOwn) {
  if (!have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  for (const bool aFirst : {true, false}) {
    cvc::app a, b;
    if (aFirst) {
      register_all(a, "a");
      register_all(b, "b");
    } else {
      register_all(b, "b");
      register_all(a, "a");
    }
    EXPECT_EQ(what_app_knows(a, "w_a", "b_a", "d_a"), "widget block decoder");
    EXPECT_EQ(what_app_knows(a, "w_b", "b_b", "d_b"), "widget? block? decoder?");
    EXPECT_EQ(what_app_knows(b, "w_b", "b_b", "d_b"), "widget block decoder");
    EXPECT_EQ(what_app_knows(b, "w_a", "b_a", "d_a"), "widget? block? decoder?");
    EXPECT_TRUE(has_widget_type(a, "w_a"));
    EXPECT_FALSE(has_widget_type(b, "w_a"));
    EXPECT_TRUE(has_ari_block(a, "b_a"));
    EXPECT_FALSE(has_ari_block(b, "b_a"));
    EXPECT_TRUE(has_data_decoder(a, "d_a"));
    EXPECT_FALSE(has_data_decoder(b, "d_a"));
  }
}

TEST(AriadneRegistries, EveryAppStartsWithTheBuiltInDecodersOnly) {
  cvc::app a;
  register_data_decoder(a, "extra", [](DataDecode &) {});
  cvc::app b; // made after a registered one
  for (const char *as :
       {"files", "bytes", "text", "json", "mesh", "volume", "heightfield", "city_bundle"}) {
    EXPECT_TRUE(has_data_decoder(a, as)) << as;
    EXPECT_TRUE(has_data_decoder(b, as)) << as;
  }
  EXPECT_FALSE(has_data_decoder(b, "extra"));
  register_data_decoder(a, "json", nullptr); // removing a built-in from one app
  EXPECT_FALSE(has_data_decoder(a, "json"));
  EXPECT_TRUE(has_data_decoder(b, "json")) << "removing it from one app removed it from another";
}

TEST(AriadneRegistries, AppsOnOtherThreadsKeepTheirOwnDocumentRegistries) {
  if (!have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  constexpr int kApps = 4;
  std::vector<std::string> mine(kApps), others(kApps);
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &mine, &others] {
      cvc::app app;
      const std::string tag = std::to_string(i), next = std::to_string((i + 1) % kApps);
      register_all(app, tag);
      mine[i] = what_app_knows(app, "w_" + tag, "b_" + tag, "d_" + tag);
      others[i] = what_app_knows(app, "w_" + next, "b_" + next, "d_" + next);
    });
  for (std::thread &t : threads)
    t.join();
  for (int i = 0; i < kApps; ++i) {
    EXPECT_EQ(mine[i], "widget block decoder") << i;
    EXPECT_EQ(others[i], "widget? block? decoder?") << i;
  }
}

// --- the URI resolver, the net host ---------------------------------------------------------

// A handler for scheme "memo" that answers `answer`.
void register_memo(cvc::app &app, const std::string &answer) {
  register_uri_handler(app, "memo", [answer](const Uri &, const std::string &) {
    return UriResult{true, answer, "memo://" + answer, std::string()};
  });
}

TEST(AriadneRegistries, UriHandlersAndCapsAreEachAppsOwn) {
  for (const bool aFirst : {true, false}) {
    cvc::app a, b;
    if (aFirst)
      register_memo(a, "from a");
    EXPECT_FALSE(resolve(b, "memo://x").ok) << "b resolves a's scheme";
    register_memo(b, "from b");
    if (!aFirst)
      register_memo(a, "from a");
    EXPECT_EQ(resolve(a, "memo://x").content, "from a");
    EXPECT_EQ(resolve(b, "memo://x").content, "from b");
    unregister_uri_handler(a, "memo");
    EXPECT_FALSE(has_uri_handler(a, "memo"));
    EXPECT_TRUE(has_uri_handler(b, "memo")) << "unregistering a's took b's";

    set_resolve_file_byte_cap(a, 10);
    set_store_file_byte_cap(a, 20);
    EXPECT_EQ(resolve_file_byte_cap(a), 10u);
    EXPECT_EQ(store_file_byte_cap(a), 20u);
    EXPECT_EQ(resolve_file_byte_cap(b), 16u * 1024u * 1024u) << "a's cap moved b's";
    EXPECT_EQ(store_file_byte_cap(b), 16u * 1024u * 1024u);
  }
}

// state:// resolves against the app it is registered on. When the registry was the process's,
// the last app to register answered every app's state:// reads.
TEST(AriadneRegistries, StateUrisReadEachAppsOwnTree) {
  cvc::app a, b;
  register_state_uri_handler(a);
  register_state_uri_handler(b);
  cvc::state::instance(a)("doc.title").value(std::string("A"));
  cvc::state::instance(b)("doc.title").value(std::string("B"));
  EXPECT_EQ(resolve(a, "state://doc.title").content, "A");
  EXPECT_EQ(resolve(b, "state://doc.title").content, "B");
  ASSERT_TRUE(store(a, "state://doc.saved", "a's").ok);
  EXPECT_EQ(cvc::state::instance(a)("doc.saved").value(), "a's");
  EXPECT_EQ(cvc::state::instance(b).findDescendant("doc.saved"), nullptr) << "a's store wrote b";
  cvc::app c; // made after: no state:// handler of its own
  EXPECT_FALSE(resolve(c, "state://doc.title").ok);
}

TEST(AriadneRegistries, TheNetHostIsEachAppsOwn) {
  cvc::app a, b;
  NetHostConfig on;
  on.enabled = true;
  ASSERT_EQ(enable_net(a, on), "");
  EXPECT_TRUE(net_host_status(a).enabled);
  EXPECT_FALSE(net_host_status(b).enabled) << "turning a's network on turned b's on";
  EXPECT_TRUE(net_host_access(a).has_value());
  EXPECT_FALSE(net_host_access(b).has_value());
  disable_net(a);
  EXPECT_FALSE(net_host_status(a).enabled);
}

TEST(AriadneRegistries, AppsOnOtherThreadsResolveWithTheirOwnHandlers) {
  constexpr int kApps = 4;
  std::vector<std::string> got(kApps);
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &got] {
      cvc::app app;
      register_memo(app, "app " + std::to_string(i));
      register_state_uri_handler(app);
      cvc::state::instance(app)("who").value(std::to_string(i));
      got[i] = resolve(app, "memo://x").content + " / " + resolve(app, "state://who").content;
    });
  for (std::thread &t : threads)
    t.join();
  for (int i = 0; i < kApps; ++i)
    EXPECT_EQ(got[i], "app " + std::to_string(i) + " / " + std::to_string(i));
}

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

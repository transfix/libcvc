// Ariadne — a host turning the network on for its documents. See inc/cvc/ariadne/net_host.h.

#include <atomic>
#include <cstdlib>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/oidc_host.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/core/app.h>
#include <cvc/net/browser.h>
#include <cvc/net/credentials.h>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {

struct NetHost {
  std::shared_ptr<std::atomic<bool>> live;
  NetHostStatus status;
  NetAccess access;
};

// What enable_net turned on for an app.
struct net_host_slot {
  static constexpr const char *app_slot = "cvc.ariadne.net-host";
  std::mutex m;
  NetHost host;
};

} // namespace

std::string enable_net(cvc::app &app, const NetHostConfig &config) {
  if (!config.enabled)
    return std::string();
  auto policy = std::make_shared<cvc::net::NetPolicy>();
  std::string why;
  for (const std::string &spec : config.allow) {
    const std::optional<cvc::net::Origin> o = cvc::net::parse_origin_spec(spec, &why);
    if (!o)
      return "--net-allow: " + why;
    policy->allow(*o);
  }
  for (const std::string &file : config.policy_files)
    if (!cvc::net::load_net_policy_file(file, *policy, &why))
      return "--net-policy: " + why;
  for (const std::string &issuer : config.oidc.issuers)
    if (const std::string bad = policy->add_oidc_issuer(issuer); !bad.empty())
      return "--oidc-issuer: " + bad;
  for (const auto &[issuer, client] : config.oidc.clients)
    if (const std::string bad = policy->set_oidc_client(issuer, client); !bad.empty())
      return "--oidc-client " + issuer + "=" + client + ": " + bad;
  if (!config.oidc.redirect_uri.empty() && !cvc::net::parse_origin(config.oidc.redirect_uri, &why))
    return "--oidc-redirect-uri: " + why;
  // A browser page can always reach the origin it came from; that is where its documents and
  // their API usually live.
  if (const std::string page = cvc::net::page_origin(); !page.empty())
    policy->allow(*cvc::net::parse_origin_spec(page));
  for (const auto &[name, origins] : policy->credential_bindings()) {
    if (!cvc::net::CredentialStore::valid_name(name))
      return "--net-policy: '" + name + "' is not a credential name (1-64 of A-Z a-z 0-9 . _ -)";
    for (const cvc::net::Origin &o : origins)
      if (!policy->allows(o))
        return "--net-policy: credential " + name + " is bound to " + o.str() +
               ", which is not allowed (add an allow line or --net-allow " + o.str() + ")";
  }

  std::string path;
  if (!config.credential_store.empty()) {
#ifdef __EMSCRIPTEN__
    return "--credential-store: a browser host keeps credentials in memory only";
#else
    path = cvc::net::CredentialStore::resolve_path(config.credential_store);
    if (path.empty())
      return "--credential-store: no config directory to keep '" + config.credential_store +
             "' in (set XDG_CONFIG_HOME or HOME, or give a path)";
#endif
  }
  auto store = std::make_shared<cvc::net::CredentialStore>(path);
  for (const auto &[name, origins] : policy->credential_bindings())
    store->bind(name, origins);
  if (!store->load(&why))
    return "--credential-store: " + why;
  std::vector<std::string> from_env;
  for (const auto &[name, var] : config.credential_env) {
    const std::string flag = "--credential-env " + name + "=" + var;
#ifdef __EMSCRIPTEN__
    return flag + ": a browser has no environment to read a token from";
#else
    if (!policy->credential_bindings().count(name))
      return flag + ": bind " + name + " with a 'credential " + name +
             " ORIGIN...' line in a --net-policy file";
    const char *value = std::getenv(var.c_str());
    if (!value || !*value)
      continue; // nothing supplied: the document signs in its own way
    if (!store->provide(name, value, &why))
      return flag + ": " + var + " holds no usable token: " + why;
    from_env.push_back(name);
#endif
  }

  auto live = std::make_shared<std::atomic<bool>>(true);
  NetHostStatus status;
  status.enabled = true;
  status.auth = true;
  status.persist = store->persistent();
  status.credential_store = path;
  status.env_credentials = std::move(from_env);
  for (const cvc::net::Origin &o : policy->allowed())
    status.allowed.push_back(o.str());
  status.oidc = cvc::net::have_http_backend();
  for (const cvc::net::OidcIssuer &i : policy->oidc_issuers())
    status.oidc_issuers.push_back(i.issuer);

  disable_net(app); // a second enable replaces the first
  NetAccess access{policy, store, live, config.ws};
  register_net_intrinsics(app, access);
  register_http_uri_handler(app, policy);
  if (status.oidc) {
    enable_oidc(app, policy, store, config.oidc, live);
    status.oidc_flows = oidc_flows(app);
  }
  status.browser = cvc::net::browser_available(app);
  net_host_slot &s = app.slot<net_host_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  s.host = NetHost{std::move(live), std::move(status), std::move(access)};
  return std::string();
}

void disable_net(cvc::app &app) {
  net_host_slot &s = app.slot<net_host_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  NetHost &h = s.host;
  if (!h.live)
    return;
  h.live->store(false);
  disable_oidc(app);
  unregister_uri_handler(app, "http");
  unregister_uri_handler(app, "https");
  unregister_uri_store_handler(app, "http");
  unregister_uri_store_handler(app, "https");
  h = NetHost{};
}

NetHostStatus net_host_status(cvc::app &app) {
  net_host_slot &slot = app.slot<net_host_slot>();
  std::lock_guard<std::mutex> lock(slot.m);
  NetHostStatus s = slot.host.status;
  if (s.enabled)
    s.browser = cvc::net::browser_available(app); // a test's opener, --no-browser
  return s;
}

std::optional<NetAccess> net_host_access(cvc::app &app) {
  net_host_slot &s = app.slot<net_host_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  const NetHost &h = s.host;
  if (!h.live)
    return std::nullopt;
  return h.access;
}

} // namespace ariadne
} // namespace cvc

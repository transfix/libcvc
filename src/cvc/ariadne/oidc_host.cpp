// Ariadne — OAuth 2.0 / OpenID Connect sign-in for documents. See inc/cvc/ariadne/oidc_host.h and
// docs/ARIADNE.md §10.5.
//
// Every token stays in C++ here. A flow's secrets (the PKCE verifier, the authorization code, the
// device code) and the tokens a reply carries live in a Session or a request on its way, are
// wiped when done with, and reach the document only as the credential names it chose. What the
// document sees, the Status, holds no secret: the phase, the provider's error code and a printable
// description, a device flow's user code and URLs, the authorization URL, and the claims sub, iss,
// name, email, preferred_username and exp.

#include "../net/json_lite.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/oidc_host.h>
#include <cvc/core/app.h>
#include <cvc/core/sliced_task.h> // threads_available
#include <cvc/net/browser.h>
#include <cvc/net/credentials.h>
#include <cvc/net/http_client.h>
#include <cvc/net/loopback_redirect.h>
#include <cvc/net/net_policy.h>
#include <cvc/net/oidc.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

namespace se = cvc::state_exec;
namespace oidc = cvc::net::oidc;
using steady = std::chrono::steady_clock;

namespace {

std::int64_t unix_now() {
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void wipe(std::string &s) {
  std::fill(s.begin(), s.end(), '\0');
  s.clear();
}

// The secret-free progress of one sign-in: what the state tree, (oidc-status) and the channel
// message show.
struct Status {
  std::string phase = "signed_out";
  std::string flow;   // loopback, device, redirect, or "stored" (a session kept from before)
  std::string issuer; // the issuer signed in with
  std::string error;  // a short code: the provider's OAuth error, or one of ours
  std::string error_description;
  std::string user_code, verification_uri, verification_uri_complete;
  std::int64_t code_expires_at = 0; // the device code's expiry, unix seconds
  std::int64_t interval = 0;        // seconds between the device flow's polls
  std::string browser_url;          // the authorization URL, for the user to open themselves
  bool browser_opened = false;
  std::int64_t expires_at = 0; // the access token's expiry, unix seconds (0: unknown)
  bool has_refresh = false;
  std::string sub, iss, name, email, preferred_username;
  std::int64_t claims_exp = 0;
  bool signed_in() const { return phase == "signed_in"; }
};

se::value_t status_dict(const Status &s, const std::string &credential) {
  return se::make_dict({
      {"phase", se::value_t(s.phase)},
      {"signed_in", se::value_t(s.signed_in())},
      {"credential", se::value_t(credential)},
      {"flow", se::value_t(s.flow)},
      {"issuer", se::value_t(s.issuer)},
      {"error", se::value_t(s.error)},
      {"error_description", se::value_t(s.error_description)},
      {"user_code", se::value_t(s.user_code)},
      {"verification_uri", se::value_t(s.verification_uri)},
      {"verification_uri_complete", se::value_t(s.verification_uri_complete)},
      {"code_expires_at", se::value_t(s.code_expires_at)},
      {"interval", se::value_t(s.interval)},
      {"browser_url", se::value_t(s.browser_url)},
      {"browser_opened", se::value_t(s.browser_opened)},
      {"expires_at", se::value_t(s.expires_at)},
      {"has_refresh", se::value_t(s.has_refresh)},
      {"claims", se::make_dict({{"sub", se::value_t(s.sub)},
                                {"iss", se::value_t(s.iss)},
                                {"name", se::value_t(s.name)},
                                {"email", se::value_t(s.email)},
                                {"preferred_username", se::value_t(s.preferred_username)},
                                {"exp", se::value_t(s.claims_exp)}})},
  });
}

void write_status(cvc::app &app, const std::string &path, const Status &s) {
  cvc::state &root = cvc::state::instance(app);
  const auto put = [&](const std::string &key, const std::string &value) {
    root.childThroughLinks(path + cvc::state::SEPARATOR + key).value(value);
  };
  const auto flag = [](bool b) { return std::string(b ? "true" : "false"); };
  put("signed_in", flag(s.signed_in()));
  put("flow", s.flow);
  put("issuer", s.issuer);
  put("error", s.error);
  put("error_description", s.error_description);
  put("user_code", s.user_code);
  put("verification_uri", s.verification_uri);
  put("verification_uri_complete", s.verification_uri_complete);
  put("code_expires_at", std::to_string(s.code_expires_at));
  put("interval", std::to_string(s.interval));
  put("browser_url", s.browser_url);
  put("browser_opened", flag(s.browser_opened));
  put("expires_at", std::to_string(s.expires_at));
  put("has_refresh", flag(s.has_refresh));
  const std::string c = std::string("claims") + cvc::state::SEPARATOR;
  put(c + "sub", s.sub);
  put(c + "iss", s.iss);
  put(c + "name", s.name);
  put(c + "email", s.email);
  put(c + "preferred_username", s.preferred_username);
  put(c + "exp", std::to_string(s.claims_exp));
  put("phase", s.phase); // last: a bind on the phase sees the rest already there
}

// A claim fit to show: printable, bounded.
std::string claim(const std::string &s) { return oidc::printable(s, 256); }

// What (oidc-sign-in OPTS) asked for.
struct SignInOptions {
  std::string issuer, client_id, client_secret, scope = "openid";
  oidc::Params params;
  std::string credential;
  std::string flow = "auto";
  bool interactive = true;
  std::string status; // the state path, under the caller's root
  std::string redirect_uri;
  std::string redirect_path = "/callback";
  int port = 0;
  double timeout = 300.0;
  bool openid() const { return (" " + scope + " ").find(" openid ") != std::string::npos; }
};

// One request on its way, its reply when done.
struct Pending {
  std::atomic<bool> done{false};
  cvc::net::HttpResponse reply;
};

struct Host;

// One credential's sign-in: its options, its provider, the flow in progress and its status.
class Session : public std::enable_shared_from_this<Session> {
public:
  enum class Step {
    idle,
    discover,      // the discovery request is out
    resume,        // a kept session's refresh is out
    loopback_wait, // waiting for the browser's redirect
    exchange,      // the code exchange is out
    device_start,  // the device authorization request is out
    device_wait,   // the code is shown; polling at its interval
    device_poll,   // a poll is out
    redirecting,   // a browser page is leaving for the provider
    revoke,        // a revocation is out
  };

  Session(const std::shared_ptr<Host> &host, std::string name);

  const std::string &name() const { return name_; }
  std::string refresh_name() const { return name_ + ".refresh"; }
  std::string id_name() const { return name_ + ".id"; }

  // Start a sign-in (the verb, on the host's thread). Returns "started", "in_progress" or
  // "signed_in" (already).
  std::string start(SignInOptions opts, std::string status_path);
  bool cancel();
  void set_status_path(std::string path) {
    std::lock_guard<std::mutex> lock(m_);
    if (status_path_.empty()) {
      status_path_ = std::move(path);
      ++version_;
    }
  }
  std::string sign_out(bool revoke, bool end_session, const std::string &post_logout_redirect);
  void step(); // the pump, on the host's thread
  // A request is out, or a browser was opened and its redirect is due (a snapshot waits for it);
  // a flow waiting on a person (a device code, a URL nobody opened) is not busy.
  bool busy() const {
    return (pending_ != nullptr && step_ != Step::idle) ||
           (step_ == Step::loopback_wait && browser_opened_);
  }
  Status status() const {
    std::lock_guard<std::mutex> lock(m_);
    return status_;
  }
  std::string status_path() const {
    std::lock_guard<std::mutex> lock(m_);
    return status_path_;
  }
  // Mirror the status to the state tree when it changed (the host's thread).
  void mirror(cvc::app &app);

  // The renewer's side, from any thread (renew_async: the one thread of a browser page).
  bool expiring() const;
  std::string renew(const std::string &rejected);
  void renew_async(const std::string &rejected, std::function<void(std::string)> done);
  void close_listener() {
    if (listener_)
      listener_->close();
    listener_.reset();
  }
  // The host turned sign-in off: nothing more may arrive (on the host's thread).
  void shutdown() {
    close_listener();
    if (std::shared_ptr<cvc::net::HttpTransfer> t = std::move(transfer_))
      t->abort();
    if (std::shared_ptr<cvc::net::HttpTransfer> t = std::move(renew_transfer_))
      t->abort();
    pending_.reset();
  }

private:
  void set_status(const std::function<void(Status &)> &edit, bool terminal = false);
  void stop_flow(bool ok, const std::string &message);
  void fail(const std::string &code, const std::string &description);
  void send(cvc::net::HttpRequest req, Step next);
  cvc::net::HttpRequest form_post(const std::string &url, const oidc::Params &fields) const;
  void after_discovery();
  void decide();
  void start_loopback();
  void start_device();
  void start_redirect();
  bool redirect_return();
  void exchange(const std::string &code, const std::string &redirect_uri);
  void take_tokens(oidc::TokenReply &t, std::optional<std::string> nonce, bool need_id_token,
                   const std::string &flow);
  std::string keep_tokens(oidc::TokenReply &t, std::optional<std::string> nonce, bool need_id_token,
                          const oidc::Provider &p, const std::string &client_id, Status &into);
  void bind_names(const oidc::Provider &p);
  struct RefreshPlan;
  void plan_refresh(const std::string &rejected, RefreshPlan &plan);
  std::string take_refresh(RefreshPlan &plan, cvc::net::HttpResponse &reply);
  void finish_renew(const std::string &why);
  void handle(cvc::net::HttpResponse &reply);
  void resume_from_store();

  // What the session uses, held by it: a renewal on the background pool may outlive the host.
  std::weak_ptr<Host> host_; // the provider cache only
  cvc::app *app_;            // outlives the network being on (enable_net's contract)
  std::shared_ptr<const cvc::net::NetPolicy> policy_;
  std::shared_ptr<cvc::net::CredentialStore> store_;
  OidcHostConfig config_;
  std::string name_;
  mutable std::mutex m_;   // status_, version_, provider_, opts_, status_path_
  std::mutex renew_mutex_; // one renewal at a time
  Status status_;
  std::uint64_t version_ = 1, mirrored_ = 0;
  std::string status_path_;
  SignInOptions opts_;
  std::optional<oidc::Provider> provider_;
  std::atomic<std::int64_t> expires_at_{0};
  std::atomic<std::int64_t> lifetime_{0}; // the last token's expires_in
  std::atomic<std::int64_t> renew_backoff_until_{0};

  // The flow in progress (the host's thread only).
  Step step_ = Step::idle;
  std::shared_ptr<Pending> pending_;
  std::shared_ptr<std::atomic<bool>> renewing_; // a pump-started renewal is out
  // Where requests are started rather than sent (cvc::net::http_starts_async): the flow's request
  // out, a refresh out, and the callers waiting for that refresh.
  std::shared_ptr<cvc::net::HttpTransfer> transfer_;
  std::shared_ptr<cvc::net::HttpTransfer> renew_transfer_;
  std::vector<std::function<void(std::string)>> renew_waiters_;
  std::unique_ptr<cvc::net::LoopbackRedirect> listener_;
  std::string verifier_, state_, nonce_, redirect_uri_, device_code_, flow_;
  steady::time_point deadline_{}, next_poll_{};
  std::int64_t interval_ = 5;
  bool browser_opened_ = false;
};

// The renewer the credential store consults for a session's access token. Holds the session
// weakly: the host owns sessions, and the store must not keep one alive.
class Renewer : public cvc::net::CredentialRenewer {
public:
  explicit Renewer(std::weak_ptr<Session> s) : s_(std::move(s)) {}
  bool expiring() const override {
    const std::shared_ptr<Session> s = s_.lock();
    return s && s->expiring();
  }
  std::string renew(const std::string &rejected) override {
    const std::shared_ptr<Session> s = s_.lock();
    return s ? s->renew(rejected) : std::string("the sign-in has ended");
  }
  void renew_async(const std::string &rejected, std::function<void(std::string)> done) override {
    if (const std::shared_ptr<Session> s = s_.lock())
      s->renew_async(rejected, std::move(done));
    else
      done("the sign-in has ended");
  }

private:
  std::weak_ptr<Session> s_;
};

struct CachedProvider {
  oidc::Provider provider;
  steady::time_point at;
};

struct Host : std::enable_shared_from_this<Host> {
  cvc::app *app = nullptr;
  std::shared_ptr<const cvc::net::NetPolicy> policy;
  std::shared_ptr<cvc::net::CredentialStore> store;
  std::shared_ptr<const std::atomic<bool>> live;
  OidcHostConfig config;
  std::mutex m; // sessions, providers
  std::map<std::string, std::shared_ptr<Session>> sessions;
  std::map<std::string, CachedProvider> providers;
  std::chrono::steady_clock::time_point last_open{};

  // The pinned issuers: the flags' and the policy's oidc lines. Empty: any the policy allows.
  bool pinned() const { return !policy->oidc_issuers().empty(); }
};

// The origins besides the issuer's its endpoints may be at (the policy's oidc line).
std::vector<cvc::net::Origin> endpoint_origins(const cvc::net::NetPolicy &policy,
                                               const std::string &issuer) {
  const cvc::net::OidcIssuer *i = policy.oidc_issuer(issuer);
  return i ? i->endpoint_origins : std::vector<cvc::net::Origin>{};
}

// Run `job` (a request) on the app's background pool, or here and now where there are no threads
// (a browser build without them), as the http verbs' kernels run.
void run_job(cvc::app *app, std::function<void()> job) {
  if (cvc::threads_available() && app->submitPooled(job))
    return;
  job();
}

Session::Session(const std::shared_ptr<Host> &host, std::string name)
    : host_(host), app_(host->app), policy_(host->policy), store_(host->store),
      config_(host->config), name_(std::move(name)) {}

std::mutex &host_mutex() {
  static std::mutex m;
  return m;
}
std::shared_ptr<Host> &host_cell() {
  static std::shared_ptr<Host> h;
  return h;
}
std::atomic<bool> g_on{false};

std::shared_ptr<Host> current_host() {
  std::lock_guard<std::mutex> lock(host_mutex());
  return host_cell();
}

// --- Session -------------------------------------------------------------------------------------

void Session::set_status(const std::function<void(Status &)> &edit, bool terminal) {
  Status copy;
  std::string path;
  {
    std::lock_guard<std::mutex> lock(m_);
    edit(status_);
    ++version_;
    copy = status_;
    path = status_path_;
  }
  if (terminal && app_) // a program waiting with (msg-recv "oidc#NAME") hears of the end
    app_->exec_scheduler().post_message("oidc#" + name_, status_dict(copy, name_));
}

// End the flow in progress: answer a waiting browser tab, drop the secrets and any request out.
void Session::stop_flow(bool ok, const std::string &message) {
  if (listener_) {
    listener_->respond(ok, message);
    listener_.reset();
  }
  if (step_ == Step::redirecting)
    cvc::net::page::session_remove("cvc.oidc." + name_);
  wipe(verifier_);
  wipe(device_code_);
  pending_.reset();
  if (std::shared_ptr<cvc::net::HttpTransfer> t = std::move(transfer_))
    t->abort();
  step_ = Step::idle;
}

void Session::fail(const std::string &code, const std::string &description) {
  stop_flow(false, description);
  set_status(
      [&](Status &s) {
        s.phase = "error";
        s.error = code;
        s.error_description = description;
        s.user_code.clear();
        s.verification_uri.clear();
        s.verification_uri_complete.clear();
        s.code_expires_at = 0;
      },
      true);
}

void Session::mirror(cvc::app &app) {
  Status copy;
  std::string path;
  {
    std::lock_guard<std::mutex> lock(m_);
    if (mirrored_ == version_ || status_path_.empty())
      return;
    mirrored_ = version_;
    copy = status_;
    path = status_path_;
  }
  try {
    write_status(app, path, copy);
  } catch (const std::exception &) {
    // A read-only node in the way: the document chose a path it cannot write. (oidc-status) still
    // answers.
  }
}

cvc::net::HttpRequest Session::form_post(const std::string &url, const oidc::Params &fields) const {
  cvc::net::HttpRequest req;
  req.method = "POST";
  req.url = url;
  req.headers = {"Content-Type: application/x-www-form-urlencoded", "Accept: application/json"};
  req.body = oidc::form_encode(fields);
  req.follow_redirects = false; // a token endpoint answers; it does not redirect
  req.timeout_secs = 20;
  req.max_bytes = 1u << 20;
  return req;
}

void Session::send(cvc::net::HttpRequest req, Step next) {
  auto pending = std::make_shared<Pending>();
  pending_ = pending;
  step_ = next;
  if (cvc::net::http_starts_async()) { // a browser page without threads: never block a frame
    transfer_ = cvc::net::start(req, policy_, [pending](cvc::net::HttpResponse r) {
      pending->reply = std::move(r);
      pending->done.store(true, std::memory_order_release);
    });
    wipe(req.body);
    return;
  }
  std::shared_ptr<const cvc::net::NetPolicy> policy = policy_;
  run_job(app_, [pending, policy, req = std::move(req)]() mutable {
    pending->reply = cvc::net::send(req, *policy);
    wipe(req.body);
    pending->done.store(true, std::memory_order_release);
  });
}

std::string Session::start(SignInOptions opts, std::string status_path) {
  if (step_ != Step::idle && step_ != Step::revoke) {
    std::lock_guard<std::mutex> lock(m_);
    status_path_ = std::move(status_path);
    ++version_;
    return "in_progress";
  }
  {
    std::lock_guard<std::mutex> lock(m_);
    opts_ = std::move(opts);
    status_path_ = std::move(status_path);
  }
  pending_.reset();
  close_listener();
  set_status([&](Status &s) {
    const bool kept = s.signed_in() && s.issuer == opts_.issuer;
    if (!kept)
      s = Status{};
    s.phase = "discovering";
    s.issuer = opts_.issuer;
    s.error.clear();
    s.error_description.clear();
  });
  // A provider read within the hour is used again.
  if (const std::shared_ptr<Host> host = host_.lock()) {
    std::lock_guard<std::mutex> lock(host->m);
    const auto it = host->providers.find(opts_.issuer);
    if (it != host->providers.end() && steady::now() - it->second.at < std::chrono::hours(1)) {
      std::lock_guard<std::mutex> l2(m_);
      provider_ = it->second.provider;
    }
  }
  bool cached = false;
  {
    std::lock_guard<std::mutex> lock(m_);
    cached = provider_ && provider_->issuer == opts_.issuer;
  }
  if (cached) {
    after_discovery();
    return "started";
  }
  cvc::net::HttpRequest req;
  req.method = "GET";
  req.url = oidc::discovery_url(opts_.issuer);
  req.headers = {"Accept: application/json"};
  req.timeout_secs = 20;
  req.max_bytes = 1u << 20;
  send(std::move(req), Step::discover);
  return "started";
}

void Session::after_discovery() { decide(); }

void Session::bind_names(const oidc::Provider &p) {
  const cvc::net::Origin issuer = *cvc::net::parse_origin(p.issuer);
  std::vector<cvc::net::Origin> to = {issuer};
  const auto &bindings = policy_->credential_bindings();
  if (const auto it = bindings.find(name_); it != bindings.end())
    for (const cvc::net::Origin &o : it->second)
      if (std::find(to.begin(), to.end(), o) == to.end())
        to.push_back(o);
  store_->bind(name_, to);
  store_->bind(refresh_name(), {*cvc::net::parse_origin(p.token_endpoint)});
  store_->bind(id_name(), {issuer});
}

void Session::decide() {
  // A browser page coming back from the provider.
  if (redirect_return())
    return;
  // A session kept from before (a credential store file, or an earlier sign-in this run).
  if (store_->has(name_)) {
    resume_from_store();
    return;
  }
  if (!opts_.interactive) {
    step_ = Step::idle;
    set_status([](Status &s) { s.phase = "signed_out"; }, true);
    return;
  }
  const oidc::Provider &p = *provider_;
  const OidcFlows can = oidc_flows();
  std::string flow = opts_.flow;
  if (flow == "auto") {
    if (can.redirect && !p.authorization_endpoint.empty())
      flow = "redirect";
    else if (can.loopback && cvc::net::browser_available() && !p.authorization_endpoint.empty())
      flow = "loopback";
    else if (!p.device_authorization_endpoint.empty())
      flow = "device";
    else if (can.loopback && !p.authorization_endpoint.empty())
      flow = "loopback"; // no browser here: the user opens the URL themselves
    else
      return fail("unsupported_flow", "the provider supports neither an authorization endpoint "
                                      "nor the device flow (RFC 8628)");
  }
  if (flow == "device") {
    if (p.device_authorization_endpoint.empty())
      return fail("unsupported_flow", "the provider has no device_authorization_endpoint: it "
                                      "does not support the device flow (RFC 8628)");
    return start_device();
  }
  if (p.authorization_endpoint.empty())
    return fail("unsupported_flow",
                "the provider has no authorization_endpoint (try flow \"device\")");
  if (flow == "loopback") {
    if (!can.loopback)
      return fail("unsupported_flow", "this host cannot listen for a loopback redirect (a "
                                      "browser page: use flow \"redirect\" or \"auto\")");
    return start_loopback();
  }
  if (!can.redirect)
    return fail("unsupported_flow",
                "the redirect flow runs in a browser page; this host is native (use \"loopback\", "
                "\"device\" or \"auto\")");
  start_redirect();
}

void Session::resume_from_store() {
  bind_names(*provider_);
  Status s;
  s.phase = "signed_in";
  s.flow = "stored";
  s.issuer = opts_.issuer;
  if (const std::optional<std::string> id =
          store_->token_for(id_name(), *cvc::net::parse_origin(opts_.issuer))) {
    if (const std::optional<oidc::IdTokenClaims> c = oidc::decode_id_token(*id);
        c && c->iss == opts_.issuer) {
      s.sub = claim(c->sub);
      s.iss = claim(c->iss);
      s.name = claim(c->name);
      s.email = claim(c->email);
      s.preferred_username = claim(c->preferred_username);
      s.claims_exp = c->exp;
    }
  }
  const bool refresh = store_->has(refresh_name());
  s.has_refresh = refresh;
  store_->set_renewer(name_, std::make_shared<Renewer>(weak_from_this()));
  if (!refresh) { // nothing to check it with: the token is used until a server refuses it
    step_ = Step::idle;
    set_status([&](Status &st) { st = s; }, true);
    return;
  }
  // Refresh now: it says whether the provider still honours the session, and when the new token
  // expires. The renewal runs off this thread, as a request does.
  set_status([&](Status &st) {
    st = s;
    st.phase = "refreshing";
  });
  auto pending = std::make_shared<Pending>();
  pending_ = pending;
  step_ = Step::resume;
  const auto finished = [pending](std::string why) {
    pending->reply.error = std::move(why);
    pending->done.store(true, std::memory_order_release);
  };
  if (cvc::net::http_starts_async()) {
    renew_async(std::string(), finished);
    return;
  }
  std::weak_ptr<Session> weak = weak_from_this();
  run_job(app_, [weak, finished] {
    const std::shared_ptr<Session> self = weak.lock();
    finished(self ? self->renew(std::string()) : std::string("the sign-in has ended"));
  });
}

void Session::start_loopback() {
  listener_ = std::make_unique<cvc::net::LoopbackRedirect>();
  std::string why;
  if (!listener_->listen(opts_.port, opts_.redirect_path, &why)) {
    listener_.reset();
    return fail("loopback_failed", why);
  }
  redirect_uri_ = listener_->redirect_uri();
  verifier_ = oidc::random_token(48);
  state_ = oidc::random_token(24);
  nonce_ = opts_.openid() ? oidc::random_token(24) : std::string();
  oidc::AuthorizationRequest r{opts_.client_id, redirect_uri_, opts_.scope,
                               state_,          nonce_,        oidc::s256_challenge(verifier_),
                               opts_.params};
  const std::string url = oidc::authorization_url(*provider_, r);
  const cvc::net::OpenUrlResult opened = cvc::net::open_url(url);
  if (opened != cvc::net::OpenUrlResult::opened)
    cvc::net::url_notice(url);
  flow_ = "loopback";
  browser_opened_ = opened == cvc::net::OpenUrlResult::opened;
  step_ = Step::loopback_wait;
  deadline_ = steady::now() + std::chrono::duration_cast<steady::duration>(
                                  std::chrono::duration<double>(opts_.timeout));
  set_status([&](Status &s) {
    s.phase = "waiting_for_browser";
    s.flow = "loopback";
    s.browser_url = url;
    s.browser_opened = opened == cvc::net::OpenUrlResult::opened;
  });
}

void Session::start_device() {
  oidc::Params f = {{"client_id", opts_.client_id}, {"scope", opts_.scope}};
  if (!opts_.client_secret.empty())
    f.emplace_back("client_secret", opts_.client_secret);
  for (const auto &kv : opts_.params)
    f.push_back(kv);
  flow_ = "device";
  set_status([](Status &s) {
    s.phase = "discovering";
    s.flow = "device";
  });
  send(form_post(provider_->device_authorization_endpoint, f), Step::device_start);
}

namespace {
// The page's URL without the parameters an authorization reply adds, and without a fragment.
std::string without_reply_params(const std::string &url) {
  std::string base = url.substr(0, url.find('#'));
  const std::size_t q = base.find('?');
  if (q == std::string::npos)
    return base;
  const std::string query = base.substr(q + 1);
  base.resize(q);
  std::string kept;
  std::size_t start = 0;
  while (start <= query.size()) {
    std::size_t end = query.find('&', start);
    if (end == std::string::npos)
      end = query.size();
    const std::string pair = query.substr(start, end - start);
    const std::string key = pair.substr(0, pair.find('='));
    static const char *kReply[] = {
        "code", "state", "iss", "session_state", "error", "error_description", "error_uri"};
    const bool reply =
        std::any_of(std::begin(kReply), std::end(kReply), [&](const char *k) { return key == k; });
    if (!pair.empty() && !reply)
      kept += (kept.empty() ? "" : "&") + pair;
    start = end + 1;
  }
  return kept.empty() ? base : base + "?" + kept;
}

std::string json_quote(const std::string &s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(static_cast<char>(c));
    } else if (c < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof buf, "\\u%04x", c);
      out += buf;
    } else {
      out.push_back(static_cast<char>(c));
    }
  }
  return out + "\"";
}

constexpr char kPendingKeyPrefix[] = "cvc.oidc."; // + the credential name
constexpr std::int64_t kRedirectMaxAge = 15 * 60;
} // namespace

void Session::start_redirect() {
  const std::string page = cvc::net::page::url();
  redirect_uri_ = !config_.redirect_uri.empty() ? config_.redirect_uri
                  : !opts_.redirect_uri.empty() ? opts_.redirect_uri
                                                : without_reply_params(page);
  const std::optional<cvc::net::Origin> ro = cvc::net::parse_origin(redirect_uri_);
  const std::optional<cvc::net::Origin> po = cvc::net::parse_origin(page);
  if (!ro || !po || *ro != *po)
    return fail("bad_redirect_uri", "the redirect URI must be on this page's origin, so the "
                                    "provider sends the browser back here");
  verifier_ = oidc::random_token(48);
  state_ = oidc::random_token(24);
  nonce_ = opts_.openid() ? oidc::random_token(24) : std::string();
  oidc::AuthorizationRequest r{opts_.client_id, redirect_uri_, opts_.scope,
                               state_,          nonce_,        oidc::s256_challenge(verifier_),
                               opts_.params};
  const std::string url = oidc::authorization_url(*provider_, r);
  // What the page needs when the provider sends it back: sessionStorage is this tab's and this
  // origin's only, and lasts across the navigation.
  const std::string record =
      "{\"v\":1,\"issuer\":" + json_quote(opts_.issuer) +
      ",\"client_id\":" + json_quote(opts_.client_id) +
      ",\"redirect_uri\":" + json_quote(redirect_uri_) + ",\"state\":" + json_quote(state_) +
      ",\"nonce\":" + json_quote(nonce_) + ",\"verifier\":" + json_quote(verifier_) +
      ",\"at\":" + std::to_string(unix_now()) + "}";
  if (!cvc::net::page::session_set(kPendingKeyPrefix + name_, record))
    return fail("redirect_failed", "the page's sessionStorage is not available");
  wipe(verifier_);
  flow_ = "redirect";
  step_ = Step::redirecting;
  set_status([&](Status &s) {
    s.phase = "redirecting";
    s.flow = "redirect";
    s.browser_url = url;
    s.browser_opened = true;
  });
  if (!cvc::net::page::navigate(url)) {
    cvc::net::page::session_remove(kPendingKeyPrefix + name_);
    fail("redirect_failed", "the page could not go to the authorization endpoint");
  }
}

bool Session::redirect_return() {
  const std::string key = kPendingKeyPrefix + name_;
  std::string record = cvc::net::page::session_get(key);
  if (record.empty())
    return false;
  const std::string page = cvc::net::page::url();
  const std::size_t q = page.find('?');
  const std::string query =
      q == std::string::npos ? std::string() : page.substr(q + 1, page.find('#') - q - 1);
  const oidc::AuthorizationResponse r = oidc::parse_authorization_response(query);
  const std::optional<cvc::net::json_lite::Value> rec = cvc::net::json_lite::parse(record);
  wipe(record);
  if (r.state.empty() && r.error.empty()) {
    // Back without a reply (the user came back another way): the attempt is over.
    cvc::net::page::session_remove(key);
    return false;
  }
  cvc::net::page::replace_url(without_reply_params(page)); // the code leaves the address bar
  cvc::net::page::session_remove(key);
  if (!rec || !rec->is_object() || rec->string_at("issuer").value_or("") != opts_.issuer ||
      rec->string_at("client_id").value_or("") != opts_.client_id) {
    fail("stale_redirect", "the sign-in the page came back from was for another issuer or "
                           "client; sign in again");
    return true;
  }
  if (unix_now() - rec->int_at("at").value_or(0) > kRedirectMaxAge) {
    fail("stale_redirect", "the sign-in the page came back from is too old; sign in again");
    return true;
  }
  state_ = rec->string_at("state").value_or("");
  nonce_ = rec->string_at("nonce").value_or("");
  verifier_ = rec->string_at("verifier").value_or("");
  redirect_uri_ = rec->string_at("redirect_uri").value_or("");
  flow_ = "redirect";
  if (const std::string why = oidc::check_authorization_response(r, *provider_, state_);
      !why.empty()) {
    fail(r.error.empty() ? (r.state != state_ ? "state_mismatch" : "invalid_response") : r.error,
         why);
    return true;
  }
  exchange(r.code, redirect_uri_);
  return true;
}

void Session::exchange(const std::string &code, const std::string &redirect_uri) {
  oidc::Params f = {{"grant_type", "authorization_code"},
                    {"code", code},
                    {"redirect_uri", redirect_uri},
                    {"client_id", opts_.client_id},
                    {"code_verifier", verifier_}};
  if (!opts_.client_secret.empty())
    f.emplace_back("client_secret", opts_.client_secret);
  wipe(verifier_);
  set_status([&](Status &s) {
    s.phase = "exchanging";
    s.flow = flow_;
  });
  send(form_post(provider_->token_endpoint, f), Step::exchange);
  for (auto &kv : f)
    wipe(kv.second);
}

// Keep the tokens of a successful reply `t` (checked first): the access token as the credential,
// the refresh token and the ID token beside it. `into` gets the claims and expiry. "" or why not;
// nothing is kept then.
std::string Session::keep_tokens(oidc::TokenReply &t, std::optional<std::string> nonce,
                                 bool need_id_token, const oidc::Provider &p,
                                 const std::string &client_id, Status &into) {
  std::optional<oidc::IdTokenClaims> claims;
  if (!t.id_token.empty()) {
    std::string why;
    claims = oidc::decode_id_token(t.id_token, &why);
    if (!claims)
      return why;
    oidc::IdTokenExpect e;
    e.issuer = p.issuer;
    e.client_id = client_id;
    e.nonce = std::move(nonce);
    e.now = unix_now();
    {
      std::lock_guard<std::mutex> lock(m_);
      e.sub = status_.signed_in() ? status_.sub : std::string();
    }
    if (const std::string bad = oidc::check_id_token(*claims, e); !bad.empty())
      return bad;
  } else if (need_id_token) {
    return "the token endpoint sent no ID token for an openid sign-in";
  }
  if (!cvc::net::CredentialStore::valid_token(t.access_token))
    return "the access token is not a bearer token this host can keep (RFC 6750)";
  bind_names(p);
  const cvc::net::Origin issuer = *cvc::net::parse_origin(p.issuer);
  const cvc::net::Origin token = *cvc::net::parse_origin(p.token_endpoint);
  std::string why;
  if (!store_->capture(name_, issuer, t.access_token, &why))
    return "the access token was not kept: " + why;
  std::string warning = why; // a file that could not be written: kept in memory
  if (!t.refresh_token.empty()) {
    why.clear();
    if (!store_->capture(refresh_name(), token, t.refresh_token, &why))
      store_->forget(refresh_name());
  }
  if (!t.id_token.empty()) {
    why.clear();
    if (!store_->capture(id_name(), issuer, t.id_token, &why))
      store_->forget(id_name()); // too long to keep: sign-out sends no id_token_hint
  }
  const std::int64_t exp = t.expires_in && *t.expires_in > 0 ? unix_now() + *t.expires_in : 0;
  expires_at_.store(exp);
  lifetime_.store(exp ? *t.expires_in : 0);
  store_->set_renewer(name_, std::make_shared<Renewer>(weak_from_this()));
  into.phase = "signed_in";
  into.issuer = p.issuer;
  into.error.clear();
  into.error_description = warning.empty() ? std::string() : warning;
  into.expires_at = exp;
  into.has_refresh = store_->has(refresh_name());
  if (claims) {
    into.sub = claim(claims->sub);
    into.iss = claim(claims->iss);
    into.name = claim(claims->name);
    into.email = claim(claims->email);
    into.preferred_username = claim(claims->preferred_username);
    into.claims_exp = claims->exp;
  }
  into.user_code.clear();
  into.verification_uri.clear();
  into.verification_uri_complete.clear();
  into.code_expires_at = 0;
  return std::string();
}

void Session::take_tokens(oidc::TokenReply &t, std::optional<std::string> nonce, bool need_id_token,
                          const std::string &flow) {
  Status next;
  {
    std::lock_guard<std::mutex> lock(m_);
    next = status_;
  }
  const std::string why =
      keep_tokens(t, std::move(nonce), need_id_token, *provider_, opts_.client_id, next);
  t.wipe();
  if (!why.empty()) {
    store_->forget(name_);
    store_->forget(refresh_name());
    store_->forget(id_name());
    return fail("invalid_token_reply", why);
  }
  if (listener_) {
    listener_->respond(true, "You are signed in.");
    listener_.reset();
  }
  step_ = Step::idle;
  next.flow = flow;
  next.browser_url.clear();
  set_status([&](Status &s) { s = next; }, true);
}

// Renew 30 s ahead of expiry, or halfway through a token that lives less than a minute.
bool Session::expiring() const {
  const std::int64_t exp = expires_at_.load();
  const std::int64_t margin = std::min<std::int64_t>(30, lifetime_.load() / 2);
  return exp != 0 && unix_now() + margin >= exp && store_->has(refresh_name());
}

// A refresh (RFC 6749 6), on whatever thread asks: the credential store's renewal before or after
// a request, the pump's ahead of expiry, a resumed session's check. One at a time.
// What a refresh needs before it is sent: the provider, the client, and the refresh token, or why
// it cannot be (an empty `why` with no request: a renewal raced this one, nothing to do).
struct Session::RefreshPlan {
  std::optional<oidc::Provider> provider;
  SignInOptions opts;
  std::string refresh;
  cvc::net::HttpRequest req;
  bool send = false;
  std::string why;
  ~RefreshPlan() {
    wipe(refresh);
    wipe(req.body);
  }
};

void Session::plan_refresh(const std::string &rejected, RefreshPlan &plan) {
  {
    std::lock_guard<std::mutex> lock(m_);
    plan.provider = provider_;
    plan.opts = opts_;
  }
  if (!plan.provider) {
    plan.why = "the provider is not known yet";
    return;
  }
  const oidc::Provider &p = *plan.provider;
  const cvc::net::Origin issuer = *cvc::net::parse_origin(p.issuer);
  const cvc::net::Origin token_origin = *cvc::net::parse_origin(p.token_endpoint);
  if (!rejected.empty()) {
    std::optional<std::string> now = store_->token_for(name_, issuer);
    const bool raced = now && *now != rejected;
    if (now)
      wipe(*now);
    if (raced)
      return; // another request renewed it meanwhile
  }
  std::optional<std::string> refresh = store_->token_for(refresh_name(), token_origin);
  if (!refresh) {
    plan.why = "there is no refresh token";
    return;
  }
  plan.refresh = std::move(*refresh);
  oidc::Params f = {{"grant_type", "refresh_token"},
                    {"refresh_token", plan.refresh},
                    {"client_id", plan.opts.client_id}};
  if (!plan.opts.client_secret.empty())
    f.emplace_back("client_secret", plan.opts.client_secret);
  plan.req = form_post(p.token_endpoint, f);
  for (auto &kv : f)
    wipe(kv.second);
  plan.send = true;
}

// The token endpoint's reply to a refresh (RFC 6749 6): keep the new tokens, or, when the provider
// ended the session (invalid_grant), forget them all and say so. "" or why not.
std::string Session::take_refresh(RefreshPlan &plan, cvc::net::HttpResponse &reply) {
  if (!reply.ok) {
    renew_backoff_until_.store(unix_now() + 30);
    return "the token endpoint could not be reached: " + oidc::printable(reply.error);
  }
  oidc::TokenReply t = oidc::parse_token_reply(reply.status, reply.body);
  cvc::net::redact(reply.body, plan.refresh);
  wipe(reply.body);
  if (!t.ok) {
    if (t.error == "invalid_grant") { // the provider ended the session
      store_->forget(name_);
      store_->forget(refresh_name());
      store_->forget(id_name());
      expires_at_.store(0);
      set_status(
          [&](Status &s) {
            const std::string issuer_was = s.issuer;
            s = Status{};
            s.issuer = issuer_was;
            s.phase = "signed_out";
            s.error = "session_expired";
            s.error_description = "the provider ended the session (sign in again)";
          },
          true);
      return "the provider ended the session";
    }
    renew_backoff_until_.store(unix_now() + 30);
    return "the refresh failed: " + t.error;
  }
  Status next;
  {
    std::lock_guard<std::mutex> lock(m_);
    next = status_;
  }
  const std::string why =
      keep_tokens(t, std::nullopt, false, *plan.provider, plan.opts.client_id, next);
  t.wipe();
  if (!why.empty()) {
    renew_backoff_until_.store(unix_now() + 30);
    return why;
  }
  set_status([&](Status &s) {
    const std::string flow = s.flow;
    s = next;
    s.flow = flow;
  });
  return std::string();
}

// A refresh, sent in place, on whatever thread asks: the credential store's renewal before or after
// a request, the pump's ahead of expiry, a resumed session's check. One at a time.
std::string Session::renew(const std::string &rejected) {
  std::lock_guard<std::mutex> one(renew_mutex_);
  RefreshPlan plan;
  plan_refresh(rejected, plan);
  if (!plan.send)
    return plan.why;
  cvc::net::HttpResponse reply = cvc::net::send(plan.req, *policy_);
  return take_refresh(plan, reply);
}

// The same refresh started rather than sent, where requests must not block the frame loop (a
// browser page without threads, cvc::net::http_starts_async): `done` hears from the browser's
// event loop. Callers that ask while one is out wait for it and hear what it did.
void Session::renew_async(const std::string &rejected, std::function<void(std::string)> done) {
  if (!cvc::net::http_starts_async()) {
    done(renew(rejected));
    return;
  }
  renew_waiters_.push_back(std::move(done));
  if (renew_waiters_.size() > 1)
    return; // one is out: this caller hears what it did
  auto plan = std::make_shared<RefreshPlan>();
  plan_refresh(rejected, *plan);
  if (!plan->send) {
    finish_renew(plan->why);
    return;
  }
  std::shared_ptr<Session> self = shared_from_this();
  renew_transfer_ = cvc::net::start(plan->req, policy_, [self, plan](cvc::net::HttpResponse r) {
    self->renew_transfer_.reset();
    self->finish_renew(self->take_refresh(*plan, r));
  });
}

void Session::finish_renew(const std::string &why) {
  std::vector<std::function<void(std::string)>> waiters;
  waiters.swap(renew_waiters_);
  for (auto &w : waiters)
    w(why);
}

bool Session::cancel() {
  if (step_ == Step::idle || step_ == Step::revoke)
    return false;
  stop_flow(false, "The sign-in was cancelled.");
  set_status(
      [](Status &s) {
        s.phase = "cancelled";
        s.error = "cancelled";
        s.error_description = "the sign-in was cancelled";
        s.user_code.clear();
        s.verification_uri.clear();
        s.verification_uri_complete.clear();
        s.code_expires_at = 0;
        s.browser_url.clear();
      },
      true);
  return true;
}

std::string Session::sign_out(bool revoke, bool end_session,
                              const std::string &post_logout_redirect) {
  if (step_ != Step::idle && step_ != Step::revoke)
    stop_flow(false, "The sign-in was cancelled.");
  std::optional<oidc::Provider> p;
  SignInOptions o;
  {
    std::lock_guard<std::mutex> lock(m_);
    p = provider_;
    o = opts_;
  }
  std::optional<std::string> refresh, access, id;
  if (p) {
    refresh = store_->token_for(refresh_name(), *cvc::net::parse_origin(p->token_endpoint));
    access = store_->token_for(name_, *cvc::net::parse_origin(p->issuer));
    id = store_->token_for(id_name(), *cvc::net::parse_origin(p->issuer));
  }
  store_->forget(name_);
  store_->forget(refresh_name());
  store_->forget(id_name());
  expires_at_.store(0);
  std::string revoked = "no";
  // RFC 7009: the refresh token (which takes its access tokens with it), else the access token.
  if (revoke && p && !p->revocation_endpoint.empty() && (refresh || access)) {
    oidc::Params f = {{"token", refresh ? *refresh : *access},
                      {"token_type_hint", refresh ? "refresh_token" : "access_token"},
                      {"client_id", o.client_id}};
    if (!o.client_secret.empty())
      f.emplace_back("client_secret", o.client_secret);
    send(form_post(p->revocation_endpoint, f), Step::revoke);
    for (auto &kv : f)
      wipe(kv.second);
    revoked = "sent";
  }
  // RP-initiated logout: the provider's own session ends in the browser that holds it.
  std::string logout_url;
  if (end_session && p && !p->end_session_endpoint.empty()) {
    oidc::Params q = {{"client_id", o.client_id}};
    if (id)
      q.emplace_back("id_token_hint", *id);
    if (!post_logout_redirect.empty())
      q.emplace_back("post_logout_redirect_uri", post_logout_redirect);
    logout_url = p->end_session_endpoint;
    logout_url.push_back(logout_url.find('?') == std::string::npos ? '?' : '&');
    logout_url += oidc::form_encode(q);
    for (auto &kv : q)
      wipe(kv.second);
  }
  for (std::optional<std::string> *t : {&refresh, &access, &id})
    if (*t)
      wipe(**t);
  set_status(
      [&](Status &s) {
        const std::string issuer = s.issuer;
        s = Status{};
        s.issuer = issuer;
        s.phase = "signed_out";
      },
      true);
  if (!logout_url.empty()) {
    if (oidc_flows().redirect) {
      cvc::net::page::navigate(logout_url);
    } else if (cvc::net::open_url(logout_url) != cvc::net::OpenUrlResult::opened) {
      cvc::net::url_notice(logout_url);
    }
    wipe(logout_url);
  }
  return revoked;
}

void Session::handle(cvc::net::HttpResponse &reply) {
  const Step was = step_;
  step_ = Step::idle;
  const auto transport = [&](const char *what) {
    fail("network_error", std::string(what) + ": " + oidc::printable(reply.error));
  };
  switch (was) {
  case Step::discover: {
    if (!reply.ok)
      return transport("the discovery document could not be fetched");
    if (reply.status != 200)
      return fail("discovery_failed", "the discovery document of " + opts_.issuer + " answered " +
                                          std::to_string(reply.status));
    std::string why;
    std::optional<oidc::Provider> p = oidc::parse_discovery(
        reply.body, opts_.issuer, endpoint_origins(*policy_, opts_.issuer), &why);
    if (!p)
      return fail("discovery_failed", why);
    if (const std::shared_ptr<Host> host = host_.lock()) {
      std::lock_guard<std::mutex> lock(host->m);
      host->providers[opts_.issuer] = CachedProvider{*p, steady::now()};
    }
    {
      std::lock_guard<std::mutex> lock(m_);
      provider_ = std::move(p);
    }
    return after_discovery();
  }
  case Step::resume: {
    if (reply.error.empty()) { // renew() has set the status
      set_status([](Status &s) { s.flow = "stored"; }, true);
      return;
    }
    if (!store_->has(name_)) { // the provider ended it: sign in afresh, if asked to
      if (opts_.interactive) {
        std::lock_guard<std::mutex> lock(m_);
        status_.phase = "discovering";
      }
      if (opts_.interactive)
        return decide();
      return;
    }
    // Could not check (offline): the kept token is used until a server refuses it.
    set_status(
        [&](Status &s) {
          s.phase = "signed_in";
          s.flow = "stored";
          s.error = "refresh_failed";
          s.error_description = oidc::printable(reply.error);
        },
        true);
    return;
  }
  case Step::exchange: {
    if (!reply.ok)
      return transport("the token endpoint could not be reached");
    oidc::TokenReply t = oidc::parse_token_reply(reply.status, reply.body);
    if (!t.ok)
      return fail(t.error, t.error_description.empty() ? "the token endpoint refused the code"
                                                       : t.error_description);
    return take_tokens(t, nonce_.empty() ? std::nullopt : std::optional<std::string>(nonce_),
                       opts_.openid(), flow_);
  }
  case Step::device_start: {
    if (!reply.ok)
      return transport("the device authorization endpoint could not be reached");
    std::string why;
    std::optional<oidc::DeviceAuthorization> d =
        oidc::parse_device_authorization(reply.status, reply.body, &why);
    if (!d)
      return fail("device_failed", why);
    device_code_ = d->device_code;
    wipe(d->device_code);
    interval_ = d->interval;
    deadline_ = steady::now() + std::chrono::seconds(d->expires_in);
    next_poll_ = steady::now() + std::chrono::seconds(interval_);
    step_ = Step::device_wait;
    set_status([&](Status &s) {
      s.phase = "device_code_shown";
      s.flow = "device";
      s.user_code = d->user_code;
      s.verification_uri = d->verification_uri;
      s.verification_uri_complete = d->verification_uri_complete;
      s.code_expires_at = unix_now() + d->expires_in;
      s.interval = interval_;
    });
    return;
  }
  case Step::device_poll: {
    if (!reply.ok) { // a lost poll: try again at the next interval
      step_ = Step::device_wait;
      next_poll_ = steady::now() + std::chrono::seconds(interval_);
      return;
    }
    oidc::TokenReply t = oidc::parse_token_reply(reply.status, reply.body);
    if (t.ok) {
      wipe(device_code_);
      return take_tokens(t, std::nullopt, false, "device");
    }
    if (t.error == "authorization_pending" || t.error == "slow_down") {
      if (t.error == "slow_down")
        interval_ += 5; // RFC 8628 3.5
      step_ = Step::device_wait;
      next_poll_ = steady::now() + std::chrono::seconds(interval_);
      set_status([&](Status &s) { s.interval = interval_; });
      return;
    }
    if (t.error == "access_denied")
      return fail("access_denied", "the sign-in was denied");
    if (t.error == "expired_token")
      return fail("expired_token", "the code expired before it was entered; sign in again");
    return fail(t.error, t.error_description.empty() ? "the token endpoint refused the device code"
                                                     : t.error_description);
  }
  case Step::revoke:
    return; // best effort: the tokens are forgotten here either way
  default:
    return;
  }
}

void Session::step() {
  if (pending_) {
    if (!pending_->done.load(std::memory_order_acquire))
      return;
    std::shared_ptr<Pending> p = std::move(pending_);
    handle(p->reply);
    wipe(p->reply.body);
    return;
  }
  switch (step_) {
  case Step::loopback_wait: {
    if (std::optional<std::string> query = listener_->poll()) {
      const oidc::AuthorizationResponse r = oidc::parse_authorization_response(*query);
      wipe(*query);
      if (const std::string why = oidc::check_authorization_response(r, *provider_, state_);
          !why.empty())
        return fail(r.error.empty() ? (r.state != state_ ? "state_mismatch" : "invalid_response")
                                    : r.error,
                    why);
      return exchange(r.code, redirect_uri_);
    }
    if (steady::now() > deadline_)
      return fail("timeout", "no reply came back from the browser in time; sign in again");
    return;
  }
  case Step::device_wait: {
    if (steady::now() > deadline_)
      return fail("expired_token", "the code expired before it was entered; sign in again");
    if (steady::now() < next_poll_)
      return;
    oidc::Params f = {{"grant_type", "urn:ietf:params:oauth:grant-type:device_code"},
                      {"device_code", device_code_},
                      {"client_id", opts_.client_id}};
    if (!opts_.client_secret.empty())
      f.emplace_back("client_secret", opts_.client_secret);
    send(form_post(provider_->token_endpoint, f), Step::device_poll);
    for (auto &kv : f)
      wipe(kv.second);
    return;
  }
  case Step::idle: {
    // Ahead of expiry, renew in the background so the session stays fresh while idle.
    if (renewing_ && renewing_->load())
      return;
    if (!expiring() || unix_now() < renew_backoff_until_.load())
      return;
    auto flag = std::make_shared<std::atomic<bool>>(true);
    renewing_ = flag;
    if (cvc::net::http_starts_async()) {
      renew_async(std::string(), [flag](std::string) { flag->store(false); });
      return;
    }
    std::weak_ptr<Session> weak = weak_from_this();
    run_job(app_, [weak, flag] {
      if (const std::shared_ptr<Session> self = weak.lock())
        self->renew(std::string());
      flag->store(false);
    });
    return;
  }
  default:
    return;
  }
}

// --- the verbs -----------------------------------------------------------------------------------

const se::dict_ptr &options_dict(std::span<const se::value_t> args, std::size_t i,
                                 const char *verb) {
  static const se::dict_ptr empty =
      std::make_shared<std::vector<std::pair<std::string, se::value_t>>>();
  if (args.size() <= i)
    return empty;
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&args[i].v);
  if (!d || !*d)
    throw std::runtime_error(std::string(verb) + ": the options must be a dict");
  return *d;
}

std::string string_option(const se::value_t &v, const char *verb, const std::string &key) {
  const std::string *s = std::get_if<std::string>(&v.v);
  if (!s)
    throw std::runtime_error(std::string(verb) + ": '" + key + "' must be a string");
  return *s;
}

bool bool_option(const se::value_t &v, const char *verb, const std::string &key) {
  if (std::holds_alternative<std::monostate>(v.v)) // #f reads as nil
    return false;
  const bool *b = std::get_if<bool>(&v.v);
  if (!b)
    throw std::runtime_error(std::string(verb) + ": '" + key + "' must be #t or #f");
  return *b;
}

double number_option(const se::value_t &v, const char *verb, const std::string &key) {
  if (const int64_t *i = std::get_if<int64_t>(&v.v))
    return static_cast<double>(*i);
  if (const double *d = std::get_if<double>(&v.v))
    return *d;
  throw std::runtime_error(std::string(verb) + ": '" + key + "' must be a number");
}

std::string credential_option(const std::string &name, const char *verb) {
  if (!cvc::net::CredentialStore::valid_name(name) || name.size() > 56)
    throw std::runtime_error(std::string(verb) +
                             ": a credential name is 1-56 of A-Z a-z 0-9 . _ - (the host keeps "
                             "NAME.refresh and NAME.id beside it)");
  return name;
}

SignInOptions parse_sign_in(const Host &host, std::span<const se::value_t> args) {
  const char *verb = "oidc-sign-in";
  SignInOptions o;
  bool scope_given = false;
  for (const auto &[key, v] : *options_dict(args, 0, verb)) {
    if (key == "issuer") {
      o.issuer = string_option(v, verb, key);
    } else if (key == "client_id") {
      o.client_id = string_option(v, verb, key);
    } else if (key == "client_secret") {
      o.client_secret = string_option(v, verb, key);
    } else if (key == "scopes" || key == "scope") {
      scope_given = true;
      o.scope.clear();
      if (const std::string *s = std::get_if<std::string>(&v.v)) {
        o.scope = *s;
      } else if (const se::list_ptr *l = std::get_if<se::list_ptr>(&v.v); l && *l) {
        for (const se::value_t &e : **l)
          o.scope += (o.scope.empty() ? "" : " ") + string_option(e, verb, key);
      } else {
        throw std::runtime_error("oidc-sign-in: 'scopes' is a list of strings or one string");
      }
    } else if (key == "params") {
      const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v);
      if (!d || !*d)
        throw std::runtime_error("oidc-sign-in: 'params' must be a dict");
      for (const auto &[pk, pv] : **d) {
        if (!oidc::caller_param(pk))
          throw std::runtime_error("oidc-sign-in: the host sets '" + pk +
                                   "' itself; it cannot be one of 'params'");
        const std::string *s = std::get_if<std::string>(&pv.v);
        o.params.emplace_back(pk, s ? *s : se::to_string(pv));
      }
    } else if (key == "credential") {
      o.credential = credential_option(string_option(v, verb, key), verb);
    } else if (key == "flow") {
      o.flow = string_option(v, verb, key);
      if (o.flow != "auto" && o.flow != "loopback" && o.flow != "redirect" && o.flow != "device")
        throw std::runtime_error(
            "oidc-sign-in: 'flow' is auto, loopback, redirect or device, not '" + o.flow + "'");
    } else if (key == "interactive") {
      o.interactive = bool_option(v, verb, key);
    } else if (key == "status") {
      o.status = string_option(v, verb, key);
    } else if (key == "redirect_uri") {
      o.redirect_uri = string_option(v, verb, key);
    } else if (key == "redirect_path") {
      o.redirect_path = string_option(v, verb, key);
    } else if (key == "port") {
      const double p = number_option(v, verb, key);
      if (p < 0 || p > 65535 || p != static_cast<int>(p))
        throw std::runtime_error("oidc-sign-in: 'port' is 0-65535");
      o.port = static_cast<int>(p);
    } else if (key == "timeout") {
      o.timeout = number_option(v, verb, key);
      if (!(o.timeout > 0) || o.timeout > 3600)
        throw std::runtime_error("oidc-sign-in: 'timeout' is 0-3600 seconds");
    } else {
      throw std::runtime_error("oidc-sign-in: unknown option '" + key + "'");
    }
  }
  if (o.credential.empty())
    throw std::runtime_error("oidc-sign-in: name a 'credential' to keep the sign-in under");
  if (o.issuer.empty())
    throw std::runtime_error("oidc-sign-in: give the provider's 'issuer'");
  if (scope_given && o.scope.empty())
    o.scope = "openid";
  if (const std::string why = oidc::check_issuer(o.issuer); !why.empty())
    throw std::runtime_error("oidc-sign-in: " + why);
  if (!host.policy->allows(*cvc::net::parse_origin(o.issuer)))
    throw std::runtime_error("oidc-sign-in: the host does not allow " +
                             cvc::net::parse_origin(o.issuer)->str() +
                             " (its launcher allows an issuer with --oidc-issuer, --net-allow or "
                             "an oidc policy line)");
  const cvc::net::OidcIssuer *pin = host.policy->oidc_issuer(o.issuer);
  if (host.pinned() && !pin) {
    std::string names;
    for (const cvc::net::OidcIssuer &i : host.policy->oidc_issuers())
      names += (names.empty() ? "" : ", ") + i.issuer;
    throw std::runtime_error("oidc-sign-in: this host signs in only with " + names + ", not " +
                             o.issuer);
  }
  if (pin && !pin->client_id.empty())
    o.client_id = pin->client_id; // the launcher registered the client; its choice stands
  if (o.client_id.empty())
    throw std::runtime_error("oidc-sign-in: give a 'client_id' (or the launcher --oidc-client)");
  if (o.status.empty())
    o.status = std::string("oidc") + cvc::state::SEPARATOR + o.credential;
  return o;
}

std::string absolute_path(se::intrinsics_context &ictx, const std::string &path) {
  const std::string root = ictx.root ? ictx.root->fullName() : std::string();
  return root.empty() ? path : root + cvc::state::SEPARATOR + path;
}

std::shared_ptr<Session> session_for(Host &host, const std::string &name, bool make) {
  std::lock_guard<std::mutex> lock(host.m);
  const auto it = host.sessions.find(name);
  if (it != host.sessions.end())
    return it->second;
  if (!make)
    return nullptr;
  auto s = std::make_shared<Session>(host.shared_from_this(), name);
  host.sessions.emplace(name, s);
  return s;
}

void register_verbs(const std::weak_ptr<Host> &weak_host,
                    const std::shared_ptr<se::environment> &env, se::intrinsics_context &ictx) {
  se::intrinsics_context *ctx = &ictx;
  const auto live_host = [weak_host](const char *verb) {
    std::shared_ptr<Host> h = weak_host.lock();
    if (!h || (h->live && !h->live->load()))
      throw std::runtime_error(std::string(verb) + ": the host has turned the network off");
    return h;
  };
  // (oidc-sign-in OPTS): start (or resume) a sign-in. Returns "started", or "in_progress" when one
  // is already under way for that credential.
  se::builtins::register_fn(env, "oidc-sign-in",
                            [live_host, ctx](std::span<const se::value_t> args) -> se::value_t {
                              std::shared_ptr<Host> h = live_host("oidc-sign-in");
                              SignInOptions o = parse_sign_in(*h, args);
                              const std::string path = absolute_path(*ctx, o.status);
                              std::shared_ptr<Session> s = session_for(*h, o.credential, true);
                              return se::value_t(s->start(std::move(o), path));
                            });
  // (oidc-cancel NAME): stop a sign-in in progress. #t if there was one.
  se::builtins::register_fn(
      env, "oidc-cancel", [live_host](std::span<const se::value_t> args) -> se::value_t {
        std::shared_ptr<Host> h = live_host("oidc-cancel");
        if (args.empty())
          throw std::runtime_error("oidc-cancel: name the credential");
        const std::string name =
            credential_option(string_option(args[0], "oidc-cancel", "name"), "oidc-cancel");
        std::shared_ptr<Session> s = session_for(*h, name, false);
        return se::value_t(s && s->cancel());
      });
  // (oidc-sign-out NAME [OPTS]): forget the credential and its refresh and ID tokens; OPTS
  // "revoke" (#t: RFC 7009 where the provider has a revocation endpoint), "end_session" (#f: open
  // the provider's end_session_endpoint) and "post_logout_redirect_uri". Returns whether a
  // revocation was sent ("sent" or "no").
  se::builtins::register_fn(
      env, "oidc-sign-out", [live_host, ctx](std::span<const se::value_t> args) -> se::value_t {
        std::shared_ptr<Host> h = live_host("oidc-sign-out");
        if (args.empty())
          throw std::runtime_error("oidc-sign-out: name the credential");
        const std::string name =
            credential_option(string_option(args[0], "oidc-sign-out", "name"), "oidc-sign-out");
        bool revoke = true, end_session = false;
        std::string post_logout;
        for (const auto &[key, v] : *options_dict(args, 1, "oidc-sign-out")) {
          if (key == "revoke")
            revoke = bool_option(v, "oidc-sign-out", key);
          else if (key == "end_session")
            end_session = bool_option(v, "oidc-sign-out", key);
          else if (key == "post_logout_redirect_uri")
            post_logout = string_option(v, "oidc-sign-out", key);
          else
            throw std::runtime_error("oidc-sign-out: unknown option '" + key + "'");
        }
        std::shared_ptr<Session> s = session_for(*h, name, true);
        // A session no sign-in of this run started reports at the default path.
        s->set_status_path(absolute_path(*ctx, std::string("oidc") + cvc::state::SEPARATOR + name));
        return se::value_t(s->sign_out(revoke, end_session, post_logout));
      });
  // (oidc-status NAME): the sign-in's status, as a dict (no secret in it).
  se::builtins::register_fn(
      env, "oidc-status", [live_host](std::span<const se::value_t> args) -> se::value_t {
        std::shared_ptr<Host> h = live_host("oidc-status");
        if (args.empty())
          throw std::runtime_error("oidc-status: name the credential");
        const std::string name =
            credential_option(string_option(args[0], "oidc-status", "name"), "oidc-status");
        std::shared_ptr<Session> s = session_for(*h, name, false);
        Status st = s ? s->status() : Status{};
        if (!s && h->store->has(name))
          st.phase = "stored"; // kept, but not resumed by a sign-in yet
        return status_dict(st, name);
      });
}

} // namespace

// --- the host ------------------------------------------------------------------------------------

OidcFlows oidc_flows() {
  OidcFlows f;
  f.device = true;
  f.redirect = cvc::net::page::present(); // a browser page (or a test standing in for one)
  f.loopback = cvc::net::LoopbackRedirect::available();
  return f;
}

void enable_oidc(cvc::app &app, std::shared_ptr<const cvc::net::NetPolicy> policy,
                 std::shared_ptr<cvc::net::CredentialStore> store, const OidcHostConfig &config,
                 std::shared_ptr<const std::atomic<bool>> live) {
  disable_oidc();
  auto host = std::make_shared<Host>();
  host->app = &app;
  host->policy = std::move(policy);
  host->store = std::move(store);
  host->live = std::move(live);
  host->config = config;
  cvc::net::set_browser_disabled(config.no_browser);
  {
    std::lock_guard<std::mutex> lock(host_mutex());
    host_cell() = host;
  }
  g_on.store(true);
  std::weak_ptr<Host> weak = host;
  register_action_intrinsics(
      [weak](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        std::shared_ptr<Host> h = weak.lock();
        if (!h || (h->live && !h->live->load()))
          return; // turned off: later programs do not get the verbs
        register_verbs(weak, env, ictx);
      });
}

void disable_oidc() {
  std::shared_ptr<Host> host;
  {
    std::lock_guard<std::mutex> lock(host_mutex());
    host = std::move(host_cell());
    host_cell().reset();
  }
  g_on.store(false);
  if (!host)
    return;
  std::map<std::string, std::shared_ptr<Session>> sessions;
  {
    std::lock_guard<std::mutex> lock(host->m);
    sessions.swap(host->sessions);
  }
  for (auto &[name, s] : sessions) {
    s->shutdown();
    host->store->set_renewer(name, nullptr);
  }
  cvc::net::set_browser_disabled(false);
}

void pump_oidc() {
  if (!g_on.load(std::memory_order_acquire))
    return;
  std::shared_ptr<Host> host = current_host();
  if (!host)
    return;
  std::vector<std::shared_ptr<Session>> sessions;
  {
    std::lock_guard<std::mutex> lock(host->m);
    for (auto &kv : host->sessions)
      sessions.push_back(kv.second);
  }
  for (const std::shared_ptr<Session> &s : sessions) {
    try {
      s->step();
    } catch (const std::exception &) {
      // a CSPRNG failure, a bad URL from a provider: the flow ends in error on its next step
    }
    s->mirror(*host->app);
  }
}

bool oidc_busy() {
  if (!g_on.load(std::memory_order_acquire))
    return false;
  std::shared_ptr<Host> host = current_host();
  if (!host)
    return false;
  std::lock_guard<std::mutex> lock(host->m);
  for (auto &kv : host->sessions)
    if (kv.second->busy())
      return true;
  return false;
}

} // namespace ariadne
} // namespace cvc

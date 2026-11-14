#ifndef CVC_NET_BROWSER_H
#define CVC_NET_BROWSER_H

// cvc::net — the user's web browser (docs/ARIADNE.md §10.5, §10.6).
//
// open_url opens an http(s) URL in the user's browser: natively with $BROWSER when it is set (a
// command, or a ':'-separated list of them tried in turn; "%s" in one stands for the URL, else the
// URL is its last argument), else xdg-open (Linux, the BSDs), open (macOS) or ShellExecute
// (Windows), the child's output thrown away so it cannot garble a terminal UI; in a browser build
// with window.open. Where there is no browser to open (over SSH, no display, --no-browser) it opens
// nothing and says so, and the host shows the URL instead (url_notice). It takes only http and
// https URLs, parsed as strictly as a request's (net_policy.h); the caller checks them against its
// policy.
//
// cvc::net::page is the page a browser build runs in: its URL, its sessionStorage, and moving it
// (a navigation, or history.replaceState). Natively every call does nothing and returns empty.
//
// PER APP: the opener, the notice, the page hooks and --no-browser are `app`'s own (cvc::app), so
// a test's opener or a host's --no-browser on one app is never seen by another.

#include <functional>
#include <memory>
#include <string>

namespace cvc {
class app;
namespace net {

// Whether open_url can open a browser here: in a browser build, always; natively, when $BROWSER is
// set, or else on macOS and Windows outside an SSH session, and on Linux and the BSDs with a
// display (DISPLAY or WAYLAND_DISPLAY) outside one. False once set_browser_disabled(app, true).
bool browser_available(cvc::app &app);
// --no-browser: `app` never opens one (open_url reports no_browser).
void set_browser_disabled(cvc::app &app, bool disabled);

enum class OpenUrlResult {
  opened,     // a browser was asked to open it
  no_browser, // there is none to open here (browser_available() is false)
  failed,     // the URL is not an http(s) URL, or launching the browser failed
};
// Open `url` in the user's browser (see above). `error` says why on failed.
OpenUrlResult open_url(cvc::app &app, const std::string &url, std::string *error = nullptr);

// The seam a test (or a host) uses to open `app`'s URLs its own way: called instead of the browser,
// it returns whether the URL was opened. browser_available(app) is true while one is set. nullptr
// restores the default.
void set_url_opener(cvc::app &app, std::function<bool(const std::string &url)> opener);

// What the host does with a URL it could not open, so the user can open it themselves: by default,
// print it to stderr. A full-screen terminal host sets a no-op (the document shows the URL).
void set_url_notice(cvc::app &app, std::function<void(const std::string &url)> notice);
void url_notice(cvc::app &app, const std::string &url);

namespace page {

// Whether `app` runs in a browser page: a browser build, or a test standing in for one.
bool present(cvc::app &app);

// What a test sets to stand in for a browser page natively (each hook optional).
struct Hooks {
  std::function<std::string()> url;
  std::function<std::string(const std::string &key)> session_get;
  std::function<bool(const std::string &key, const std::string &value)> session_set;
  std::function<void(const std::string &key)> session_remove;
  std::function<bool(const std::string &url)> navigate;
  std::function<bool(const std::string &url)> replace_url;
};
// Install `app`'s hooks (nullptr removes them): its page functions call them instead of the
// browser's.
void set_hooks(cvc::app &app, std::shared_ptr<Hooks> hooks);

// The page's URL (location.href); "" natively.
std::string url(cvc::app &app);
// sessionStorage: per tab, kept across a navigation within the tab. "" when absent (or natively).
std::string session_get(cvc::app &app, const std::string &key);
bool session_set(cvc::app &app, const std::string &key, const std::string &value);
void session_remove(cvc::app &app, const std::string &key);
// Go to `url` (location.assign). The page unloads; false natively.
bool navigate(cvc::app &app, const std::string &url);
// Replace the page's URL in place (history.replaceState), without loading it; false natively.
bool replace_url(cvc::app &app, const std::string &url);

} // namespace page

} // namespace net
} // namespace cvc

#endif // CVC_NET_BROWSER_H

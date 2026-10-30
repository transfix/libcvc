// cvc::net — the user's web browser, and the page a browser build runs in. See
// inc/cvc/net/browser.h.

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cvc/net/browser.h>
#include <cvc/net/net_policy.h>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <shellapi.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

#if defined(__EMSCRIPTEN__)
// Only == and != in these bodies: clang-format and the preprocessor read them as C++ tokens and
// would split === and !== (see net_policy.cpp).
EM_JS(int, cvc_net_window_open, (const char *url), {
  try {
    var w = window.open(UTF8ToString(url), "_blank");
    if (w) {
      try {
        w.opener = null;
      } catch (e) {
      }
      return 1;
    }
  } catch (e) {
  }
  return 0;
});
EM_JS(char *, cvc_net_page_url, (), {
  var s = "";
  try {
    s = (typeof location == "object" && location && location.href) ? String(location.href) : "";
  } catch (e) {
  }
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});
EM_JS(char *, cvc_net_session_get, (const char *key), {
  var s = "";
  try {
    var v = sessionStorage.getItem(UTF8ToString(key));
    if (v != null)
      s = String(v);
  } catch (e) {
  }
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});
EM_JS(int, cvc_net_session_set, (const char *key, const char *value), {
  try {
    sessionStorage.setItem(UTF8ToString(key), UTF8ToString(value));
    return 1;
  } catch (e) {
    return 0;
  }
});
EM_JS(void, cvc_net_session_remove, (const char *key), {
  try {
    sessionStorage.removeItem(UTF8ToString(key));
  } catch (e) {
  }
});
EM_JS(int, cvc_net_navigate, (const char *url), {
  try {
    location.assign(UTF8ToString(url));
    return 1;
  } catch (e) {
    return 0;
  }
});
EM_JS(int, cvc_net_replace_url, (const char *url), {
  try {
    history.replaceState(history.state, "", UTF8ToString(url));
    return 1;
  } catch (e) {
    return 0;
  }
});
#endif

namespace cvc {
namespace net {

namespace {

std::atomic<bool> g_disabled{false};

std::mutex &hooks_mutex() {
  static std::mutex m;
  return m;
}
std::function<bool(const std::string &)> &opener_cell() {
  static std::function<bool(const std::string &)> f;
  return f;
}
std::function<void(const std::string &)> &notice_cell() {
  static std::function<void(const std::string &)> f;
  return f;
}

#if !defined(__EMSCRIPTEN__)
bool env_set(const char *name) {
  const char *v = std::getenv(name);
  return v && *v;
}

bool over_ssh() { return env_set("SSH_CONNECTION") || env_set("SSH_CLIENT") || env_set("SSH_TTY"); }
#endif

#if !defined(__EMSCRIPTEN__) && !defined(_WIN32)
// Start `argv` with its standard streams on /dev/null, and reap it on a detached thread.
bool spawn_quiet(const std::vector<std::string> &argv) {
  if (argv.empty())
    return false;
  std::vector<char *> args;
  for (const std::string &a : argv)
    args.push_back(const_cast<char *>(a.c_str()));
  args.push_back(nullptr);
  posix_spawn_file_actions_t fa;
  if (posix_spawn_file_actions_init(&fa) != 0)
    return false;
  posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
  posix_spawn_file_actions_addopen(&fa, 1, "/dev/null", O_WRONLY, 0);
  posix_spawn_file_actions_addopen(&fa, 2, "/dev/null", O_WRONLY, 0);
  pid_t pid = 0;
  const int rc = posix_spawnp(&pid, args[0], &fa, nullptr, args.data(), environ);
  posix_spawn_file_actions_destroy(&fa);
  if (rc != 0)
    return false;
  std::thread([pid] {
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
  }).detach();
  return true;
}
#endif

#if !defined(__EMSCRIPTEN__)
// $BROWSER: commands separated by ':', each split on spaces, "%s" standing for the URL (else the
// URL goes last). The first that starts is used.
bool open_with_browser_env(const std::string &url) {
  const char *spec = std::getenv("BROWSER");
  if (!spec || !*spec)
    return false;
  std::stringstream list(spec);
  for (std::string command; std::getline(list, command, ':');) {
    std::vector<std::string> argv;
    std::istringstream words(command);
    bool placed = false;
    for (std::string w; words >> w;) {
      if (const std::size_t at = w.find("%s"); at != std::string::npos) {
        w.replace(at, 2, url);
        placed = true;
      }
      argv.push_back(w);
    }
    if (argv.empty())
      continue;
    if (!placed)
      argv.push_back(url);
#if defined(_WIN32)
    std::string line;
    for (const std::string &a : argv)
      line += (line.empty() ? "" : " ") + std::string("\"") + a + "\"";
    STARTUPINFOA si{};
    si.cb = sizeof si;
    PROCESS_INFORMATION pi{};
    if (CreateProcessA(nullptr, line.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr,
                       nullptr, &si, &pi)) {
      CloseHandle(pi.hThread);
      CloseHandle(pi.hProcess);
      return true;
    }
#else
    if (spawn_quiet(argv))
      return true;
#endif
  }
  return false;
}
#endif

} // namespace

bool browser_available() {
  if (g_disabled.load())
    return false;
  {
    std::lock_guard<std::mutex> lock(hooks_mutex());
    if (opener_cell())
      return true;
  }
#if defined(__EMSCRIPTEN__)
  return true;
#else
  if (env_set("BROWSER"))
    return true;
  if (over_ssh())
    return false;
#if defined(_WIN32) || defined(__APPLE__)
  return true;
#else
  return env_set("DISPLAY") || env_set("WAYLAND_DISPLAY");
#endif
#endif
}

void set_browser_disabled(bool disabled) { g_disabled.store(disabled); }

OpenUrlResult open_url(const std::string &url, std::string *error) {
  const auto failed = [&](std::string why) {
    if (error)
      *error = std::move(why);
    return OpenUrlResult::failed;
  };
  std::string why;
  if (!parse_origin(url, &why))
    return failed("open-url: " + why);
  if (url.find_first_of(" \t\r\n\"'`\\<>") != std::string::npos)
    return failed("open-url: the URL has a character a URL may not");
  if (!browser_available())
    return OpenUrlResult::no_browser;
  std::function<bool(const std::string &)> opener;
  {
    std::lock_guard<std::mutex> lock(hooks_mutex());
    opener = opener_cell();
  }
  if (opener)
    return opener(url) ? OpenUrlResult::opened : failed("open-url: the browser did not start");
#if defined(__EMSCRIPTEN__)
  return cvc_net_window_open(url.c_str()) ? OpenUrlResult::opened
                                          : failed("open-url: the browser blocked the new window");
#else
  if (env_set("BROWSER"))
    return open_with_browser_env(url) ? OpenUrlResult::opened
                                      : failed("open-url: no command in $BROWSER started");
#if defined(_WIN32)
  const int n = MultiByteToWideChar(CP_UTF8, 0, url.c_str(), -1, nullptr, 0);
  std::wstring wide(static_cast<std::size_t>(n > 0 ? n : 1), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, url.c_str(), -1, wide.data(), n);
  const HINSTANCE h =
      ShellExecuteW(nullptr, L"open", wide.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
  return reinterpret_cast<INT_PTR>(h) > 32 ? OpenUrlResult::opened
                                           : failed("open-url: ShellExecute failed");
#elif defined(__APPLE__)
  return spawn_quiet({"open", url}) ? OpenUrlResult::opened
                                    : failed("open-url: 'open' did not start");
#else
  return spawn_quiet({"xdg-open", url}) ? OpenUrlResult::opened
                                        : failed("open-url: xdg-open did not start");
#endif
#endif
}

void set_url_opener(std::function<bool(const std::string &url)> opener) {
  std::lock_guard<std::mutex> lock(hooks_mutex());
  opener_cell() = std::move(opener);
}

void set_url_notice(std::function<void(const std::string &url)> notice) {
  std::lock_guard<std::mutex> lock(hooks_mutex());
  notice_cell() = std::move(notice);
}

void url_notice(const std::string &url) {
  std::function<void(const std::string &)> notice;
  {
    std::lock_guard<std::mutex> lock(hooks_mutex());
    notice = notice_cell();
  }
  if (notice) {
    notice(url);
    return;
  }
  std::fprintf(stderr, "Open this URL in a browser:\n  %s\n", url.c_str());
  std::fflush(stderr);
}

namespace page {

namespace {
std::shared_ptr<Hooks> &hooks_cell() {
  static std::shared_ptr<Hooks> h;
  return h;
}
std::shared_ptr<Hooks> hooks() {
  std::lock_guard<std::mutex> lock(hooks_mutex());
  return hooks_cell();
}
#if defined(__EMSCRIPTEN__)
std::string take(char *p) {
  std::string out = p ? p : "";
  std::free(p);
  return out;
}
#endif
} // namespace

void set_hooks(std::shared_ptr<Hooks> h) {
  std::lock_guard<std::mutex> lock(hooks_mutex());
  hooks_cell() = std::move(h);
}

bool present() {
#if defined(__EMSCRIPTEN__)
  return true;
#else
  return hooks() != nullptr;
#endif
}

std::string url() {
  if (const auto h = hooks())
    return h->url ? h->url() : std::string();
#if defined(__EMSCRIPTEN__)
  return take(cvc_net_page_url());
#else
  return std::string();
#endif
}

std::string session_get(const std::string &key) {
  if (const auto h = hooks())
    return h->session_get ? h->session_get(key) : std::string();
#if defined(__EMSCRIPTEN__)
  return take(cvc_net_session_get(key.c_str()));
#else
  return std::string();
#endif
}

bool session_set(const std::string &key, const std::string &value) {
  if (const auto h = hooks())
    return h->session_set && h->session_set(key, value);
#if defined(__EMSCRIPTEN__)
  return cvc_net_session_set(key.c_str(), value.c_str()) != 0;
#else
  return false;
#endif
}

void session_remove(const std::string &key) {
  if (const auto h = hooks()) {
    if (h->session_remove)
      h->session_remove(key);
    return;
  }
#if defined(__EMSCRIPTEN__)
  cvc_net_session_remove(key.c_str());
#endif
}

bool navigate(const std::string &url) {
  if (const auto h = hooks())
    return h->navigate && h->navigate(url);
#if defined(__EMSCRIPTEN__)
  return cvc_net_navigate(url.c_str()) != 0;
#else
  return false;
#endif
}

bool replace_url(const std::string &url) {
  if (const auto h = hooks())
    return h->replace_url && h->replace_url(url);
#if defined(__EMSCRIPTEN__)
  return cvc_net_replace_url(url.c_str()) != 0;
#else
  return false;
#endif
}

} // namespace page

} // namespace net
} // namespace cvc

// cvc::net — origins, the allowlist and the policy file. See inc/cvc/net/net_policy.h.

#include <algorithm>
#include <cvc/net/net_policy.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <cstdlib>
#include <emscripten/emscripten.h>

// The page's origin, malloc'd (the caller frees it). `location` exists on the main thread and in
// workers alike (WorkerLocation). The same string-return pattern as cvcGL's clipboard.
EM_JS(char *, cvc_net_page_origin, (), {
  var o = (typeof location != = "undefined" && location.origin) ? location.origin : "";
  var n = lengthBytesUTF8(o) + 1;
  var p = _malloc(n);
  stringToUTF8(o, p, n);
  return p;
});
#endif

namespace cvc {
namespace net {

namespace {

char lower(char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; }

std::string lowered(std::string s) {
  for (char &c : s)
    c = lower(c);
  return s;
}

int default_port(const std::string &scheme) { return scheme == "https" ? 443 : 80; }

bool fail(std::string *error, std::string message) {
  if (error)
    *error = std::move(message);
  return false;
}

bool host_char(char c) {
  return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '.';
}

bool hex_or_colon(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || c == ':' || c == '.';
}

// Split "scheme://authority<rest>" and check the authority. `rest` is what follows it (a path,
// query or fragment, starting with '/', '?' or '#'), or "".
std::optional<Origin> parse(const std::string &url, std::string *rest, std::string *error) {
  for (unsigned char c : url)
    if (c <= 0x20 || c == 0x7f || c == '\\') {
      fail(error, "a URL may not hold spaces, control bytes or backslashes");
      return std::nullopt;
    }
  const std::size_t sep = url.find("://");
  if (sep == std::string::npos || sep == 0) {
    fail(error, "'" + url + "' is not an http(s) URL");
    return std::nullopt;
  }
  Origin o;
  o.scheme = lowered(url.substr(0, sep));
  if (o.scheme != "http" && o.scheme != "https") {
    fail(error, "'" + url + "' is not an http(s) URL");
    return std::nullopt;
  }
  const std::size_t start = sep + 3;
  const std::size_t end = std::min(url.find_first_of("/?#", start), url.size());
  const std::string authority = url.substr(start, end - start);
  if (rest)
    *rest = url.substr(end);
  if (authority.find('@') != std::string::npos) {
    fail(error, "'" + url + "' has user info; a URL may not carry credentials");
    return std::nullopt;
  }
  std::string host = authority, port;
  if (!authority.empty() && authority[0] == '[') { // [IPv6]
    const std::size_t close = authority.find(']');
    if (close == std::string::npos) {
      fail(error, "'" + url + "' has an unterminated IPv6 address");
      return std::nullopt;
    }
    host = lowered(authority.substr(0, close + 1));
    const std::string after = authority.substr(close + 1);
    if (!after.empty() && after[0] != ':') {
      fail(error, "'" + url + "' has a malformed host");
      return std::nullopt;
    }
    if (!after.empty())
      port = after.substr(1);
    if (host.size() < 3 || !std::all_of(host.begin() + 1, host.end() - 1, hex_or_colon)) {
      fail(error, "'" + url + "' has a malformed IPv6 address");
      return std::nullopt;
    }
  } else {
    const std::size_t colon = authority.rfind(':');
    if (colon != std::string::npos) {
      host = authority.substr(0, colon);
      port = authority.substr(colon + 1);
    }
    host = lowered(host);
    if (host.empty() || !std::all_of(host.begin(), host.end(), host_char) || host.front() == '.' ||
        host.find("..") != std::string::npos) {
      fail(error, "'" + url + "' has a malformed host (use the punycode form of a non-ASCII name)");
      return std::nullopt;
    }
  }
  o.host = std::move(host);
  o.port = default_port(o.scheme);
  if (!port.empty()) {
    if (port.size() > 5 ||
        !std::all_of(port.begin(), port.end(), [](char c) { return c >= '0' && c <= '9'; })) {
      fail(error, "'" + url + "' has a malformed port");
      return std::nullopt;
    }
    const int p = std::stoi(port);
    if (p < 1 || p > 65535) {
      fail(error, "'" + url + "' has a port outside 1-65535");
      return std::nullopt;
    }
    o.port = p;
  }
  return o;
}

std::string trimmed(const std::string &s) {
  const std::size_t b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos)
    return std::string();
  const std::size_t e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

} // namespace

std::string Origin::str() const {
  std::string s = scheme + "://" + host;
  if (port != default_port(scheme))
    s += ":" + std::to_string(port);
  return s;
}

std::optional<Origin> parse_origin(const std::string &url, std::string *error) {
  return parse(url, nullptr, error);
}

std::optional<Origin> parse_origin_spec(const std::string &spec, std::string *error) {
  std::string rest;
  std::optional<Origin> o = parse(spec, &rest, error);
  if (o && !rest.empty() && rest != "/") {
    fail(error, "'" + spec + "' is a URL, not an origin (write scheme://host[:port])");
    return std::nullopt;
  }
  return o;
}

void NetPolicy::allow(const Origin &o) {
  if (!allows(o))
    allowed_.push_back(o);
}

bool NetPolicy::allows(const Origin &o) const {
  return std::find(allowed_.begin(), allowed_.end(), o) != allowed_.end();
}

bool NetPolicy::allows_url(const std::string &url, std::string *why) const {
  const std::optional<Origin> o = parse_origin(url, why);
  if (!o)
    return false;
  if (!allows(*o))
    return fail(why, o->str() + " is not an origin this host allows");
  return true;
}

void NetPolicy::bind_credential(const std::string &name, const Origin &o) {
  std::vector<Origin> &v = bindings_[name];
  if (std::find(v.begin(), v.end(), o) == v.end())
    v.push_back(o);
}

bool parse_net_policy(const std::string &text, NetPolicy &into, std::string *error,
                      const std::string &source) {
  std::istringstream in(text);
  std::string line;
  for (int n = 1; std::getline(in, line); ++n) {
    if (const std::size_t hash = line.find('#'); hash != std::string::npos)
      line.resize(hash);
    std::istringstream words(trimmed(line));
    std::vector<std::string> w;
    for (std::string word; words >> word;)
      w.push_back(word);
    if (w.empty())
      continue;
    const std::string where = source + ":" + std::to_string(n) + ": ";
    std::string why;
    if (w[0] == "allow") {
      if (w.size() != 2)
        return fail(error, where + "allow takes one origin");
      const std::optional<Origin> o = parse_origin_spec(w[1], &why);
      if (!o)
        return fail(error, where + why);
      into.allow(*o);
    } else if (w[0] == "credential") {
      if (w.size() < 3)
        return fail(error, where + "credential takes a name and at least one origin");
      for (std::size_t i = 2; i < w.size(); ++i) {
        const std::optional<Origin> o = parse_origin_spec(w[i], &why);
        if (!o)
          return fail(error, where + why);
        into.bind_credential(w[1], *o);
      }
    } else {
      return fail(error, where + "unknown directive '" + w[0] + "' (allow or credential)");
    }
  }
  return true;
}

bool load_net_policy_file(const std::string &path, NetPolicy &into, std::string *error) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    return fail(error, "cannot read the net policy '" + path + "'");
  std::ostringstream text;
  text << in.rdbuf();
  return parse_net_policy(text.str(), into, error, path);
}

std::string page_origin() {
#ifdef __EMSCRIPTEN__
  char *s = cvc_net_page_origin();
  std::string out = s ? s : "";
  std::free(s);
  return parse_origin_spec(out) ? out : std::string();
#else
  return std::string();
#endif
}

} // namespace net
} // namespace cvc

// cvc::net — the host's credential store, and the JSON capture and redaction it uses. See
// inc/cvc/net/credentials.h.

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cvc/net/credentials.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace cvc {
namespace net {

namespace fs = std::filesystem;

namespace {

bool fail(std::string *error, std::string message) {
  if (error)
    *error = std::move(message);
  return false;
}

bool name_char(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' ||
         c == '_' || c == '-';
}

constexpr char kFileHeader[] = "# cvc credentials v1: <name> <origin> <token> a line. Owner-only;\n"
                               "# these are secrets. Written by the host; do not edit.\n";

// --- a JSON scan just deep enough to find one string ------------------------------------------

constexpr int kMaxJsonDepth = 64;

class JsonScan {
public:
  explicit JsonScan(const std::string &s) : s_(s) {}

  // Find the string at `path` in the object starting at the first non-space byte.
  bool find(const std::vector<std::string> &path, std::size_t &begin, std::size_t &end,
            std::string &value) {
    ws();
    return object(path, 0, 0, begin, end, value);
  }

private:
  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r'))
      ++i_;
  }

  static void put_utf8(std::string &out, unsigned cp) {
    if (cp < 0x80) {
      out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
      out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
      out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
      out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  }

  bool hex4(unsigned &cp) {
    if (i_ + 4 > s_.size())
      return false;
    cp = 0;
    for (int k = 0; k < 4; ++k) {
      const char c = s_[i_++];
      cp <<= 4;
      if (c >= '0' && c <= '9')
        cp |= static_cast<unsigned>(c - '0');
      else if (c >= 'a' && c <= 'f')
        cp |= static_cast<unsigned>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F')
        cp |= static_cast<unsigned>(c - 'A' + 10);
      else
        return false;
    }
    return true;
  }

  // A string literal at i_, decoded into `out` (when given).
  bool string(std::string *out) {
    if (i_ >= s_.size() || s_[i_] != '"')
      return false;
    ++i_;
    while (i_ < s_.size()) {
      const char c = s_[i_++];
      if (c == '"')
        return true;
      if (static_cast<unsigned char>(c) < 0x20)
        return false;
      if (c != '\\') {
        if (out)
          out->push_back(c);
        continue;
      }
      if (i_ >= s_.size())
        return false;
      const char e = s_[i_++];
      char plain = 0;
      switch (e) {
      case '"':
      case '\\':
      case '/':
        plain = e;
        break;
      case 'b':
        plain = '\b';
        break;
      case 'f':
        plain = '\f';
        break;
      case 'n':
        plain = '\n';
        break;
      case 'r':
        plain = '\r';
        break;
      case 't':
        plain = '\t';
        break;
      case 'u': {
        unsigned cp = 0;
        if (!hex4(cp))
          return false;
        if (cp >= 0xD800 && cp < 0xDC00) { // a high surrogate wants its low half
          unsigned lo = 0;
          if (i_ + 2 > s_.size() || s_[i_] != '\\' || s_[i_ + 1] != 'u')
            return false;
          i_ += 2;
          if (!hex4(lo) || lo < 0xDC00 || lo >= 0xE000)
            return false;
          cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
        }
        if (out)
          put_utf8(*out, cp);
        continue;
      }
      default:
        return false;
      }
      if (out)
        out->push_back(plain);
    }
    return false;
  }

  bool literal(const char *word) {
    const std::string w(word);
    if (s_.compare(i_, w.size(), w) != 0)
      return false;
    i_ += w.size();
    return true;
  }

  bool number() {
    const std::size_t start = i_;
    while (i_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '-' ||
                              s_[i_] == '+' || s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E'))
      ++i_;
    return i_ > start;
  }

  // Skip any value at i_.
  bool value(int depth) {
    if (depth > kMaxJsonDepth || i_ >= s_.size())
      return false;
    const char c = s_[i_];
    if (c == '"')
      return string(nullptr);
    if (c == '{')
      return skip_object(depth);
    if (c == '[') {
      ++i_;
      ws();
      if (i_ < s_.size() && s_[i_] == ']') {
        ++i_;
        return true;
      }
      for (;;) {
        ws();
        if (!value(depth + 1))
          return false;
        ws();
        if (i_ >= s_.size())
          return false;
        if (s_[i_] == ',') {
          ++i_;
          continue;
        }
        if (s_[i_] == ']') {
          ++i_;
          return true;
        }
        return false;
      }
    }
    if (c == 't')
      return literal("true");
    if (c == 'f')
      return literal("false");
    if (c == 'n')
      return literal("null");
    return number();
  }

  // Skip the object at i_.
  bool skip_object(int depth) {
    if (depth > kMaxJsonDepth || i_ >= s_.size() || s_[i_] != '{')
      return false;
    ++i_;
    ws();
    if (i_ < s_.size() && s_[i_] == '}') {
      ++i_;
      return true;
    }
    for (;;) {
      ws();
      if (!string(nullptr))
        return false;
      ws();
      if (i_ >= s_.size() || s_[i_] != ':')
        return false;
      ++i_;
      ws();
      if (!value(depth + 1))
        return false;
      ws();
      if (i_ >= s_.size())
        return false;
      if (s_[i_] == ',') {
        ++i_;
        continue;
      }
      if (s_[i_] == '}') {
        ++i_;
        return true;
      }
      return false;
    }
  }

  // Find the string at path[at...] in the object at i_: its literal's byte range and its decoded
  // value. The first key that matches decides; false when it names no string (or the text is
  // malformed before it).
  bool object(const std::vector<std::string> &path, std::size_t at, int depth, std::size_t &begin,
              std::size_t &end, std::string &value_out) {
    if (depth > kMaxJsonDepth || i_ >= s_.size() || s_[i_] != '{')
      return false;
    ++i_;
    for (;;) {
      ws();
      if (i_ < s_.size() && s_[i_] == '}')
        return false;
      std::string key;
      if (!string(&key))
        return false;
      ws();
      if (i_ >= s_.size() || s_[i_] != ':')
        return false;
      ++i_;
      ws();
      if (key == path[at]) {
        if (at + 1 < path.size())
          return object(path, at + 1, depth + 1, begin, end, value_out);
        begin = i_;
        std::string v;
        if (!string(&v))
          return false; // the field is there but not a string
        end = i_;
        value_out = std::move(v);
        return true;
      }
      if (!value(depth + 1))
        return false;
      ws();
      if (i_ >= s_.size() || s_[i_] != ',')
        return false; // '}' (not found) or malformed
      ++i_;
    }
  }

  const std::string &s_;
  std::size_t i_ = 0;
};

#if !defined(__EMSCRIPTEN__)
// The user's config directory for cvc, as device keys find it (state_device_key.cpp).
fs::path config_dir() {
#if defined(_WIN32)
  if (const wchar_t *dir = ::_wgetenv(L"LOCALAPPDATA"); dir != nullptr && *dir != L'\0')
    return fs::path(dir) / L"cvc";
  return {};
#else
  if (const char *xdg = std::getenv("XDG_CONFIG_HOME"); xdg != nullptr && *xdg != '\0')
    return fs::path(xdg) / "cvc";
  if (const char *home = std::getenv("HOME"); home != nullptr && *home != '\0')
    return fs::path(home) / ".config" / "cvc";
  return {};
#endif
}

// Write `text` to `path`, replacing it in one step, owner-only.
bool write_private_file(const fs::path &path, const std::string &text, std::string *error) {
  std::error_code ec;
  const fs::path dir = path.parent_path();
  if (!dir.empty() && !fs::exists(dir, ec)) {
    fs::create_directories(dir, ec);
#if !defined(_WIN32)
    ::chmod(dir.c_str(), 0700); // ours, so nobody else lists it
#endif
  }
#if defined(_WIN32)
  fs::path tmp = path;
  tmp += L".tmp." + std::to_wstring(::GetCurrentProcessId());
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out << text;
    if (!out.flush()) {
      fs::remove(tmp, ec);
      return fail(error, "could not write the credential store '" + path.string() + "'");
    }
  }
  if (::MoveFileExW(tmp.c_str(), path.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    fs::remove(tmp, ec);
    return fail(error, "could not replace the credential store '" + path.string() + "'");
  }
  return true;
#else
  const std::string tmp = path.string() + ".tmp." + std::to_string(::getpid());
  ::unlink(tmp.c_str());
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd < 0)
    return fail(error, "could not write the credential store '" + path.string() + "'");
  bool ok = ::fchmod(fd, 0600) == 0; // whatever the umask
  ok = ok && ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size());
  ok = ok && ::fsync(fd) == 0;
  ok = ::close(fd) == 0 && ok;
  ok = ok && ::rename(tmp.c_str(), path.c_str()) == 0;
  if (!ok) {
    ::unlink(tmp.c_str());
    return fail(error, "could not write the credential store '" + path.string() + "'");
  }
  return true;
#endif
}
#endif

std::vector<std::string> split_path(const std::string &field) {
  std::vector<std::string> out;
  std::string part;
  std::istringstream in(field);
  while (std::getline(in, part, '.'))
    out.push_back(part);
  return out;
}

} // namespace

CredentialStore::CredentialStore(std::string path) : path_(std::move(path)) {}

bool CredentialStore::valid_name(const std::string &name) {
  return !name.empty() && name.size() <= 64 && std::all_of(name.begin(), name.end(), name_char);
}

bool CredentialStore::valid_token(const std::string &token) {
  std::size_t n = token.size();
  while (n > 0 && token[n - 1] == '=')
    --n;
  if (n < 16 || token.size() > 4096)
    return false;
  return std::all_of(token.begin(), token.begin() + static_cast<std::ptrdiff_t>(n), [](char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
           c == '.' || c == '_' || c == '~' || c == '+' || c == '/';
  });
}

std::string CredentialStore::resolve_path(const std::string &spec) {
#if defined(__EMSCRIPTEN__)
  (void)spec;
  return std::string();
#else
  if (!valid_name(spec) || spec == "." || spec == "..")
    return spec;
  const fs::path dir = config_dir();
  if (dir.empty())
    return std::string();
  return (dir / "credentials" / spec).string();
#endif
}

bool CredentialStore::load(std::string *error) {
  if (path_.empty())
    return true;
#if defined(__EMSCRIPTEN__)
  return fail(error, "a browser host keeps credentials in memory only");
#else
  std::error_code ec;
  if (!fs::exists(path_, ec))
    return true;
#if !defined(_WIN32)
  struct stat st {};
  if (::stat(path_.c_str(), &st) != 0)
    return fail(error, "cannot read the credential store '" + path_ + "'");
  if (st.st_uid != ::geteuid())
    return fail(error, "the credential store '" + path_ + "' belongs to another user");
  if ((st.st_mode & 077) != 0)
    return fail(error, "the credential store '" + path_ +
                           "' is open to other users; it must be owner-only (chmod 600)");
#endif
  std::ifstream in(path_, std::ios::binary);
  if (!in)
    return fail(error, "cannot read the credential store '" + path_ + "'");
  std::vector<Entry> loaded;
  std::string line;
  for (int n = 1; std::getline(in, line); ++n) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream words(line);
    Entry e;
    std::string origin, extra;
    // Never quote the line: it holds a token.
    const std::string where = "the credential store '" + path_ + "' line " + std::to_string(n);
    if (!(words >> e.name >> origin >> e.token) || (words >> extra))
      return fail(error, where + " is not '<name> <origin> <token>'");
    const std::optional<Origin> o = parse_origin_spec(origin);
    if (!valid_name(e.name) || !o || !valid_token(e.token))
      return fail(error, where + " is malformed");
    e.from = *o;
    loaded.push_back(std::move(e));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  entries_ = std::move(loaded);
  return true;
#endif
}

bool CredentialStore::save_locked(std::string *error) const {
  if (path_.empty())
    return true;
#if defined(__EMSCRIPTEN__)
  return fail(error, "a browser host keeps credentials in memory only");
#else
  std::string text = kFileHeader;
  for (const Entry &e : entries_)
    if (e.kept)
      text += e.name + " " + e.from.str() + " " + e.token + "\n";
  return write_private_file(fs::path(path_), text, error);
#endif
}

void CredentialStore::bind(const std::string &name, std::vector<Origin> origins) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto &b : bindings_)
    if (b.first == name) {
      b.second = std::move(origins);
      return;
    }
  bindings_.emplace_back(name, std::move(origins));
}

std::vector<Origin> CredentialStore::binding(const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &b : bindings_)
    if (b.first == name)
      return b.second;
  return {};
}

// Whether entry `e` (of credential `name`) is sent to `to`. mutex_ held.
bool CredentialStore::sends_to(const std::string &name, const Entry &e, const Origin &to) const {
  for (const auto &b : bindings_)
    if (b.first == name) {
      const auto has = [&](const Origin &o) {
        return std::find(b.second.begin(), b.second.end(), o) != b.second.end();
      };
      return has(e.from) && has(to); // an entry from a since-dropped origin is not used
    }
  return e.from == to;
}

bool CredentialStore::capture(const std::string &name, const Origin &from, const std::string &token,
                              std::string *error) {
  if (!valid_name(name))
    return fail(error, "'" + name + "' is not a credential name (1-64 of A-Z a-z 0-9 . _ -)");
  if (!valid_token(token))
    return fail(error, "the value is not a bearer token (16-4096 of A-Z a-z 0-9 - . _ ~ + /)");
  std::lock_guard<std::mutex> lock(mutex_);
  bool bound = false;
  for (const auto &b : bindings_)
    if (b.first == name) {
      bound = true;
      if (std::find(b.second.begin(), b.second.end(), from) == b.second.end())
        return fail(error, "credential '" + name + "' is not bound to " + from.str());
    }
  std::erase_if(entries_,
                [&](const Entry &e) { return e.name == name && (bound || e.from == from); });
  entries_.push_back({name, from, token});
  std::string werr;
  if (!save_locked(&werr) && error)
    *error = werr;
  return true;
}

bool CredentialStore::provide(const std::string &name, const std::string &token,
                              std::string *error) {
  if (!valid_name(name))
    return fail(error, "'" + name + "' is not a credential name (1-64 of A-Z a-z 0-9 . _ -)");
  if (!valid_token(token))
    return fail(error, "the value is not a bearer token (16-4096 of A-Z a-z 0-9 - . _ ~ + /)");
  std::lock_guard<std::mutex> lock(mutex_);
  const auto bound = std::find_if(bindings_.begin(), bindings_.end(),
                                  [&](const auto &b) { return b.first == name; });
  if (bound == bindings_.end() || bound->second.empty())
    return fail(error, "credential '" + name + "' is bound to no origin");
  // The file's token for it stays on file: this one is ahead of it, for this run.
  std::erase_if(entries_, [&](const Entry &e) { return e.name == name && !e.kept; });
  entries_.push_back({name, bound->second.front(), token, false});
  return true;
}

std::optional<std::string> CredentialStore::token_for(const std::string &name,
                                                      const Origin &to) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it)
    if (it->name == name && sends_to(name, *it, to))
      return it->token;
  return std::nullopt;
}

bool CredentialStore::has(const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::any_of(entries_.begin(), entries_.end(),
                     [&](const Entry &e) { return e.name == name; });
}

std::vector<std::string> CredentialStore::names() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> out;
  for (const Entry &e : entries_)
    if (std::find(out.begin(), out.end(), e.name) == out.end())
      out.push_back(e.name);
  std::sort(out.begin(), out.end());
  return out;
}

bool CredentialStore::forget(const std::string &name, std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t dropped =
      std::erase_if(entries_, [&](const Entry &e) { return e.name == name; });
  if (dropped > 0)
    save_locked(error);
  return dropped > 0;
}

void CredentialStore::clear(std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
  save_locked(error);
}

bool take_json_string(std::string &body, const std::string &field, std::string &token) {
  const std::vector<std::string> path = split_path(field);
  if (path.empty() ||
      std::any_of(path.begin(), path.end(), [](const std::string &p) { return p.empty(); }))
    return false;
  std::size_t begin = 0, end = 0;
  std::string value;
  JsonScan scan(body);
  if (!scan.find(path, begin, end, value))
    return false;
  token = std::move(value);
  body.replace(begin, end - begin, std::string("\"") + kRedacted + "\"");
  return true;
}

std::size_t redact(std::string &text, const std::string &secret) {
  if (secret.empty())
    return 0;
  std::size_t n = 0;
  for (std::size_t at = text.find(secret); at != std::string::npos;
       at = text.find(secret, at + sizeof(kRedacted) - 1)) {
    text.replace(at, secret.size(), kRedacted);
    ++n;
  }
  return n;
}

} // namespace net
} // namespace cvc

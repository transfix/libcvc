// cvc::net — the host's credential store, and the JSON capture and redaction it uses. See
// inc/cvc/net/credentials.h.

#include "private_file.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cvc/net/credentials.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

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

constexpr char kFileHeader[] =
    "# cvc credentials v2: <principal> <name> <origin> <token> a line, the principal\n"
    "# percent-encoded; a line of three words is the local principal's. Owner-only;\n"
    "# these are secrets. Written by the host; do not edit.\n";

// A principal's longest form (a file: directory's path, a remote: origin).
constexpr std::size_t kMaxPrincipal = 4096;

// A principal as a word of the file: every byte but [A-Za-z0-9] and ":/._~-" as %XX, so a
// directory with a space in it stays one word.
std::string encode_principal(const std::string &p) {
  static const char kHex[] = "0123456789ABCDEF";
  std::string out;
  for (const char c : p) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (std::isalnum(u) || c == ':' || c == '/' || c == '.' || c == '_' || c == '~' || c == '-') {
      out.push_back(c);
    } else {
      out.push_back('%');
      out.push_back(kHex[u >> 4]);
      out.push_back(kHex[u & 15]);
    }
  }
  return out;
}

// encode_principal undone; "" when a %XX is malformed.
std::string decode_principal(const std::string &w) {
  const auto hex = [](char c) {
    return c >= '0' && c <= '9'   ? c - '0'
           : c >= 'A' && c <= 'F' ? c - 'A' + 10
           : c >= 'a' && c <= 'f' ? c - 'a' + 10
                                  : -1;
  };
  std::string out;
  for (std::size_t i = 0; i < w.size(); ++i) {
    if (w[i] != '%') {
      out.push_back(w[i]);
      continue;
    }
    if (i + 2 >= w.size() || hex(w[i + 1]) < 0 || hex(w[i + 2]) < 0)
      return std::string();
    out.push_back(static_cast<char>(hex(w[i + 1]) * 16 + hex(w[i + 2])));
    i += 2;
  }
  return out;
}

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

std::vector<std::string> split_path(const std::string &field) {
  std::vector<std::string> out;
  std::string part;
  std::istringstream in(field);
  while (std::getline(in, part, '.'))
    out.push_back(part);
  return out;
}

} // namespace

// One store's state, which every principal's view of it shares (credentials.h).
struct CredentialStore::Shared {
  struct Entry {
    std::string principal;
    std::string name;
    Origin from;
    std::string token;
    bool kept = true; // written to the file (false: provided, this run only)
  };
  // What one principal has besides its entries.
  struct Own {
    std::vector<std::pair<std::string, std::vector<Origin>>> bindings;
    std::vector<std::pair<std::string, std::shared_ptr<CredentialRenewer>>> renewers;
  };

  explicit Shared(std::string p) : path(std::move(p)) {}

  const std::string path;
  std::mutex mutex; // everything below
  std::vector<Entry> entries;
  std::map<std::string, Own> own; // by principal

  const std::vector<Origin> *bound(const std::string &principal, const std::string &name) const {
    const auto o = own.find(principal);
    if (o == own.end())
      return nullptr;
    for (const auto &b : o->second.bindings)
      if (b.first == name)
        return &b.second;
    return nullptr;
  }

  // Whether entry `e` is sent to `to`. mutex held.
  bool sends_to(const Entry &e, const Origin &to) const {
    if (const std::vector<Origin> *b = bound(e.principal, e.name)) {
      const auto has = [&](const Origin &o) {
        return std::find(b->begin(), b->end(), o) != b->end();
      };
      return has(e.from) && has(to); // an entry from a since-dropped origin is not used
    }
    return e.from == to;
  }

  bool save_locked(std::string *error) const {
    if (path.empty())
      return true;
#if defined(__EMSCRIPTEN__)
    return fail(error, "a browser host keeps credentials in memory only");
#else
    std::string text = kFileHeader;
    for (const Entry &e : entries)
      if (e.kept)
        text += encode_principal(e.principal) + " " + e.name + " " + e.from.str() + " " + e.token +
                "\n";
    return private_file::write(fs::path(path), text, "the credential store", error);
#endif
  }
};

CredentialStore::CredentialStore(std::string path)
    : shared_(std::make_shared<Shared>(std::move(path))), principal_(kLocalPrincipal) {}

CredentialStore::CredentialStore(std::shared_ptr<Shared> shared, std::string principal)
    : shared_(std::move(shared)), principal_(std::move(principal)) {}

std::shared_ptr<CredentialStore> CredentialStore::for_principal(const std::string &principal) {
  std::string p = canonical_principal(principal);
  if (p.empty())
    return nullptr;
  return std::shared_ptr<CredentialStore>(new CredentialStore(shared_, std::move(p)));
}

const std::string &CredentialStore::path() const { return shared_->path; }

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

bool CredentialStore::valid_principal(const std::string &principal) {
  return !canonical_principal(principal).empty();
}

std::string CredentialStore::canonical_principal(const std::string &principal) {
  if (principal == kLocalPrincipal)
    return principal;
  constexpr std::string_view kFile = "file:", kRemote = "remote:";
  if (principal.size() > kMaxPrincipal)
    return std::string();
  if (principal.compare(0, kFile.size(), kFile) == 0) {
    if (principal.size() == kFile.size() ||
        std::any_of(principal.begin(), principal.end(),
                    [](char c) { return static_cast<unsigned char>(c) < 0x20 || c == 0x7f; }))
      return std::string();
    return principal;
  }
  if (principal.compare(0, kRemote.size(), kRemote) == 0) {
    const std::optional<Origin> o = parse_origin_spec(principal.substr(kRemote.size()));
    return o ? std::string(kRemote) + o->str() : std::string();
  }
  return std::string();
}

std::string CredentialStore::resolve_path(const std::string &spec) {
#if defined(__EMSCRIPTEN__)
  (void)spec;
  return std::string();
#else
  if (!valid_name(spec) || spec == "." || spec == "..")
    return spec;
  const fs::path dir = private_file::config_dir();
  if (dir.empty())
    return std::string();
  return (dir / "credentials" / spec).string();
#endif
}

bool CredentialStore::load(std::string *error) {
  const std::string &file = shared_->path;
  if (file.empty())
    return true;
#if defined(__EMSCRIPTEN__)
  return fail(error, "a browser host keeps credentials in memory only");
#else
  std::error_code ec;
  if (!fs::exists(file, ec))
    return true;
  if (!private_file::check_owner_only(file, "the credential store", error))
    return false;
  std::ifstream in(file, std::ios::binary);
  if (!in)
    return fail(error, "cannot read the credential store '" + file + "'");
  std::vector<Shared::Entry> loaded;
  std::string line;
  for (int n = 1; std::getline(in, line); ++n) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream words_in(line);
    std::vector<std::string> words;
    for (std::string w; words_in >> w;)
      words.push_back(std::move(w));
    // Never quote the line: it holds a token.
    const std::string where = "the credential store '" + file + "' line " + std::to_string(n);
    if (words.size() != 3 && words.size() != 4)
      return fail(error, where + " is not '<principal> <name> <origin> <token>'");
    // Three words: a line written before principals were kept, the local principal's.
    const std::size_t at = words.size() - 3;
    Shared::Entry e;
    e.principal =
        at == 0 ? std::string(kLocalPrincipal) : canonical_principal(decode_principal(words[0]));
    e.name = words[at];
    e.token = words[at + 2];
    const std::optional<Origin> o = parse_origin_spec(words[at + 1]);
    if (e.principal.empty() || !valid_name(e.name) || !o || !valid_token(e.token))
      return fail(error, where + " is malformed");
    e.from = *o;
    loaded.push_back(std::move(e));
  }
  std::lock_guard<std::mutex> lock(shared_->mutex);
  shared_->entries = std::move(loaded);
  return true;
#endif
}

void CredentialStore::bind(const std::string &name, std::vector<Origin> origins) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  auto &bindings = shared_->own[principal_].bindings;
  for (auto &b : bindings)
    if (b.first == name) {
      b.second = std::move(origins);
      return;
    }
  bindings.emplace_back(name, std::move(origins));
}

std::vector<Origin> CredentialStore::binding(const std::string &name) const {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  const std::vector<Origin> *b = shared_->bound(principal_, name);
  return b ? *b : std::vector<Origin>{};
}

bool CredentialStore::capture(const std::string &name, const Origin &from, const std::string &token,
                              std::string *error) {
  if (!valid_name(name))
    return fail(error, "'" + name + "' is not a credential name (1-64 of A-Z a-z 0-9 . _ -)");
  if (!valid_token(token))
    return fail(error, "the value is not a bearer token (16-4096 of A-Z a-z 0-9 - . _ ~ + /)");
  std::lock_guard<std::mutex> lock(shared_->mutex);
  const std::vector<Origin> *b = shared_->bound(principal_, name);
  if (b && std::find(b->begin(), b->end(), from) == b->end())
    return fail(error, "credential '" + name + "' is not bound to " + from.str());
  const bool bound = b != nullptr;
  std::erase_if(shared_->entries, [&](const Shared::Entry &e) {
    return e.principal == principal_ && e.name == name && (bound || e.from == from);
  });
  shared_->entries.push_back({principal_, name, from, token});
  std::string werr;
  if (!shared_->save_locked(&werr) && error)
    *error = werr;
  return true;
}

bool CredentialStore::provide(const std::string &name, const std::string &token,
                              std::string *error) {
  if (!valid_name(name))
    return fail(error, "'" + name + "' is not a credential name (1-64 of A-Z a-z 0-9 . _ -)");
  if (!valid_token(token))
    return fail(error, "the value is not a bearer token (16-4096 of A-Z a-z 0-9 - . _ ~ + /)");
  std::lock_guard<std::mutex> lock(shared_->mutex);
  const std::vector<Origin> *b = shared_->bound(principal_, name);
  if (!b || b->empty())
    return fail(error, "credential '" + name + "' is bound to no origin");
  // The file's token for it stays on file: this one is ahead of it, for this run.
  std::erase_if(shared_->entries, [&](const Shared::Entry &e) {
    return e.principal == principal_ && e.name == name && !e.kept;
  });
  shared_->entries.push_back({principal_, name, b->front(), token, false});
  return true;
}

std::optional<std::string> CredentialStore::token_for(const std::string &name,
                                                      const Origin &to) const {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  const auto &entries = shared_->entries;
  for (auto it = entries.rbegin(); it != entries.rend(); ++it)
    if (it->principal == principal_ && it->name == name && shared_->sends_to(*it, to))
      return it->token;
  return std::nullopt;
}

bool CredentialStore::has(const std::string &name) const {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  return std::any_of(shared_->entries.begin(), shared_->entries.end(), [&](const Shared::Entry &e) {
    return e.principal == principal_ && e.name == name;
  });
}

std::vector<std::string> CredentialStore::names() const {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  std::vector<std::string> out;
  for (const Shared::Entry &e : shared_->entries)
    if (e.principal == principal_ && std::find(out.begin(), out.end(), e.name) == out.end())
      out.push_back(e.name);
  std::sort(out.begin(), out.end());
  return out;
}

bool CredentialStore::forget(const std::string &name, std::string *error) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  std::erase_if(shared_->own[principal_].renewers, [&](const auto &r) { return r.first == name; });
  const std::size_t dropped = std::erase_if(shared_->entries, [&](const Shared::Entry &e) {
    return e.principal == principal_ && e.name == name;
  });
  if (dropped > 0)
    shared_->save_locked(error);
  return dropped > 0;
}

void CredentialStore::clear(std::string *error) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  std::erase_if(shared_->entries,
                [&](const Shared::Entry &e) { return e.principal == principal_; });
  shared_->own[principal_].renewers.clear();
  shared_->save_locked(error);
}

void CredentialStore::set_renewer(const std::string &name,
                                  std::shared_ptr<CredentialRenewer> renewer) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  auto &renewers = shared_->own[principal_].renewers;
  std::erase_if(renewers, [&](const auto &r) { return r.first == name; });
  if (renewer)
    renewers.emplace_back(name, std::move(renewer));
}

std::shared_ptr<CredentialRenewer> CredentialStore::renewer(const std::string &name) const {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  const auto o = shared_->own.find(principal_);
  if (o == shared_->own.end())
    return nullptr;
  for (const auto &r : o->second.renewers)
    if (r.first == name)
      return r.second;
  return nullptr;
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

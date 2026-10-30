// Ariadne — a document set fetched from a URL. See inc/cvc/ariadne/doc_fetch.h.

#include <algorithm>
#include <cctype>
#include <cvc/ariadne/doc_fetch.h>
#include <deque>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#ifdef CVC_ARIADNE_HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace cvc {
namespace ariadne {

namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// "/a/./b/../c" -> "/a/c": RFC 3986 dot-segment removal, ".." stopping at the root, and empty
// segments dropped as the file system does ("a//b" is "a/b").
std::string remove_dot_segments(const std::string &path) {
  std::vector<std::string> segs;
  bool trailing = false;
  std::size_t i = 0;
  while (i <= path.size()) {
    const std::size_t slash = path.find('/', i);
    const std::size_t end = slash == std::string::npos ? path.size() : slash;
    const std::string seg = path.substr(i, end - i);
    const bool last = slash == std::string::npos;
    if (seg == "..") {
      if (!segs.empty())
        segs.pop_back();
      trailing = last;
    } else if (seg == "." || seg.empty()) {
      trailing = last && !segs.empty();
    } else {
      segs.push_back(seg);
      trailing = false;
    }
    if (last)
      break;
    i = slash + 1;
  }
  std::string out;
  for (const std::string &s : segs)
    out += "/" + s;
  if (out.empty() || trailing)
    out += "/";
  return out;
}

// A unit's argument token ({name}) in a reference: what it names is known only once an include:
// fills it in, so it cannot be fetched ahead.
bool templated(const std::string &ref) {
  const std::size_t open = ref.find('{');
  return open != std::string::npos && ref.find('}', open) != std::string::npos;
}

} // namespace

std::string DocUrl::origin() const { return scheme + "://" + authority; }

std::string DocUrl::str() const { return origin() + path + (query.empty() ? "" : "?" + query); }

bool parse_doc_url(const std::string &text, DocUrl &out, std::string *error) {
  const auto fail = [&](const std::string &why) {
    if (error)
      *error = why;
    return false;
  };
  const std::size_t sep = text.find("://");
  if (sep == std::string::npos)
    return fail("'" + text + "' is not an absolute URL");
  DocUrl u;
  u.scheme = lower(text.substr(0, sep));
  if (u.scheme != "http" && u.scheme != "https")
    return fail("'" + text + "' is not an http:// or https:// URL");
  const std::size_t start = sep + 3;
  const std::size_t auth_end = text.find_first_of("/?#", start);
  std::string authority =
      text.substr(start, (auth_end == std::string::npos ? text.size() : auth_end) - start);
  if (authority.empty())
    return fail("'" + text + "' has no host");
  if (authority.find('@') != std::string::npos)
    return fail("'" + text + "' has user info; a document URL may not carry credentials");
  for (const unsigned char c : authority)
    if (c <= 0x20 || c >= 0x7f)
      return fail("'" + text + "' has a malformed host");
  authority = lower(authority);
  // A default port is the same origin as none.
  const std::string dflt = u.scheme == "https" ? ":443" : ":80";
  if (authority.size() > dflt.size() &&
      authority.compare(authority.size() - dflt.size(), dflt.size(), dflt) == 0)
    authority.erase(authority.size() - dflt.size());
  u.authority = authority;
  if (auth_end != std::string::npos) {
    std::string rest = text.substr(auth_end);
    if (const std::size_t hash = rest.find('#'); hash != std::string::npos)
      rest.erase(hash);
    const std::size_t q = rest.find('?');
    u.path = rest.substr(0, q);
    if (q != std::string::npos)
      u.query = rest.substr(q + 1);
  }
  if (u.path.empty())
    u.path = "/";
  out = u;
  return true;
}

std::string doc_url_for(const DocUrl &origin, const std::string &path) {
  static const char *const hex = "0123456789ABCDEF";
  std::string out = origin.origin();
  for (const unsigned char c : path) {
    if (c <= 0x20 || c >= 0x7f || c == '"' || c == '<' || c == '>' || c == '\\' || c == '^' ||
        c == '`' || c == '{' || c == '|' || c == '}' || c == '#') {
      out += '%';
      out += hex[c >> 4];
      out += hex[c & 15];
    } else {
      out += static_cast<char>(c);
    }
  }
  return out;
}

bool resolve_doc_ref(const std::string &base_path, const std::string &ref, std::string &out) {
  // The loader's parse_uri: a "<scheme>://" anywhere makes it a URI, not a file path.
  if (ref.empty() || ref.find("://") != std::string::npos || ref.rfind("//", 0) == 0)
    return false;
  if (ref[0] == '/') {
    out = remove_dot_segments(ref);
    return true;
  }
  const std::size_t slash = base_path.find_last_of('/');
  const std::string dir = slash == std::string::npos ? "/" : base_path.substr(0, slash + 1);
  out = remove_dot_segments((dir.empty() || dir[0] != '/' ? "/" + dir : dir) + ref);
  return true;
}

#ifdef CVC_ARIADNE_HAVE_YAML
namespace {
void collect_refs(const YAML::Node &n, std::vector<DocReference> &out, std::set<std::string> &seen,
                  bool top) {
  const auto add = [&](const YAML::Node &v, bool document) {
    if (v && v.IsScalar() && !v.Scalar().empty() && seen.insert(v.Scalar()).second)
      out.push_back({v.Scalar(), document});
  };
  if (n.IsMap()) {
    for (const auto &kv : n) {
      const std::string key = kv.first.IsScalar() ? kv.first.Scalar() : std::string();
      const YAML::Node &v = kv.second;
      if (top && key == "import") {
        if (v.IsSequence())
          for (const auto &e : v)
            add(e, true);
        else
          add(v, true);
      } else if (key == "load") {
        add(v, true);
      } else if (key == "vinay") {
        add(v, false);
      }
      if (v.IsMap() || v.IsSequence())
        collect_refs(v, out, seen, false);
    }
  } else if (n.IsSequence()) {
    for (const auto &e : n)
      collect_refs(e, out, seen, false);
  }
}
} // namespace

std::vector<DocReference> document_references(const std::string &yaml, std::string *error) {
  std::vector<DocReference> out;
  try {
    std::set<std::string> seen;
    collect_refs(YAML::Load(yaml), out, seen, true);
  } catch (const std::exception &e) {
    if (error)
      *error = e.what();
    out.clear();
  }
  return out;
}
#else
std::vector<DocReference> document_references(const std::string &, std::string *error) {
  if (error)
    *error = "libcvc was built without yaml-cpp";
  return {};
}
#endif

bool doc_origin_allowed(const DocUrl &src, const std::string &page_origin,
                        const std::vector<std::string> &allow_origins, std::string *error) {
  if (page_origin.empty())
    return true;
  const auto origin_of = [](const std::string &s) {
    DocUrl u;
    return parse_doc_url(s, u) ? u.origin() : s;
  };
  const std::string page = origin_of(page_origin);
  if (src.origin() == page)
    return true;
  for (const std::string &a : allow_origins)
    if (a == "*" || origin_of(a) == src.origin())
      return true;
  if (error)
    *error = "the document is on " + src.origin() + ", not this page's origin " + page +
             " (a page loads documents only from its own origin unless its config lists "
             "another in src_origins)";
  return false;
}

DocSet fetch_document_set(const std::string &src_url, const DocFetcher &fetch,
                          const DocSetOptions &options) {
  DocSet set;
  std::string why;
  if (!parse_doc_url(src_url, set.src, &why)) {
    set.error = "src: " + why;
    return set;
  }
  if (!doc_origin_allowed(set.src, options.page_origin, options.allow_origins, &why)) {
    set.error = "src " + set.src.str() + ": " + why;
    return set;
  }
  resolve_doc_ref("/", set.src.path, set.doc_path);
  if (set.doc_path.back() == '/') {
    set.error = "src " + set.src.str() + " names a directory, not a document";
    return set;
  }
  if (!fetch) {
    set.error = "no way to fetch " + set.src.str();
    return set;
  }

  struct Pending {
    std::string path, url;
    bool document;
    std::string from, ref; // who named it, and how (empty for the document itself)
  };
  std::deque<Pending> queue;
  std::set<std::string> queued{set.doc_path};
  queue.push_back({set.doc_path, set.src.str(), true, std::string(), std::string()});
  const auto enqueue = [&](const std::string &from, const std::string &ref, bool document) {
    if (templated(ref)) {
      set.warnings.push_back(from + ": '" + ref +
                             "' depends on a unit's arguments, so it was not fetched ahead (list "
                             "the files it can name in the page's preload)");
      return;
    }
    std::string path;
    if (!resolve_doc_ref(from, ref, path) || !queued.insert(path).second)
      return;
    queue.push_back({path, doc_url_for(set.src, path), document, from, ref});
  };
  std::size_t bytes = 0;
  bool first = true;
  while (!queue.empty()) {
    const Pending p = queue.front();
    queue.pop_front();
    if (set.files.size() >= options.max_files) {
      set.warnings.push_back("the document names more than " + std::to_string(options.max_files) +
                             " files; the rest were not fetched");
      break;
    }
    const DocFetchResponse r = fetch(p.url);
    std::string failed;
    if (!r.ok)
      failed = r.error.empty() ? "no reply" : r.error;
    else if (r.status < 200 || r.status > 299)
      failed = "HTTP " + std::to_string(r.status);
    else if (bytes + r.body.size() > options.max_bytes)
      failed = "the document set is over " + std::to_string(options.max_bytes) + " bytes";
    if (!failed.empty()) {
      if (first) {
        set.error = "could not fetch " + p.url + ": " + failed;
        return set;
      }
      set.warnings.push_back(p.from + ": '" + p.ref + "' (" + p.url + "): " + failed);
      continue;
    }
    bytes += r.body.size();
    set.files.push_back({p.path, p.url, r.body});
    if (p.document) {
      // A file that does not parse names nothing; the loader reports the parse error itself.
      for (const DocReference &ref : document_references(r.body))
        enqueue(p.path, ref.uri, ref.document);
    }
    if (first)
      for (const std::string &f : options.preload)
        enqueue(p.path, f, false);
    first = false;
  }
  set.ok = true;
  return set;
}

bool write_document_set(const DocSet &set, const std::string &root, std::string *error) {
  namespace fs = std::filesystem;
  std::string base = root;
  while (!base.empty() && base.back() == '/')
    base.pop_back();
  for (const DocSetFile &f : set.files) {
    const fs::path p(base + f.path);
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    std::ofstream o(p, std::ios::binary | std::ios::trunc);
    o.write(f.content.data(), static_cast<std::streamsize>(f.content.size()));
    o.close();
    if (!o) {
      if (error)
        *error = "cannot write " + p.string() + (ec ? " (" + ec.message() + ")" : "");
      return false;
    }
  }
  return true;
}

} // namespace ariadne
} // namespace cvc

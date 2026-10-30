// Ariadne — a document set fetched from a URL, for a host that has no file system of its own to
// run a document from: arigl's browser shell (arigl-web) opened on `?src=<url>`.
//
// The host fetches the document and every file it names relative to itself (`import:`, `load:`,
// a transfer function's `vinay:`), and the files those name, and writes each one at its URL path
// in a local file system (the browser's MEMFS). The loader then runs the local copy of the main
// document with its own file scheme: a relative reference resolves against the importing file's
// directory there exactly as it did against its URL, so nothing in the loader knows about URLs.
//
//   https://tx.wtf/apps/nicotina/main.ari      ->  /apps/nicotina/main.ari
//     import: [lib/feed.ari, ../shared/a.ari]  ->  /apps/nicotina/lib/feed.ari, /apps/shared/a.ari
//
// A reference with a scheme (cvc://, state://, https://) is not fetched here: cvc:// is the
// component library the host ships, and the rest go through the loader's URI handlers, under
// whatever network policy the host sets. Only the document's own origin is fetched, and the
// document itself must be on the page's origin unless the host allows another.

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// An absolute http:// or https:// URL, split. The path is kept as spelled (percent escapes too),
// since it doubles as the local path of the fetched file.
struct DocUrl {
  std::string scheme;    // "http" or "https"
  std::string authority; // host[:port], the host lowercased, a default port dropped
  std::string path;      // begins with '/'
  std::string query;     // after '?', without it; empty for none

  std::string origin() const; // scheme://authority
  std::string str() const;    // the whole URL (no fragment)
};

// Parse an absolute http(s) URL. False, with `error` saying why, for anything else: another
// scheme, a relative reference, a URL with user info, or no host.
bool parse_doc_url(const std::string &text, DocUrl &out, std::string *error = nullptr);

// The URL that fetches the file at local `path` on `origin`'s server: bytes a URL cannot carry
// (a space, non-ASCII) percent-encoded, existing escapes and '?' kept.
std::string doc_url_for(const DocUrl &origin, const std::string &path);

// Resolve `ref`, as written in the file at absolute path `base_path`, the way the loader resolves
// a relative file reference: against the file's directory (an absolute ref against the root),
// with "." and ".." removed (".." stops at the root). False for a reference that is not a path:
// it has a scheme (cvc://, https://, file://) or is a //host reference.
bool resolve_doc_ref(const std::string &base_path, const std::string &ref, std::string &out);

// A file a document names.
struct DocReference {
  std::string uri;
  bool document = true; // an .ari the loader parses (import:, load:); false for data (vinay:)
};

// The files a document names, in document order without repeats: its top-level `import:` (one
// URI or a list), and every `load:` and `vinay:` anywhere in it. Empty, with `error` set, when the
// text is not YAML or this libcvc was built without yaml-cpp.
std::vector<DocReference> document_references(const std::string &yaml,
                                              std::string *error = nullptr);

// May a page at `page_origin` load a document from `src`? Its own origin always; another only if
// `allow_origins` lists it (an origin like "https://cdn.example", or "*" for any). An empty
// `page_origin` (a native host, no page) allows any. False, with `error`, otherwise.
bool doc_origin_allowed(const DocUrl &src, const std::string &page_origin,
                        const std::vector<std::string> &allow_origins,
                        std::string *error = nullptr);

// One GET. ok means a reply came back; `status` is its HTTP status.
struct DocFetchResponse {
  bool ok = false;
  long status = 0;
  std::string body;
  std::string error; // why there was no reply
};
using DocFetcher = std::function<DocFetchResponse(const std::string &url)>;

struct DocSetOptions {
  std::string page_origin;                // see doc_origin_allowed
  std::vector<std::string> allow_origins; // see doc_origin_allowed
  // More files to fetch with the document, relative to it: data the scan does not see (an image,
  // a mesh). A missing one is a warning.
  std::vector<std::string> preload;
  std::size_t max_files = 512;                  // the whole set, the document included
  std::size_t max_bytes = 128u * 1024u * 1024u; // the whole set
};

struct DocSetFile {
  std::string path; // the local path: the URL's path
  std::string url;  // where it came from
  std::string content;
};

struct DocSet {
  bool ok = false;
  std::string error; // why the document could not be loaded (ok is false)
  DocUrl src;
  std::string doc_path;              // the main document's local path
  std::vector<DocSetFile> files;     // the main document first
  std::vector<std::string> warnings; // a named file that could not be fetched, a skipped name
};

// Fetch the document at `src_url` and the files it names, recursively. Fails (ok false) when the
// URL is not allowed, or the document itself cannot be fetched (no reply, an HTTP error status)
// or parsed; a file it names that cannot be fetched is a warning, as the loader's own is.
DocSet fetch_document_set(const std::string &src_url, const DocFetcher &fetch,
                          const DocSetOptions &options = {});

// Write each file of `set` at `root` + its path, making the directories. False, with `error`, at
// the first file that cannot be written.
bool write_document_set(const DocSet &set, const std::string &root, std::string *error = nullptr);

} // namespace ariadne
} // namespace cvc

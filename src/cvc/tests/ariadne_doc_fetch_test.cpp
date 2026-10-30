// Tests for a document set fetched from a URL (cvc/ariadne/doc_fetch.h): the URL parsing and
// reference resolution arigl's browser shell (arigl-web) loads `?src=` documents with, its
// same-origin rule, and the fetch walk over a fake server. The walk's local copy must be one the
// loader resolves exactly as it would the originals, so the last tests load it.

#include <cvc/ariadne/doc_fetch.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/widget.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <string>
#include <vector>

using namespace cvc::ariadne;

namespace {

namespace fs = std::filesystem;

#define SKIP_WITHOUT_YAML()                                                                        \
  do {                                                                                             \
    if (!have_yaml())                                                                              \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

// A fake web server: URL -> (status, body). Records what was asked for, in order.
struct FakeServer {
  std::map<std::string, std::pair<long, std::string>> pages;
  std::vector<std::string> asked;
  DocFetcher fetcher() {
    return [this](const std::string &url) {
      asked.push_back(url);
      DocFetchResponse r;
      const auto it = pages.find(url);
      r.ok = true;
      r.status = it == pages.end() ? 404 : it->second.first;
      if (it != pages.end())
        r.body = it->second.second;
      return r;
    };
  }
  void add(const std::string &url, const std::string &body) { pages[url] = {200, body}; }
};

const DocSetFile *file_at(const DocSet &set, const std::string &path) {
  for (const DocSetFile &f : set.files)
    if (f.path == path)
      return &f;
  return nullptr;
}

bool has_text(const Widget &w, const std::string &label) {
  if (w.kind == Kind::Text && w.label == label)
    return true;
  for (const Widget &c : w.children)
    if (has_text(c, label))
      return true;
  return false;
}

std::string joined(const std::vector<std::string> &v) {
  std::string s;
  for (const std::string &x : v)
    s += x + "\n";
  return s;
}

} // namespace

TEST(AriadneDocFetch, ParsesDocumentUrls) {
  DocUrl u;
  ASSERT_TRUE(parse_doc_url("https://TX.wtf:443/apps/nicotina/main.ari?v=3#top", u));
  EXPECT_EQ(u.scheme, "https");
  EXPECT_EQ(u.authority, "tx.wtf"); // lowercased, the default port dropped
  EXPECT_EQ(u.path, "/apps/nicotina/main.ari");
  EXPECT_EQ(u.query, "v=3"); // the fragment is not part of what is fetched
  EXPECT_EQ(u.origin(), "https://tx.wtf");
  EXPECT_EQ(u.str(), "https://tx.wtf/apps/nicotina/main.ari?v=3");

  ASSERT_TRUE(parse_doc_url("http://127.0.0.1:8000", u));
  EXPECT_EQ(u.authority, "127.0.0.1:8000");
  EXPECT_EQ(u.path, "/");

  std::string why;
  EXPECT_FALSE(parse_doc_url("main.ari", u, &why)); // relative: the page resolves it first
  EXPECT_NE(why.find("not an absolute URL"), std::string::npos);
  EXPECT_FALSE(parse_doc_url("file:///etc/passwd", u, &why));
  EXPECT_NE(why.find("http"), std::string::npos);
  EXPECT_FALSE(parse_doc_url("https://user:secret@tx.wtf/main.ari", u, &why));
  EXPECT_NE(why.find("user info"), std::string::npos);
  EXPECT_FALSE(parse_doc_url("https:///main.ari", u, &why));
}

TEST(AriadneDocFetch, ResolvesReferencesLikeTheLoader) {
  const std::string base = "/apps/nicotina/main.ari";
  std::string out;
  ASSERT_TRUE(resolve_doc_ref(base, "lib/feed.ari", out));
  EXPECT_EQ(out, "/apps/nicotina/lib/feed.ari");
  ASSERT_TRUE(resolve_doc_ref(base, "./profile.ari", out));
  EXPECT_EQ(out, "/apps/nicotina/profile.ari");
  ASSERT_TRUE(resolve_doc_ref(base, "../shared/theme.ari", out));
  EXPECT_EQ(out, "/apps/shared/theme.ari");
  ASSERT_TRUE(resolve_doc_ref(base, "../../../../../top.ari", out)); // ".." stops at the root
  EXPECT_EQ(out, "/top.ari");
  ASSERT_TRUE(resolve_doc_ref(base, "/abs/lib.ari", out)); // against the origin's root
  EXPECT_EQ(out, "/abs/lib.ari");
  ASSERT_TRUE(resolve_doc_ref(base, "lib//x.ari", out)); // as the file system reads it
  EXPECT_EQ(out, "/apps/nicotina/lib/x.ari");
  ASSERT_TRUE(resolve_doc_ref("/apps/nicotina/lib/feed.ari", "../main.ari", out));
  EXPECT_EQ(out, "/apps/nicotina/main.ari");

  // Not paths: the host ships cvc://, and the loader's URI handlers take the rest.
  EXPECT_FALSE(resolve_doc_ref(base, "cvc://components/stage_lighting.ari", out));
  EXPECT_FALSE(resolve_doc_ref(base, "https://cdn.example/lib.ari", out));
  EXPECT_FALSE(resolve_doc_ref(base, "state://ui.docs.lib", out));
  EXPECT_FALSE(resolve_doc_ref(base, "//evil.example/lib.ari", out));
  EXPECT_FALSE(resolve_doc_ref(base, "", out));
}

// The point of the mirror: a relative reference the walk resolved to path P is found by the
// loader's file scheme at <root>P when the importing file is at <root><its path>.
TEST(AriadneDocFetch, ResolutionAgreesWithTheFileScheme) {
  const fs::path root = fs::temp_directory_path() / "ariadne_doc_fetch_agree";
  fs::remove_all(root);
  const std::string base = "/apps/nicotina/main.ari";
  for (const std::string ref :
       {"lib/feed.ari", "./profile.ari", "../shared/theme.ari", "lib/../lib/./again.ari",
        "with space.ari", "pct%20name.ari", "q.ari?v=2"}) {
    SCOPED_TRACE(ref);
    std::string path;
    ASSERT_TRUE(resolve_doc_ref(base, ref, path));
    const fs::path file = root.string() + path;
    fs::create_directories(file.parent_path());
    std::ofstream(file) << "units: {}\n";
    const UriResult r = resolve(ref, (root / "apps" / "nicotina").string());
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(fs::path(r.canonical), fs::weakly_canonical(file));
  }
  fs::remove_all(root);
}

TEST(AriadneDocFetch, UrlForALocalPathEscapesWhatAUrlCannotCarry) {
  DocUrl o;
  ASSERT_TRUE(parse_doc_url("https://tx.wtf/apps/nicotina/main.ari", o));
  EXPECT_EQ(doc_url_for(o, "/apps/nicotina/lib/feed.ari"),
            "https://tx.wtf/apps/nicotina/lib/feed.ari");
  EXPECT_EQ(doc_url_for(o, "/a b/\xc3\xa9.ari"), "https://tx.wtf/a%20b/%C3%A9.ari");
  EXPECT_EQ(doc_url_for(o, "/pct%20name.ari"), "https://tx.wtf/pct%20name.ari"); // kept
  EXPECT_EQ(doc_url_for(o, "/q.ari?v=2"), "https://tx.wtf/q.ari?v=2");
  EXPECT_EQ(doc_url_for(o, "/hash#name.ari"), "https://tx.wtf/hash%23name.ari");
}

TEST(AriadneDocFetch, ListsTheFilesADocumentNames) {
  SKIP_WITHOUT_YAML();
  const std::string doc = "import: [lib/a.ari, cvc://components/stage_lighting.ari]\n"
                          "units:\n"
                          "  panel:\n"
                          "    window: P\n"
                          "    children:\n"
                          "      - load: frag.ari\n"
                          "        as: f\n"
                          "      - import: not_an_import.ari\n" // import: is top-level only
                          "windows:\n"
                          "  - load: frag.ari\n" // a repeat
                          "    as: g\n"
                          "scene:\n"
                          "  nodes:\n"
                          "    - transfer_function: { vinay: tf/bone.vinay }\n";
  std::string error;
  const std::vector<DocReference> refs = document_references(doc, &error);
  EXPECT_TRUE(error.empty()) << error;
  ASSERT_EQ(refs.size(), 4u);
  EXPECT_EQ(refs[0].uri, "lib/a.ari");
  EXPECT_TRUE(refs[0].document);
  EXPECT_EQ(refs[1].uri, "cvc://components/stage_lighting.ari");
  EXPECT_EQ(refs[2].uri, "frag.ari");
  EXPECT_TRUE(refs[2].document);
  EXPECT_EQ(refs[3].uri, "tf/bone.vinay");
  EXPECT_FALSE(refs[3].document); // a colour table, not parsed for more names

  EXPECT_EQ(document_references("import: lone.ari\n").size(), 1u);
  EXPECT_TRUE(document_references("windows: [ { window: W, children: [] }\n", &error).empty());
  EXPECT_FALSE(error.empty());
}

TEST(AriadneDocFetch, APageLoadsOnlyItsOwnOriginUnlessItAllowsAnother) {
  DocUrl src;
  ASSERT_TRUE(parse_doc_url("https://tx.wtf/apps/nicotina/main.ari", src));
  std::string why;
  EXPECT_TRUE(doc_origin_allowed(src, "https://tx.wtf", {}, &why));
  EXPECT_TRUE(doc_origin_allowed(src, "https://tx.wtf:443", {}, &why)); // the same origin
  EXPECT_TRUE(doc_origin_allowed(src, "", {}, &why));                   // no page: a native host

  EXPECT_FALSE(doc_origin_allowed(src, "http://tx.wtf", {}, &why)); // another scheme
  EXPECT_FALSE(doc_origin_allowed(src, "https://evil.example", {}, &why));
  EXPECT_NE(why.find("src_origins"), std::string::npos) << why; // says how to allow it
  EXPECT_FALSE(doc_origin_allowed(src, "https://www.tx.wtf", {"https://cdn.tx.wtf"}, &why));
  EXPECT_TRUE(doc_origin_allowed(src, "https://www.tx.wtf", {"https://tx.wtf/"}, &why));
  EXPECT_TRUE(doc_origin_allowed(src, "https://www.tx.wtf", {"*"}, &why));
}

TEST(AriadneDocFetch, FetchesTheDocumentAndWhatItNamesRelativeToItsUrl) {
  SKIP_WITHOUT_YAML();
  FakeServer web;
  const std::string main_doc = "import:\n"
                               "  - lib/feed.ari\n"
                               "  - ../shared/theme.ari\n"
                               "  - cvc://components/stage_lighting.ari\n"
                               "  - https://cdn.example/remote.ari\n"
                               "  - missing.ari\n"
                               "windows:\n"
                               "  - load: '{which}.ari'\n"
                               "    as: t\n";
  web.add("https://tx.wtf/apps/nicotina/main.ari?v=3", main_doc);
  web.add("https://tx.wtf/apps/nicotina/lib/feed.ari",
          "import: ./post.ari\nunits:\n  feed: { load: ../main.ari, as: loop }\n");
  web.add("https://tx.wtf/apps/nicotina/lib/post.ari", "units: {}\n");
  web.add("https://tx.wtf/apps/shared/theme.ari",
          "scene:\n  nodes: [ { transfer_function: { vinay: colors.vinay } } ]\n");
  web.add("https://tx.wtf/apps/shared/colors.vinay", "not yaml: [");
  web.add("https://tx.wtf/apps/nicotina/img/logo.png", std::string("\x89PNG\0\1", 6));

  DocSetOptions opts;
  opts.page_origin = "https://tx.wtf";
  opts.preload = {"img/logo.png"};
  const DocSet set =
      fetch_document_set("https://tx.wtf/apps/nicotina/main.ari?v=3", web.fetcher(), opts);
  ASSERT_TRUE(set.ok) << set.error;
  EXPECT_EQ(set.doc_path, "/apps/nicotina/main.ari");
  ASSERT_FALSE(set.files.empty());
  EXPECT_EQ(set.files[0].path, "/apps/nicotina/main.ari");
  EXPECT_NE(file_at(set, "/apps/nicotina/lib/feed.ari"), nullptr);
  EXPECT_NE(file_at(set, "/apps/nicotina/lib/post.ari"), nullptr); // feed.ari's own import
  EXPECT_NE(file_at(set, "/apps/shared/theme.ari"), nullptr);
  EXPECT_NE(file_at(set, "/apps/shared/colors.vinay"), nullptr); // relative to theme.ari
  ASSERT_NE(file_at(set, "/apps/nicotina/img/logo.png"), nullptr);
  EXPECT_EQ(file_at(set, "/apps/nicotina/img/logo.png")->content.size(), 6u); // bytes, not text
  EXPECT_EQ(set.files.size(), 6u);

  // Each file once (feed.ari's load: of ../main.ari is the document itself), and nothing off the
  // page's origin or from the component library.
  for (const std::string &u : web.asked) {
    EXPECT_EQ(u.rfind("https://tx.wtf/", 0), 0u) << u;
    EXPECT_EQ(u.find("stage_lighting"), std::string::npos) << u;
  }
  EXPECT_EQ(web.asked.size(), 7u) << joined(web.asked); // the six, and missing.ari

  // A missing import and an argument-dependent name are warnings, as the loader's are.
  const std::string w = joined(set.warnings);
  EXPECT_NE(w.find("missing.ari"), std::string::npos) << w;
  EXPECT_NE(w.find("HTTP 404"), std::string::npos) << w;
  EXPECT_NE(w.find("{which}.ari"), std::string::npos) << w;
}

TEST(AriadneDocFetch, FailsWhenTheDocumentItselfCannotBeLoaded) {
  FakeServer web;
  DocSetOptions opts;
  opts.page_origin = "https://tx.wtf";

  DocSet set = fetch_document_set("https://tx.wtf/apps/nicotina/main.ari", web.fetcher(), opts);
  EXPECT_FALSE(set.ok);
  EXPECT_NE(set.error.find("https://tx.wtf/apps/nicotina/main.ari"), std::string::npos);
  EXPECT_NE(set.error.find("HTTP 404"), std::string::npos) << set.error;

  // Cross-origin: refused before anything is fetched.
  web.asked.clear();
  set = fetch_document_set("https://evil.example/main.ari", web.fetcher(), opts);
  EXPECT_FALSE(set.ok);
  EXPECT_NE(set.error.find("not this page's origin"), std::string::npos) << set.error;
  EXPECT_TRUE(web.asked.empty());

  set = fetch_document_set("https://tx.wtf/apps/nicotina/", web.fetcher(), opts);
  EXPECT_FALSE(set.ok);
  EXPECT_NE(set.error.find("directory"), std::string::npos) << set.error;

  // No reply at all (the network is down, CORS refused it).
  set = fetch_document_set(
      "https://tx.wtf/main.ari",
      [](const std::string &) {
        DocFetchResponse r;
        r.error = "TypeError: Failed to fetch";
        return r;
      },
      opts);
  EXPECT_FALSE(set.ok);
  EXPECT_NE(set.error.find("Failed to fetch"), std::string::npos) << set.error;

  // A set over its byte budget.
  web.add("https://tx.wtf/big.ari", std::string(2048, '#'));
  opts.max_bytes = 1024;
  set = fetch_document_set("https://tx.wtf/big.ari", web.fetcher(), opts);
  EXPECT_FALSE(set.ok);
  EXPECT_NE(set.error.find("bytes"), std::string::npos) << set.error;
}

// End to end: the fetched set, written under a root, loads with its imports resolved.
TEST(AriadneDocFetch, TheWrittenSetLoadsWithItsImports) {
  SKIP_WITHOUT_YAML();
  FakeServer web;
  web.add("http://127.0.0.1:8000/apps/demo/main.ari",
          "import: lib/panel.ari\nwindows:\n  - include: panel\n");
  web.add("http://127.0.0.1:8000/apps/demo/lib/panel.ari",
          "import: ../../shared/text.ari\n"
          "units:\n  panel:\n    window: P\n    children:\n      - include: hello\n");
  web.add("http://127.0.0.1:8000/apps/shared/text.ari", "units:\n  hello: { text: FromShared }\n");
  DocSetOptions opts;
  opts.page_origin = "http://127.0.0.1:8000";
  const DocSet set =
      fetch_document_set("http://127.0.0.1:8000/apps/demo/main.ari", web.fetcher(), opts);
  ASSERT_TRUE(set.ok) << set.error;
  ASSERT_EQ(set.files.size(), 3u);

  const fs::path root = fs::temp_directory_path() / "ariadne_doc_fetch_load";
  fs::remove_all(root);
  std::string error;
  ASSERT_TRUE(write_document_set(set, root.string() + "/", &error)) << error;
  const LoadResult r = load_file(root.string() + set.doc_path);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(joined(r.warnings).find("could not be resolved"), std::string::npos)
      << joined(r.warnings);
  EXPECT_TRUE(has_text(r.root, "FromShared"));
  fs::remove_all(root);
}

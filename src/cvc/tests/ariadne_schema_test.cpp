// The .ari schema (docs/ARIADNE.md §14): the file the library carries, its shape (every built-in
// widget type and top-level block in it, in the validator's subset), what validate_* and a load
// find (with where), what they accept, and the shipped documents, which must all be valid.

#include <algorithm>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef CVC_SCHEMA_TEST_HAVE_JSON
#include <nlohmann/json.hpp>
#endif

using namespace cvc::ariadne;
namespace fs = std::filesystem;

namespace {

const fs::path kSource = CVC_SCHEMA_TEST_SOURCE_DIR;

std::string read_file(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
  return s;
}

#define SKIP_WITHOUT_VALIDATOR()                                                                   \
  if (!have_yaml() || !have_jsonschema())                                                          \
  GTEST_SKIP() << "libcvc built without yaml-cpp or json-schema-validator"

std::string all_issues(const std::vector<SchemaIssue> &issues) {
  std::string s;
  for (const SchemaIssue &i : issues)
    s += "\n  " + i.text();
  return s;
}

} // namespace

// ---- the schema text --------------------------------------------------------------------------

TEST(AriSchema, TheLibraryCarriesTheSourceFile) {
  const fs::path file = kSource / "src/cvc/ariadne/schema/ari.v1.schema.json";
  ASSERT_TRUE(fs::exists(file)) << file;
  std::string embedded = ari_schema_json();
  embedded.erase(std::remove(embedded.begin(), embedded.end(), '\r'), embedded.end());
  EXPECT_EQ(embedded, read_file(file)) << "rebuild: the embedded schema is stale";
}

#ifdef CVC_SCHEMA_TEST_HAVE_JSON
namespace {
using njson = nlohmann::json;

njson schema() { return njson::parse(ari_schema_json()); }

// Every keyword the validator (json-schema-validator, draft-7 core) does not implement, and the
// keys a $ref may stand beside (draft 7 ignores any other sibling of a $ref).
void check_subset(const njson &s, const std::string &where, std::vector<std::string> &problems) {
  if (s.is_object()) {
    for (const char *k : {"unevaluatedProperties", "unevaluatedItems", "prefixItems",
                          "dependentRequired", "dependentSchemas", "$dynamicRef", "$anchor"})
      if (s.contains(k))
        problems.push_back(where + ": " + k);
    if (s.contains("$ref"))
      for (const auto &kv : s.items())
        if (kv.key() != "$ref" && kv.key() != "description" && kv.key() != "$comment")
          problems.push_back(where + ": '" + kv.key() + "' beside a $ref");
    // The validator reports a value's error under a schema-valued additionalProperties as one
    // message at the map, with no pointer into the value: such a map uses patternProperties.
    if (s.contains("additionalProperties") && !s["additionalProperties"].is_boolean())
      problems.push_back(where + ": additionalProperties with a schema (use patternProperties)");
    for (const auto &kv : s.items())
      check_subset(kv.value(), where + "/" + kv.key(), problems);
  } else if (s.is_array()) {
    for (std::size_t i = 0; i < s.size(); ++i)
      check_subset(s[i], where + "/" + std::to_string(i), problems);
  }
}

// The names a "^(a|b|c)$" pattern alternates.
std::set<std::string> alternatives(const std::string &re) {
  std::set<std::string> out;
  if (re.size() < 4 || re.compare(0, 2, "^(") != 0 || re.compare(re.size() - 2, 2, ")$") != 0)
    return out;
  std::stringstream in(re.substr(2, re.size() - 4));
  for (std::string k; std::getline(in, k, '|');)
    out.insert(k);
  return out;
}
} // namespace

TEST(AriSchema, KeepsToWhatTheValidatorImplements) {
  std::vector<std::string> problems;
  check_subset(schema(), "", problems);
  EXPECT_TRUE(problems.empty()) << ::testing::PrintToString(problems);
}

// Every built-in widget type has its schema, in typed_widget, and in short_widget's chain in the
// loader's order (the first type key a widget has is its type); no schema names another type.
TEST(AriSchema, EveryBuiltinWidgetTypeHasItsSchema) {
  const njson s = schema();
  const njson &defs = s["$defs"];
  const std::vector<std::string> types = builtin_widget_types();
  ASSERT_FALSE(types.empty());
  std::set<std::string> typed;
  for (const njson &e : defs["typed_widget"]["allOf"])
    typed.insert(e["if"]["properties"]["type"]["const"].get<std::string>());
  for (const std::string &t : types) {
    EXPECT_TRUE(defs.contains("w_" + t)) << t;
    EXPECT_TRUE(typed.count(t)) << t << " is not in typed_widget";
  }
  std::vector<std::string> chain;
  for (const njson *n = &defs["short_widget"]; n->contains("if") && (*n)["if"].contains("required");
       n = &(*n)["else"]) {
    const std::string key = (*n)["if"]["required"][0].get<std::string>();
    if (key == "type")
      break;
    chain.push_back(key);
  }
  EXPECT_EQ(chain, types)
      << "short_widget's chain is not the loader's order (builtin_widget_types)";
  for (const auto &kv : defs.items())
    if (kv.key().rfind("w_", 0) == 0 && kv.key() != "w_custom")
      EXPECT_TRUE(is_builtin_widget_type(kv.key().substr(2))) << kv.key();
}

// Each widget type's schema is closed over its own keys, the shared ones (widget_common) and x-.
TEST(AriSchema, EveryWidgetTakesTheSharedKeys) {
  const njson s = schema();
  const njson &defs = s["$defs"];
  std::set<std::string> common;
  for (const auto &kv : defs["widget_common"]["properties"].items())
    common.insert(kv.key());
  for (const auto &kv : defs.items()) {
    const bool widget = kv.key().rfind("w_", 0) == 0 && kv.key() != "w_custom";
    if (!widget && kv.key() != "include_site" && kv.key() != "load_site")
      continue;
    const njson &w = kv.value();
    EXPECT_EQ(w.value("additionalProperties", true), false) << kv.key() << " is not closed";
    std::set<std::string> shared;
    bool ext = false;
    for (const auto &p : w["patternProperties"].items()) {
      if (p.key() == "^x-")
        ext = true;
      else
        shared = alternatives(p.key());
    }
    EXPECT_TRUE(ext) << kv.key() << " takes no x- keys";
    EXPECT_EQ(shared, common) << kv.key() << "'s shared keys are not widget_common's";
  }
}

TEST(AriSchema, EveryBuiltinBlockIsInTheSchema) {
  const njson s = schema();
  const njson &doc = s["$defs"]["document"]["properties"];
  for (const std::string &b : builtin_ari_blocks())
    EXPECT_TRUE(doc.contains(b)) << b;
}
#endif // CVC_SCHEMA_TEST_HAVE_JSON

// ---- what a check finds -----------------------------------------------------------------------

namespace {
struct Mistake {
  const char *what, *doc, *pointer, *message;
  int line, column; // 0: not checked
};

const Mistake kMistakes[] = {
    {"a misspelt key",
     "windows:\n  - window: W\n    children:\n      - slider_int: N\n        bnd: n\n",
     "/windows/0/children/0", "unknown key 'bnd'", 5, 9},
    {"a misspelt type", "windows:\n  - window: W\n    children:\n      - slider_itn: N\n",
     "/windows/0/children/0", "unknown key 'slider_itn'", 4, 9},
    {"a misspelt block", "widnows:\n  - window: W\n", "", "unknown key 'widnows'", 1, 1},
    {"a misspelt key in a map's entry", "keys:\n  next:\n    lable: Next\n    key: N\n",
     "/keys/next", "unknown key 'lable'", 3, 5},
    {"a dataset with neither a uri nor a generator", "data:\n  neither: {as: bytes}\n",
     "/data/neither", "is not any of the forms this key takes", 2, 12},
    {"two types in one widget",
     "windows:\n  - window: W\n    children:\n      - button: Go\n        text: x\n",
     "/windows/0/children/0", "unknown key 'button'", 4, 9},
    {"text for a number",
     "windows:\n  - window: W\n    children:\n      - slider_int: N\n        lo: abc\n",
     "/windows/0/children/0/lo", "'abc' is the wrong type here", 5, 13},
    {"a value not in the list",
     "windows:\n  - window: W\n    children:\n      - slider_int: N\n        commit: relase\n",
     "/windows/0/children/0/commit", "'relase' is not one of the values this key takes", 5, 17},
    {"a missing min_libcvc", "meta: {name: x}\n", "/meta", "missing key 'min_libcvc'", 1, 7},
    {"a list for a map", "windows: {a: 1}\n", "/windows", "a map is the wrong type here", 1, 10},
    {"a map for a script", "init: {a: 1}\n", "/init", "a map is the wrong type here", 1, 7},
    {"a misspelt node key",
     "scene:\n  nodes:\n    - node: a\n      transform: {positon: [1, 2, 3]}\n",
     "/scene/nodes/0/transform", "unknown key 'positon'", 4, 19},
    {"a short vector", "scene:\n  nodes:\n    - node: a\n      transform: {position: [1, 2]}\n",
     "/scene/nodes/0/transform/position", "a list ", 0, 0},
    {"one bad field of a map form",
     "scene:\n  nodes:\n    - node: a\n      body: {motion: dynamik}\n",
     "/scene/nodes/0/body/motion", "'dynamik' is not one of the values", 4, 22},
    {"not a colour",
     "windows:\n  - window: W\n    children:\n      - text: T\n        color: reddish\n",
     "/windows/0/children/0/color", "'reddish' is not any of the forms this key takes", 5, 16},
    {"an unknown transfer function key",
     "scene:\n  nodes:\n    - node: v\n      type: volren\n      volren:\n        "
     "transfer_function: {colour: []}\n",
     "/scene/nodes/0/volren/transfer_function", "unknown key 'colour'", 6, 29},
    {"a colour picker's fmt not in the list",
     "windows:\n  - window: W\n    children:\n      - color: C\n        bind: c\n        "
     "fmt: cmyk\n",
     "/windows/0/children/0/fmt", "'cmyk' is not one of the values this key takes", 6, 14},
    {"a bell that is neither a flag nor always", "notifications: sometimes\n", "/notifications",
     "'sometimes' is not any of the forms this key takes", 1, 16},
    {"a list for the state: map", "state: [a, b]\n", "/state", "a list is the wrong type here", 1,
     8},
    {"a map in a state: list", "state:\n  a: [1, {b: 2}]\n", "/state/a/1",
     "a map is the wrong type here", 2, 10},
};

const char *const kValid[] = {
    // x- keys are anyone's, anywhere
    "windows:\n  - window: W\n    x-layout-note: hi\n    children:\n      - slider_int: N\n        "
    "bind: n\n        x-note: hi\nx-tooling: {a: 1}\n",
    // a custom widget's and a custom node's own keys are theirs
    "windows:\n  - window: W\n    children:\n      - type: gauge\n        gauge: 0.8\n        "
    "needle: red\n",
    "scene:\n  nodes:\n    - node: f\n      type: forest_trees\n      count: 30\n",
    // a type key that is another type's field (a progress's caption), or no label at all
    "windows:\n  - window: W\n    children:\n      - progress: P\n        bind: p\n        "
    "overlay: '50%'\n",
    "root:\n  - menubar:\n    collapse: never\n    items:\n      - menu: File\n",
    // a tf_editor's colour nodes, a radio's colour list, flags in every spelling
    "windows:\n  - window: W\n    children:\n      - tf_editor: TF\n        bind: tf\n        "
    "color: [[0, 0, 0, 0]]\n",
    "windows:\n  - window: W\n    children:\n      - radio: R\n        bind: r\n        options: "
    "[a, b]\n        color: 'c4;c5'\n",
    "windows:\n  - window: W\n    children:\n      - checkbox: C\n        bind: c\n        def: "
    "Yes\n        autofocus: off\n",
    // a document that is a list of widgets, and an anonymous group
    "- text: hi\n- separator\n",
    "windows:\n  - window: W\n    children:\n      - children:\n          - text: a\n",
    // the hosts' blocks in their short forms: a guest's path, a relay's URL, a bare server: and
    // world:
    "requires: [wasm]\nwasm: guest.wat\nroot:\n  - text: hi\n",
    "relay: grpc://localhost:50051\nroot:\n  - text: hi\n",
    "server:\nworld:\nroot:\n  - text: hi\n",
    "wasm: {module: guest.wat, export: [echo]}\nrelay: {url: grpc://localhost:50051}\n"
    "server: {}\nworld: {}\n",
    // keys the loader took after the schema was written: a bell shown always, a tray icon's
    // picture, a colour picker's hex field, a loaded application's name, the world's lag
    "notifications: always\nmenubar:\n  - tray_icon: T\n    icon: https://example.com/t.png\n"
    "    auth: secret\n",
    "windows:\n  - window: W\n    children:\n      - color: C\n        bind: c\n        fmt: "
    "hex\n",
    "root:\n  - load: cvc://components/conference.ari\n    as: site\n    name: (state-get "
    "\"site_name\")\n",
    "scene:\n  clock: {hz: 60, max_lag: 0.5}\n",
    // state: (§5.3): nested and dotted keys, an app-root one, a list, nothing, a flag
    "state:\n  run: {crop: 600, paused: 0, name: x}\n  view.camera: follow\n  /app.theme: dark\n"
    "  pos: [1, 2, 3]\n  empty:\n  ready: true\n",
};
} // namespace

TEST(AriSchema, FindsEachMistakeWhereItIs) {
  SKIP_WITHOUT_VALIDATOR();
  cvc::app app;
  for (const Mistake &m : kMistakes) {
    const ValidationResult v = validate_string(app, m.doc);
    ASSERT_TRUE(v.checked) << m.what << ": " << v.error;
    const auto hit = std::find_if(v.issues.begin(), v.issues.end(), [&](const SchemaIssue &i) {
      return i.pointer == m.pointer && i.message.find(m.message) != std::string::npos;
    });
    ASSERT_NE(hit, v.issues.end()) << m.what << ": expected " << m.pointer << ": " << m.message
                                   << ", found" << all_issues(v.issues);
    if (m.line) {
      EXPECT_EQ(hit->line, m.line) << m.what;
      EXPECT_EQ(hit->column, m.column) << m.what;
    }
    // A load says the same, as a warning.
    const LoadResult r = load_string(app, m.doc);
    EXPECT_TRUE(
        std::any_of(r.warnings.begin(), r.warnings.end(),
                    [&](const std::string &w) { return w == "ari: schema: " + hit->text(); }))
        << m.what << ": the load does not warn " << hit->text();
  }
}

TEST(AriSchema, AcceptsWhatTheLoaderTakes) {
  SKIP_WITHOUT_VALIDATOR();
  cvc::app app;
  for (const char *doc : kValid) {
    const ValidationResult v = validate_string(app, doc);
    EXPECT_TRUE(v.ok()) << doc << all_issues(v.issues);
  }
}

TEST(AriSchema, ABadYamlIsAnError) {
  cvc::app app;
  const ValidationResult v = validate_string(app, "windows: [\n");
  EXPECT_FALSE(v.ok());
  if (have_yaml()) {
    EXPECT_FALSE(v.checked);
    EXPECT_NE(v.error.find("YAML"), std::string::npos) << v.error;
  }
}

// A top-level block is the schema's, one the app registered, or one the document declares.
TEST(AriSchema, ARegisteredOrDeclaredBlockIsAllowed) {
  SKIP_WITHOUT_VALIDATOR();
  cvc::app app;
  const std::string doc = "theme: {name: midnight}\nwindows: []\n";
  EXPECT_FALSE(validate_string(app, doc).ok()) << "an unknown block";
  register_ari_block(app, "theme", [](const Value &, LoadResult &) {});
  const ValidationResult registered = validate_string(app, doc);
  EXPECT_TRUE(registered.ok()) << all_issues(registered.issues);
  cvc::app other; // registered on `app` only
  EXPECT_FALSE(validate_string(other, doc).ok());
  const ValidationResult declared =
      validate_string(other, "customs:\n  - block: theme\ntheme: {name: midnight}\n");
  EXPECT_TRUE(declared.ok()) << all_issues(declared.issues);
}

// A unit is checked with the args it declares; a token only an include site fills is not a value.
TEST(AriSchema, UnitsAreCheckedWithTheirDeclaredArgs) {
  SKIP_WITHOUT_VALIDATOR();
  cvc::app app;
  const ValidationResult v = validate_string(app, R"(
units:
  panel:
    args: {n: "5"}
    window: Panel
    children:
      - slider_int: S
        bind: s
        lo: "{n}"
        hi: "{m}"
        bnd: x
)");
  ASSERT_TRUE(v.checked) << v.error;
  ASSERT_EQ(v.issues.size(), 1u) << all_issues(v.issues);
  EXPECT_EQ(v.issues[0].context, "unit panel");
  EXPECT_EQ(v.issues[0].pointer, "/children/0");
  EXPECT_EQ(v.issues[0].message, "unknown key 'bnd'");
  EXPECT_EQ(v.issues[0].line, 11) << "the template's line";
}

// A load checks what it reaches: an include: expansion (placed in its unit), an import:ed library
// and a load: fragment (in their own files).
TEST(AriSchema, ALoadChecksExpansionsLibrariesAndFragments) {
  SKIP_WITHOUT_VALIDATOR();
  const fs::path dir = fs::temp_directory_path() / "ari_schema_test_load";
  fs::remove_all(dir);
  fs::create_directories(dir);
  {
    std::ofstream(dir / "lib.ari")
        << "units:\n  card:\n    text: Card\n    colr: red\nsurplus: 1\n";
    std::ofstream(dir / "frag.ari") << "windows:\n  - window: F\n    closeable: false\n";
    std::ofstream(dir / "main.ari") << "meta: {min_libcvc: 0.0.0}\n"
                                       "import: lib.ari\n"
                                       "windows:\n"
                                       "  - window: Main\n"
                                       "    children:\n"
                                       "      - include: card\n"
                                       "      - include: card\n"
                                       "      - load: frag.ari\n";
  }
  cvc::app app;
  const LoadResult r = load_file(app, (dir / "main.ari").string());
  ASSERT_TRUE(r.ok) << r.error;
  const auto find = [&](const std::string &file, const std::string &message) {
    return std::find_if(r.schema_issues.begin(), r.schema_issues.end(), [&](const SchemaIssue &i) {
      return fs::path(i.source).filename() == file && i.message == message;
    });
  };
  const auto lib = find("lib.ari", "unknown key 'surplus'");
  EXPECT_NE(lib, r.schema_issues.end()) << all_issues(r.schema_issues);
  const auto card = find("lib.ari", "unknown key 'colr'");
  ASSERT_NE(card, r.schema_issues.end()) << all_issues(r.schema_issues);
  EXPECT_EQ(card->context, "include card");
  EXPECT_EQ(card->line, 4) << "the unit's own line, in the library";
  EXPECT_EQ(std::count_if(r.schema_issues.begin(), r.schema_issues.end(),
                          [&](const SchemaIssue &i) { return i.message == "unknown key 'colr'"; }),
            1)
      << "included twice, its template's problem is reported once";
  const auto frag = find("frag.ari", "unknown key 'closeable'");
  ASSERT_NE(frag, r.schema_issues.end()) << all_issues(r.schema_issues);
  EXPECT_EQ(frag->pointer, "/windows/0");
  EXPECT_EQ(frag->line, 3);
  fs::remove_all(dir);
}

// Each app compiles and keeps its own validators (cvc::app slots, no process-wide state): apps
// made in either order, and checking at once on threads, find what their own documents hold.
TEST(AriSchema, SeveralAppsCheckApart) {
  SKIP_WITHOUT_VALIDATOR();
  const std::string good = "windows:\n  - window: W\n";
  const std::string bad = "windows:\n  - window: W\n    closeable: false\n";
  for (int order = 0; order < 2; ++order) {
    cvc::app first, second;
    cvc::app &a = order ? second : first, &b = order ? first : second;
    EXPECT_TRUE(validate_string(a, good).ok());
    EXPECT_FALSE(validate_string(b, bad).ok());
  }
  int failures = 0;
  std::vector<std::thread> threads;
  std::vector<int> wrong(4, 0);
  for (int t = 0; t < 4; ++t)
    threads.emplace_back([&, t] {
      cvc::app app;
      for (int i = 0; i < 10; ++i) {
        const bool want_ok = (t + i) % 2 == 0;
        if (validate_string(app, want_ok ? good : bad).ok() != want_ok)
          ++wrong[t];
      }
    });
  for (std::thread &t : threads)
    t.join();
  for (int w : wrong)
    failures += w;
  EXPECT_EQ(failures, 0);
}

// A window's groups written inline, `groups` of them around a text with an unknown key: 6 + 2 *
// groups levels of YAML (the document, windows:, the window, its children:, each group and its
// children:, then the text and its values).
std::string groups_doc(std::size_t groups) {
  std::string doc = "windows:\n  - window: W\n    children: ";
  for (std::size_t i = 0; i < groups; ++i)
    doc += "[{type: group, children: ";
  doc += "[{text: deepest, bnd: x}]";
  for (std::size_t i = 0; i < groups; ++i)
    doc += "}]";
  return doc + "\n";
}

// docs/ARIADNE.md §6.7: the check goes kMaxSchemaDepth levels of YAML deep. A document that deep is
// checked; one deeper is not, and its one issue, at its first value past the limit, says so (a
// load warns it, as any issue, and goes on).
TEST(AriSchema, TheCheckGoesAtMostkMaxSchemaDepthLevels) {
  SKIP_WITHOUT_VALIDATOR();
  ASSERT_EQ(kMaxSchemaDepth % 2, 0u);
  const std::size_t groups = (kMaxSchemaDepth - 6) / 2;
  cvc::app app;
  const ValidationResult checked = validate_string(app, groups_doc(groups));
  ASSERT_TRUE(checked.checked) << checked.error;
  ASSERT_EQ(checked.issues.size(), 1u) << all_issues(checked.issues);
  EXPECT_EQ(checked.issues[0].message, "unknown key 'bnd'");

  const std::string doc = groups_doc(groups + 1);
  const ValidationResult v = validate_string(app, doc);
  ASSERT_TRUE(v.checked) << v.error;
  EXPECT_FALSE(v.ok());
  ASSERT_EQ(v.issues.size(), 1u) << all_issues(v.issues);
  const SchemaIssue &i = v.issues[0];
  std::string pointer = "/windows/0/children/0";
  for (std::size_t g = 0; g <= groups; ++g)
    pointer += "/children/0";
  EXPECT_EQ(i.pointer, pointer) << "the text, a level past the limit";
  EXPECT_EQ(i.line, 3);
  EXPECT_EQ(i.column, 1 + static_cast<int>(doc.substr(doc.find("    children:")).find("{text:")));
  EXPECT_EQ(i.message, "nested more than " + std::to_string(kMaxSchemaDepth) +
                           " levels of YAML deep, the most the schema check goes "
                           "(docs/ARIADNE.md §6.7): this document is not checked");
  const LoadResult r = load_string(app, doc);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.schema_issues.size(), 1u) << all_issues(r.schema_issues);
  EXPECT_TRUE(std::any_of(r.warnings.begin(), r.warnings.end(),
                          [&](const std::string &w) { return w == "ari: schema: " + i.text(); }));
}

TEST(AriSchema, WithoutTheValidatorNothingIsChecked) {
  if (have_jsonschema())
    GTEST_SKIP() << "this build has the validator";
  cvc::app app;
  EXPECT_FALSE(validate_string(app, "windows: []\n").checked);
  EXPECT_TRUE(load_string(app, "widnows: []\n").schema_issues.empty());
}

// ---- the shipped documents --------------------------------------------------------------------

// Every .ari in the tree (examples, components, tests' documents) is valid, its includes, imports
// and fragments too, as a load reaches them. A document under a directory named `invalid` is a
// deliberate mistake (ari validate's own test) and is left out.
TEST(AriSchema, EveryShippedDocumentIsValid) {
  SKIP_WITHOUT_VALIDATOR();
  cvc::app app;
  register_cvc_uri_handler(app, {(kSource / "src/cvc/ariadne").string()});
  std::vector<fs::path> docs;
  for (const char *top : {"examples", "src", "bindings"})
    for (const fs::directory_entry &e : fs::recursive_directory_iterator(kSource / top)) {
      const fs::path &p = e.path();
      if (!e.is_regular_file() || p.extension() != ".ari")
        continue;
      if (std::any_of(p.begin(), p.end(), [](const fs::path &part) { return part == "invalid"; }))
        continue;
      docs.push_back(p);
    }
  ASSERT_GT(docs.size(), 100u) << "the walk found too few documents under " << kSource;
  std::string failures;
  for (const fs::path &p : docs) {
    const ValidationResult v = validate_file(app, p.string());
    if (!v.error.empty())
      failures += "\n" + v.error;
    std::vector<SchemaIssue> issues = v.issues;
    const LoadResult r = load_file(app, p.string());
    issues.insert(issues.end(), r.schema_issues.begin(), r.schema_issues.end());
    for (const SchemaIssue &i : issues)
      failures += "\n" + fs::relative(p, kSource).generic_string() + ": " + i.text();
  }
  EXPECT_TRUE(failures.empty()) << docs.size() << " documents; not valid:" << failures;
}

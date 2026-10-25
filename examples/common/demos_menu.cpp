// demos_menu — see demos_menu.h.
#include "demos_menu.h"

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>

#include <cstdlib>
#include <string>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#include <cvc/core/process.h>
#include <filesystem>
#endif

namespace cvc_examples {
namespace {

struct demo_entry {
  const char *slug, *title, *desc;
  const char *doc; // an .ari demo: arigl opened on ariadne/documents/<doc>.ari
};

// examples/web/demos.json's order, written by common/demos.cmake.
const demo_entry kDemos[] = {
#include "demos_catalog.inc"
};
constexpr std::size_t kCount = sizeof kDemos / sizeof kDemos[0];

bool g_switch_requested = false;

// How to open each demo: worked out at the first draw, then kept. Empty = not available here.
struct opener {
  bool resolved = false;
#ifdef __EMSCRIPTEN__
  std::string href; // where the page goes
#else
  std::vector<std::string> argv; // the program to start
#endif
};
opener g_open[kCount];

#ifdef __EMSCRIPTEN__

// The demo's address in the deployed gallery (gallery.js's window.CVC_GALLERY): "" when the gallery
// didn't build it. Outside a gallery page (no gallery.js) it is where the gallery would put it.
std::string href_for(const demo_entry &d) {
  char *got = static_cast<char *>(MAIN_THREAD_EM_ASM_PTR(
      {
        const g = typeof window !== 'undefined' ? window.CVC_GALLERY : undefined;
        if (!Array.isArray(g))
          return 0;
        const e = g.find(x => x.slug === UTF8ToString($0));
        return stringToNewUTF8(e ? e.href : '');
      },
      d.slug));
  if (!got)
    return d.doc[0] ? std::string("../arigl/?doc=") + d.doc : std::string("../") + d.slug + "/";
  std::string href = got;
  std::free(got);
  return href;
}

#else

namespace fs = std::filesystem;

// The program beside this one, or nothing when it isn't there.
fs::path sibling(const std::string &name) {
  static const fs::path dir = cvc::process::self_path().parent_path();
#ifdef _WIN32
  const fs::path p = dir / (name + ".exe");
#else
  const fs::path p = dir / name;
#endif
  std::error_code ec;
  return !dir.empty() && fs::exists(p, ec) ? p : fs::path();
}

// The first of `candidates` that exists.
fs::path first_existing(const std::vector<fs::path> &candidates) {
  std::error_code ec;
  for (const fs::path &p : candidates)
    if (fs::exists(p, ec))
      return p;
  return {};
}

// Natively: a C++ demo is its program beside this one; an .ari demo is arigl on its document, given
// the gallery's catalog so it has the Demos menu too. The terminal pages are browser-only.
std::vector<std::string> argv_for(const demo_entry &d) {
  if (!d.doc[0]) {
    const std::string slug = d.slug;
    if (slug == "ari" || slug == "ari_tvision")
      return {};
    const fs::path exe = sibling(slug);
    return exe.empty() ? std::vector<std::string>{} : std::vector<std::string>{exe.string()};
  }
  const fs::path arigl = sibling("arigl");
  const fs::path share = cvc::process::self_path().parent_path() / ".." / "share" / "libcvc-examples";
  const fs::path doc =
      first_existing({fs::path(CVC_EXAMPLES_DOCUMENTS) / (std::string(d.doc) + ".ari"),
                      share / "ariadne" / "documents" / (std::string(d.doc) + ".ari")});
  if (arigl.empty() || doc.empty())
    return {};
  std::vector<std::string> argv = {arigl.string(), "run"};
  std::error_code ec;
  if (fs::exists(CVC_EXAMPLES_COMPONENTS, ec)) // a build tree; installed, arigl finds its own
    argv.insert(argv.end(), {"--component-path", CVC_EXAMPLES_COMPONENTS});
  argv.push_back(doc.string());
  return argv;
}

// arigl's catalog for the program about to start: the build tree's, else the installed one.
void export_gallery() {
  const fs::path share = cvc::process::self_path().parent_path() / ".." / "share" / "libcvc-examples";
  const fs::path catalog =
      first_existing({fs::path(CVC_EXAMPLES_GALLERY), share / "gallery.json"});
  if (catalog.empty())
    return;
#ifdef _WIN32
  _putenv_s("CVC_ARIGL_GALLERY", catalog.string().c_str());
#else
  setenv("CVC_ARIGL_GALLERY", catalog.string().c_str(), 1);
#endif
}

#endif

} // namespace

void demos_menu(const char *self) {
  if (!ImGui::BeginMenu("Demos"))
    return;
  const std::string me = self ? self : "";
  for (std::size_t i = 0; i < kCount; ++i) {
    const demo_entry &d = kDemos[i];
    const bool current = me == d.slug;
    opener &o = g_open[i];
    if (!o.resolved) {
#ifdef __EMSCRIPTEN__
      o.href = href_for(d);
#else
      o.argv = argv_for(d);
#endif
      o.resolved = true;
    }
#ifdef __EMSCRIPTEN__
    if (o.href.empty() && !current)
      continue; // this gallery didn't build it
    if (ImGui::MenuItem(d.title, nullptr, current, !current))
      MAIN_THREAD_EM_ASM({ window.location.href = UTF8ToString($0); }, o.href.c_str());
#else
    if (o.argv.empty() && !current)
      continue; // not built here, or a browser-only page
    if (ImGui::MenuItem(d.title, nullptr, current, !current)) {
      export_gallery();
      if (cvc::process::spawn_detached(o.argv))
        g_switch_requested = true;
    }
#endif
    if (d.desc[0] && ImGui::IsItemHovered())
      ImGui::SetTooltip("%s", d.desc);
  }
  ImGui::EndMenu();
}

bool demo_switch_requested() { return g_switch_requested; }

} // namespace cvc_examples

#else // a cvcGL without the ImGui overlay draws no menus

namespace cvc_examples {
void demos_menu(const char *) {}
bool demo_switch_requested() { return false; }
} // namespace cvc_examples

#endif

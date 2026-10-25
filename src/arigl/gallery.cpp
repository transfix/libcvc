// arigl's gallery -- see gallery.h.
#include "gallery.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/core/process.h>
#include <exception>
#include <filesystem>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace arigl {

namespace ari = cvc::ariadne;
namespace fs = std::filesystem;
namespace pt = boost::property_tree;

gallery gallery::load(const std::string &path, std::string *error) {
  gallery g;
  if (path.empty())
    return g;
  pt::ptree root;
  try {
    pt::read_json(path, root);
  } catch (const std::exception &e) {
    if (error)
      *error = e.what();
    return g;
  }
  const fs::path base = fs::path(path).parent_path();
#ifndef __EMSCRIPTEN__
  const fs::path self_dir = cvc::process::self_path().parent_path();
#endif
  std::error_code ec;
  const auto demos = root.get_child_optional("demos");
  if (!demos)
    return g;
  for (const auto &kv : *demos) {
    const pt::ptree &d = kv.second;
    entry e;
    e.slug = d.get<std::string>("slug", "");
    e.title = d.get<std::string>("title", e.slug);
    e.desc = d.get<std::string>("desc", "");
    if (const std::string doc = d.get<std::string>("doc", ""); !doc.empty()) {
      const fs::path p = fs::path(doc).is_absolute() ? fs::path(doc) : base / doc;
      if (fs::exists(p, ec))
        e.doc = p.lexically_normal().string();
    }
#ifdef __EMSCRIPTEN__
    e.href = d.get<std::string>("href", "");
#else
    if (const std::string prog = d.get<std::string>("program", "");
        !prog.empty() && !self_dir.empty()) {
#ifdef _WIN32
      const fs::path exe = self_dir / (prog + ".exe");
#else
      const fs::path exe = self_dir / prog;
#endif
      if (fs::exists(exe, ec))
        e.program = {exe.string()};
    }
#endif
    if (!e.slug.empty() && (!e.doc.empty() || !e.href.empty() || !e.program.empty()))
      g.entries_.push_back(std::move(e));
  }
  return g;
}

ari::Widget gallery::menu(const std::string &current) const {
  std::vector<ari::Widget> items;
  const fs::path now = current.empty() ? fs::path() : fs::path(current).lexically_normal();
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const entry &e = entries_[i];
    ari::Widget item = ari::menu_action(e.title, "gallery:" + std::to_string(i));
    item.tooltip = e.desc;
    std::error_code ec;
    if (!e.doc.empty() && !now.empty() && fs::equivalent(e.doc, now, ec))
      item.disabled_when = "(= 1 1)"; // the one showing
    items.push_back(std::move(item));
  }
  return ari::menu("Demos", std::move(items));
}

void gallery::install(ari::AppRuntime &rt, std::string *pending, bool *quit) const {
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const entry e = entries_[i];
    rt.on("gallery:" + std::to_string(i), [e, pending, quit] {
      if (!e.doc.empty()) {
        *pending = e.doc; // loaded by the host between frames, never mid-walk
        return;
      }
#ifdef __EMSCRIPTEN__
      if (!e.href.empty())
        MAIN_THREAD_EM_ASM({ window.location.href = UTF8ToString($0); }, e.href.c_str());
#else
      if (!e.program.empty() && cvc::process::spawn_detached(e.program))
        *quit = true;
#endif
    });
  }
}

void gallery::show_in_url(const std::string &doc) {
#ifdef __EMSCRIPTEN__
  const std::string name = fs::path(doc).stem().string();
  MAIN_THREAD_EM_ASM(
      {
        const u = new URL(window.location.href);
        u.searchParams.set('doc', UTF8ToString($0));
        window.history.replaceState(null, '', u.toString());
      },
      name.c_str());
#else
  (void)doc;
#endif
}

} // namespace arigl

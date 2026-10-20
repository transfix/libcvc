// ImGuiBackend's §11.4 table-column edge on a bare ImGui context (no window, no GL): a resizable
// table seeds its column widths from tree.<id>.columns and reports the widths it draws back. A
// fixed column takes the persisted width; stretch columns take theirs as weights.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <sstream>
#include <string>
#include <vector>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace ari = cvc::ariadne;

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

std::vector<float> widths(const std::string &s) {
  std::vector<float> out;
  std::string t = s;
  for (char &c : t)
    if (c == ',')
      c = ' ';
  std::istringstream is(t);
  float v = 0;
  while (is >> v)
    out.push_back(v);
  return out;
}

#ifdef CVC_ENABLE_IMGUI
// One resizable table: a 100 px column and two stretch columns of equal weight.
ari::Widget table(const std::string &id) {
  ari::Widget g = ari::group({ari::text("a"), ari::text("b"), ari::text("c")});
  g.id = id;
  g.layout.kind = ari::LayoutKind::Grid;
  g.layout.resizable = true;
  g.layout.col_widths = {ari::Track{ari::Unit::Px, 100.0f}, ari::Track{ari::Unit::Percent, 50.0f},
                         ari::Track{ari::Unit::Percent, 50.0f}};
  return g;
}

// Walk `id`'s table for a few frames in a fresh context, with `persisted` in tree.<id>.columns
// ("" = none), and return what the backend persisted.
std::string run(const std::string &id, const std::string &persisted) {
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(800.0f, 600.0f);
  io.IniFilename = nullptr;
  unsigned char *px = nullptr;
  int w = 0, h = 0;
  io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
  std::string out;
  {
    cvc::app app;
    ari::Runtime rt(app, "ui");
    cvc::gl::ImGuiBackend be;
    rt.set_backend(&be);
    if (!persisted.empty())
      cvc::state::instance(app)("ui.tree." + id + ".columns").value(persisted);
    rt.set_root(ari::group({table(id)}));
    for (int f = 0; f < 3; ++f) {
      io.DeltaTime = 1.0f / 60.0f;
      ImGui::NewFrame();
      ImGui::SetNextWindowSize(ImVec2(520.0f, 200.0f));
      ImGui::Begin("host");
      rt.render();
      ImGui::End();
      ImGui::Render();
    }
    out = cvc::state::instance(app)("ui.tree." + id + ".columns").value();
  }
  ImGui::DestroyContext();
  return out;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  // Nothing persisted: the authored tracks, reported in px.
  const std::vector<float> fresh = widths(run("t1", ""));
  chk(fresh.size() == 3, "a resizable table reports one width per column");
  if (fresh.size() == 3) {
    chk(std::fabs(fresh[0] - 100.0f) < 0.5f, "the fixed column draws its authored 100 px");
    chk(fresh[1] > 0 && std::fabs(fresh[1] - fresh[2]) <= 1.0f,
        "equal weights draw equal stretch columns");
  }

  // Persisted widths: 160 px fixed, then stretch weights 3 : 1.
  const std::vector<float> seeded = widths(run("t2", "160,300,100"));
  chk(seeded.size() == 3, "a seeded table reports its widths");
  if (seeded.size() == 3) {
    chk(std::fabs(seeded[0] - 160.0f) < 0.5f, "the fixed column takes its persisted width");
    const double ratio = seeded[2] > 0 ? seeded[1] / seeded[2] : 0;
    chk(std::fabs(ratio - 3.0) < 0.1,
        "the stretch columns keep the persisted 3 : 1 proportions (got " + std::to_string(ratio) +
            ")");
  }

  // A round trip through what was persisted draws the same widths again.
  const std::vector<float> again = widths(run("t3", run("t3", "160,300,100")));
  chk(again == seeded, "persisting and reloading keeps the widths");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

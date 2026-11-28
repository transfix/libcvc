// ImGuiBackend's table (Kind::Table) on a bare ImGui context (no GL), through ImGui's own mouse
// input:
//   - the header is ImGui's own header row (TableHeader, in a row flagged as headers), the sorted
//     column's label with its arrow;
//   - a click on a sortable header writes the sort (ascending, then descending), and one on a
//     header with no sort: key writes nothing;
//   - every row's cells are cells of the one table, and its column widths persist.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <map>
#include <string>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // the current table's row flags
#endif

namespace ari = cvc::ariadne;

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

#ifdef CVC_ENABLE_IMGUI
// The ImGui backend, noting each header it drew (its text, its rect, whether its row is a header
// row) and the texts drawn this frame.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Header {
    std::string text;
    ImVec2 min, max;
    bool header_row = false;
  };
  std::map<std::string, Header> headers; // by label
  std::map<std::string, int> texts;
  int tables = 0; // tables open at a header: one ImGui table for the header and every row
  bool table_header(const std::string &label, int dir, bool sortable) override {
    Header h;
    h.text = ari::table_header_text(label, dir, unicode_glyphs());
    const ImGuiTable *t = ImGui::GetCurrentTable();
    h.header_row = t && (t->RowFlags & ImGuiTableRowFlags_Headers) != 0;
    const bool clicked = ImGuiBackend::table_header(label, dir, sortable);
    h.min = ImGui::GetItemRectMin();
    h.max = ImGui::GetItemRectMax();
    headers[label] = h;
    return clicked;
  }
  void text_line(const char *t) override {
    ++texts[t];
    ImGuiBackend::text_line(t);
  }
};

const char *kDoc = R"(meta: { name: table, min_libcvc: 4.0.0 }
windows:
  - window: Panel
    pos: [10, 10]
    size: [600, 400]
    children:
      - type: table
        id: t
        on: (state-set "ran" (state-get "tree.t.sort"))
        rows: 2
        borders: { show: inner }
        columns:
          - { label: open, width: 80 }
          - { label: name, width: 160, sort: name }
          - { label: size, width: 120, sort: size }
        cells:
          - text: o{i}
          - text: n{i}
          - text: s{i}
)";

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1000.0f, 700.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h);
    const ari::LoadResult r = ari::load_string(app, kDoc);
    for (const std::string &warn : r.warnings)
      chk(false, "loads without warnings: " + warn);
    rt.set_backend(&be);
    rt.set_root(r.root);
    for (int f = 0; f < 4; ++f)
      frame(ImVec2(990.0f, 690.0f), false);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame(ImVec2 mouse, bool down) {
    be.texts.clear();
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, down);
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    rt.drain();
  }
  void click(ImVec2 p) {
    frame(p, false);
    frame(p, true);
    frame(p, false);
    frame(p, false);
  }
  ImVec2 centre(const std::string &label) {
    const RecordingBackend::Header &h = be.headers[label];
    return ImVec2(0.5f * (h.min.x + h.max.x), 0.5f * (h.min.y + h.max.y));
  }
  std::string val(const std::string &key) { return cvc::state::instance(app)("ui." + key).value(); }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  Session s;
  chk(s.be.headers.size() == 3, "a header a column (" + std::to_string(s.be.headers.size()) + ")");
  chk(s.be.headers["name"].header_row && s.be.headers["open"].header_row,
      "in ImGui's header row (TableNextRow(ImGuiTableRowFlags_Headers))");
  chk(s.be.headers["name"].text == "name", "no sort yet: the label alone");
  for (const char *cell : {"o0", "n0", "s0", "o1", "n1", "s1"})
    chk(s.be.texts[cell] == 1, std::string("cell ") + cell + " drawn once");
  chk(s.be.headers["name"].max.x <= s.be.headers["size"].min.x &&
          s.be.headers["name"].min.y == s.be.headers["size"].min.y,
      "the headers side by side, on one row");

  s.click(s.centre("name"));
  chk(s.val("tree.t.sort") == "name", "a click sorts by it (" + s.val("tree.t.sort") + ")");
  chk(s.val("ran") == "name", "and runs on: after the write (" + s.val("ran") + ")");
  chk(s.be.headers["name"].text == "name \xE2\x96\xB2", "its header shows the arrow up");
  s.click(s.centre("name"));
  chk(s.val("tree.t.sort") == "-name", "again: descending (" + s.val("tree.t.sort") + ")");
  chk(s.be.headers["name"].text == "name \xE2\x96\xBC", "its header shows the arrow down");
  s.click(s.centre("size"));
  chk(s.val("tree.t.sort") == "size",
      "another column: it, ascending (" + s.val("tree.t.sort") + ")");
  s.click(s.centre("open"));
  chk(s.val("tree.t.sort") == "size", "a header with no sort: does nothing");
  chk(!s.val("tree.t.columns").empty(),
      "the column widths persist (" + s.val("tree.t.columns") + ")");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

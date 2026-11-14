// The overlay's fonts (ImGuiOverlay::loadUiFonts) on a bare ImGui context (no GL): the UI font,
// DejaVu Sans Mono's symbols under it, and, where ImGui has 32-bit characters and draws with
// FreeType (the cvcpkg imgui 1.92.9+cvc.4), Twemoji's emoji in colour. The fonts come from
// <build>/share/libcvc/fonts, beside the program, as an install's come from its share/.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <string>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImFontBaked, ImFontGlyph
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP cvcGL was built without CVC_ENABLE_IMGUI\n");
  return 0;
#else
  ImGuiContext *ctx = ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.DisplaySize = ImVec2(800.0f, 600.0f);
  // As the overlay's renderer does: the atlas grows its textures as glyphs are baked, and the
  // fonts are sized when drawn (the overlay adds them at size 0). No GL: nothing uploads them.
  io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
  cvc::app app; // the default UI font settings: an app no one has set any on
  cvc::gl::ImGuiOverlay::loadUiFonts(app, io);
  ImGui::GetStyle().FontSizeBase = 16.0f;
  ImGui::NewFrame();
  ImFont *font = io.Fonts->Fonts.Size > 0 ? io.Fonts->Fonts[0] : nullptr;
  chk(font != nullptr, "the overlay loaded a UI font");
  if (!font)
    return 1;

  // A feed's counts (↩ ⇄ ♥): arrows and symbols most UI fonts lack, from DejaVu Sans Mono.
  chk(font->IsGlyphInFont(0x21A9), "U+21A9 (leftwards arrow with hook) has a glyph");
  chk(font->IsGlyphInFont(0x21C4), "U+21C4 (right arrow over left arrow) has a glyph");
  chk(font->IsGlyphInFont(0x2665), "U+2665 (black heart suit) has a glyph");

#if defined(CVC_IMGUI_FREETYPE)
  if (sizeof(ImWchar) >= 4) {
    const ImWchar hundred = static_cast<ImWchar>(0x1F4AF);
    chk(font->IsGlyphInFont(hundred), "U+1F4AF (hundred points) has a glyph: Twemoji's");
    ImFontBaked *baked = font->GetFontBaked(32.0f);
    const ImFontGlyph *g = baked ? baked->FindGlyph(hundred) : nullptr;
    chk(g != nullptr && g->Codepoint == hundred, "... which bakes at 32 px");
    chk(g != nullptr && g->Colored, "... in colour (FreeType, COLR)");
  } else {
    std::printf("SKIP emoji: this imgui's ImWchar is 16-bit (cvcpkg imgui before 1.92.9+cvc.4)\n");
  }
#else
  std::printf("SKIP emoji: cvcGL draws with stb_truetype (no FreeType loader in this imgui)\n");
#endif

  ImGui::EndFrame();
  ImGui::DestroyContext(ctx);
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

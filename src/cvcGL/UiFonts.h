// UiFonts — where ImGuiOverlay finds its fallback fonts (fonts/README.md). Internal to cvcGL.
#ifndef CVC_GL_UI_FONTS_H
#define CVC_GL_UI_FONTS_H

#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace detail {

// The directories that may hold the bundled fonts, in order: CVC_UI_FONT_DIR; share/libcvc/fonts
// beside the library or program holding cvcGL (<bin or lib>/..), which a relocated install and a
// build tree both have; the install's own data directory; and in a browser build the embedded
// /usr/share/libcvc/fonts.
std::vector<std::string> ui_font_dirs();

// The first readable `name` in ui_font_dirs(); "" when none has it.
std::string bundled_ui_font(const char *name);

// The system's colour emoji font (Segoe UI Emoji, Apple Color Emoji or Noto Color Emoji), or "".
std::string system_emoji_font();

} // namespace detail
} // namespace gl
} // namespace cvc

#endif // CVC_GL_UI_FONTS_H

// UiFonts — where ImGuiOverlay finds its fallback fonts. See UiFonts.h.

#include "UiFonts.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <system_error>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <dlfcn.h>
#endif

namespace cvc {
namespace gl {
namespace detail {

namespace {

bool readable(const std::string &path) {
  if (path.empty())
    return false;
  if (FILE *f = std::fopen(path.c_str(), "rb")) {
    std::fclose(f);
    return true;
  }
  return false;
}

// <the file holding cvcGL>/../share/libcvc/fonts: cvcGL's shared library (lib/ on POSIX, bin/ on
// Windows) or the program that links it statically. A cvcpkg install unpacked anywhere has it, and
// so does a build tree (<build>/bin and <build>/share), or ../../share from a multi-config build's
// <build>/bin/<Config>. Empty where it can't be found.
std::string beside_module() {
  namespace fs = std::filesystem;
  fs::path module;
#if defined(_WIN32)
  HMODULE h = nullptr;
  if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                             GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                         reinterpret_cast<LPCWSTR>(&beside_module), &h)) {
    std::wstring buf(32768, L'\0');
    const DWORD n = GetModuleFileNameW(h, buf.data(), static_cast<DWORD>(buf.size()));
    if (n > 0 && n < buf.size())
      module = fs::path(buf.substr(0, n));
  }
#elif !defined(__EMSCRIPTEN__)
  Dl_info info;
  if (dladdr(reinterpret_cast<const void *>(&beside_module), &info) && info.dli_fname)
    module = fs::path(info.dli_fname);
#endif
  if (module.empty())
    return std::string();
  for (const fs::path up : {fs::path(".."), fs::path("..") / ".."}) {
    std::error_code ec;
    const fs::path dir =
        fs::weakly_canonical(module.parent_path() / up / "share" / "libcvc" / "fonts", ec);
    if (!ec && fs::is_directory(dir, ec))
      return dir.string();
  }
  return std::string();
}

} // namespace

std::vector<std::string> ui_font_dirs() {
  std::vector<std::string> dirs;
  if (const char *e = std::getenv("CVC_UI_FONT_DIR"); e && *e)
    dirs.push_back(e);
#ifdef __EMSCRIPTEN__
  dirs.push_back("/usr/share/libcvc/fonts"); // the browser hosts embed them there
#else
  if (std::string d = beside_module(); !d.empty())
    dirs.push_back(d);
#endif
#ifdef CVC_GL_FONT_DIR
  dirs.push_back(CVC_GL_FONT_DIR); // the install's data directory, where it was built for
#endif
  return dirs;
}

std::string bundled_ui_font(const char *name) {
  for (const std::string &d : ui_font_dirs()) {
    const std::string p = d + "/" + name;
    if (readable(p))
      return p;
  }
  return std::string();
}

std::string system_emoji_font() {
  static const char *const kCandidates[] = {
      "C:\\Windows\\Fonts\\seguiemj.ttf",                      // Windows: Segoe UI Emoji (COLR)
      "/System/Library/Fonts/Apple Color Emoji.ttc",           // macOS (sbix)
      "/usr/share/fonts/truetype/noto/NotoColorEmoji.ttf",     // Debian/Ubuntu (CBDT)
      "/usr/share/fonts/noto/NotoColorEmoji.ttf",              // Arch
      "/usr/share/fonts/google-noto-emoji/NotoColorEmoji.ttf", // Fedora
      "/usr/local/share/fonts/noto/NotoColorEmoji.ttf",        // FreeBSD
  };
  for (const char *c : kCandidates)
    if (readable(c))
      return c;
  return std::string();
}

} // namespace detail
} // namespace gl
} // namespace cvc

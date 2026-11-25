// Ariadne — saving a picture or a video the user sees. See media_save.h.

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/files.h>
#include <cvc/ariadne/media_save.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
// windows.h comes first.
// clang-format off
#include <windows.h>
#include <shlobj.h> // SHGetKnownFolderPath: the Downloads known folder
#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>
// clang-format on
#ifdef _MSC_VER
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "ole32.lib") // CoTaskMemFree
#endif
#else
#include <fcntl.h>
#include <unistd.h>
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace cvc {
namespace ariadne {

namespace {

namespace fs = std::filesystem;

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool starts(std::string_view b, std::size_t at, std::string_view what) {
  return b.size() >= at + what.size() && b.compare(at, what.size(), what) == 0;
}

// Whether `needle` is in the first `within` bytes of `b`.
bool within(std::string_view b, std::string_view needle, std::size_t within) {
  return b.substr(0, std::min(b.size(), within)).find(needle) != std::string_view::npos;
}

int hex_digit(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return -1;
}

std::string percent_decoded(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '%' && i + 2 < s.size()) {
      const int hi = hex_digit(s[i + 1]), lo = hex_digit(s[i + 2]);
      if (hi >= 0 && lo >= 0) {
        out += static_cast<char>(hi * 16 + lo);
        i += 2;
        continue;
      }
    }
    out += s[i];
  }
  return out;
}

// The URL's last path part, without its query or fragment: "" for a URL that names no path part
// (an origin, a path ending in '/').
std::string url_last_part(const std::string &url) {
  std::string s = url.substr(0, url.find_first_of("?#"));
  const std::size_t scheme = s.find("://");
  if (scheme != std::string::npos) {
    const std::string sch = lower(s.substr(0, scheme));
    s = s.substr(scheme + 3);
    if (sch == "http" || sch == "https") { // the authority is not a file's name
      const std::size_t slash = s.find('/');
      s = slash == std::string::npos ? std::string() : s.substr(slash);
    }
  }
  const std::size_t last = s.find_last_of('/');
  // A slash it encodes (%2F) is part of its name, not a folder.
  std::string part = percent_decoded(last == std::string::npos ? s : s.substr(last + 1));
  std::replace(part.begin(), part.end(), '/', '_');
  std::replace(part.begin(), part.end(), '\\', '_');
  return part;
}

// Invalid UTF-8 (a percent-decoded byte that starts nothing) as '_': a path is made of UTF-8.
std::string valid_utf8(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  std::size_t i = 0;
  while (i < s.size()) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    const std::size_t n = c < 0x80         ? 1
                          : (c >> 5) == 6  ? 2
                          : (c >> 4) == 14 ? 3
                          : (c >> 3) == 30 ? 4
                                           : 0;
    bool ok = n > 0 && i + n <= s.size();
    for (std::size_t k = 1; ok && k < n; ++k)
      ok = (static_cast<unsigned char>(s[i + k]) >> 6) == 2;
    if (!ok) {
      out += '_';
      ++i;
      continue;
    }
    out.append(s, i, n);
    i += n;
  }
  return out;
}

// The lowercase extension of a file name, without the dot; "" when it has none (a leading dot is
// not one).
std::string name_extension(const std::string &name) {
  const std::size_t dot = name.find_last_of('.');
  if (dot == std::string::npos || dot == 0 || dot + 1 >= name.size())
    return std::string();
  return lower(name.substr(dot + 1));
}

// The name a file system takes: its last path part, with control characters and <>:"/\|?* as '_',
// no blanks at its ends, no dot at its start (a hidden file) or its end (Windows drops it), a
// reserved Windows name prefixed, and at most 200 bytes, the extension kept.
std::string clean_name(const std::string &raw) {
  std::string s = raw;
  const std::size_t slash = s.find_last_of("/\\");
  if (slash != std::string::npos)
    s = s.substr(slash + 1);
  s = valid_utf8(s);
  for (char &c : s) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (u < 0x20 || u == 0x7f || std::strchr("<>:\"/\\|?*", c))
      c = '_';
  }
  const std::size_t a = s.find_first_not_of(" ");
  s = a == std::string::npos ? std::string() : s.substr(a);
  while (!s.empty() && (s.back() == ' ' || s.back() == '.'))
    s.pop_back();
  if (!s.empty() && s.front() == '.')
    s.front() = '_';
  if (!s.empty()) {
    const std::string stem = lower(s.substr(0, s.find('.')));
    static const char *const kReserved[] = {"con", "prn", "aux", "nul"};
    bool reserved =
        std::find(std::begin(kReserved), std::end(kReserved), stem) != std::end(kReserved);
    if (stem.size() == 4 && (stem.compare(0, 3, "com") == 0 || stem.compare(0, 3, "lpt") == 0) &&
        stem[3] >= '1' && stem[3] <= '9')
      reserved = true;
    if (reserved)
      s = "_" + s;
  }
  constexpr std::size_t kMax = 200;
  if (s.size() > kMax) {
    const std::string ext = name_extension(s);
    const std::size_t keep_ext = ext.empty() || ext.size() > 16 ? 0 : ext.size() + 1;
    std::size_t cut = kMax - keep_ext;
    while (cut > 0 && (static_cast<unsigned char>(s[cut]) >> 6) == 2)
      --cut; // not inside a UTF-8 sequence
    s = s.substr(0, cut) + (keep_ext ? s.substr(s.size() - keep_ext) : std::string());
  }
  return s;
}

} // namespace

std::string sniff_mime(std::string_view b) {
  if (starts(b, 0, "\x89PNG\r\n\x1a\n"))
    return "image/png";
  if (starts(b, 0, "\xff\xd8\xff"))
    return "image/jpeg";
  if (starts(b, 0, "GIF87a") || starts(b, 0, "GIF89a"))
    return "image/gif";
  if (starts(b, 0, "RIFF") && starts(b, 8, "WEBP"))
    return "image/webp";
  if (starts(b, 0, "RIFF") && starts(b, 8, "WAVE"))
    return "audio/wav";
  if (starts(b, 0, std::string_view("II*\0", 4)) || starts(b, 0, std::string_view("MM\0*", 4)))
    return "image/tiff";
  if (starts(b, 0, "BM") && b.size() > 26)
    return "image/bmp";
  if (starts(b, 0, std::string_view("\0\0\1\0", 4)) && b.size() > 22)
    return "image/x-icon";
  if (starts(b, 4, "ftyp")) { // ISO base media: the major brand says which
    const std::string brand(b.substr(8, std::min<std::size_t>(4, b.size() > 8 ? b.size() - 8 : 0)));
    if (brand == "avif" || brand == "avis")
      return "image/avif";
    if (brand == "heic" || brand == "heix" || brand == "mif1" || brand == "msf1")
      return "image/heic";
    if (brand == "qt  ")
      return "video/quicktime";
    if (brand == "M4A " || brand == "M4B ")
      return "audio/mp4";
    return "video/mp4";
  }
  if (starts(b, 0, "\x1a\x45\xdf\xa3")) // EBML: WebM is Matroska with the doctype "webm"
    return within(b, "webm", 64) ? "video/webm" : "video/x-matroska";
  if (starts(b, 0, "OggS"))
    return within(b, "\x80theora", 512) ? "video/ogg" : "audio/ogg";
  if (starts(b, 0, "fLaC"))
    return "audio/flac";
  if (starts(b, 0, "ID3") || (b.size() > 2 && static_cast<unsigned char>(b[0]) == 0xff &&
                              (static_cast<unsigned char>(b[1]) & 0xe6) == 0xe2))
    return "audio/mpeg";
  std::string_view t = b.substr(0, std::min<std::size_t>(b.size(), 1024));
  if (starts(t, 0, "\xef\xbb\xbf"))
    t.remove_prefix(3);
  while (!t.empty() && std::isspace(static_cast<unsigned char>(t.front())))
    t.remove_prefix(1);
  if ((starts(t, 0, "<?xml") || starts(t, 0, "<svg")) && t.find("<svg") != std::string_view::npos)
    return "image/svg+xml";
  return std::string();
}

std::string save_file_name(const std::string &url, const std::string &name, const std::string &mime,
                           bool video) {
  std::string out = clean_name(name.empty() ? url_last_part(url) : name);
  const std::string major = lower(mime.substr(0, mime.find('/')));
  const std::string want = mime_extension(mime);
  if (out.empty()) {
    if (major == "image" || major == "video" || major == "audio")
      out = major;
    else if (!want.empty())
      out = "file";
    else
      out = video ? "video" : "image";
  }
  if (want.empty())
    return out;
  const std::string ext = name_extension(out);
  if (ext.empty())
    return out + "." + want;
  if (mime_extension(guess_mime_type(out)) == want)
    return out; // its own extension for the type ("a.jpeg", "a.m4v")
  if (guess_mime_type(out) != "application/octet-stream")
    return out.substr(0, out.size() - ext.size()) + want; // another type's: the bytes' instead
  return out + "." + want;
}

std::string downloads_dir(std::string *error) {
  const auto made = [&](const fs::path &p) -> std::string {
    std::error_code ec;
    fs::create_directories(p, ec);
    if (!fs::is_directory(p, ec)) {
      if (error)
        *error = path_utf8(p) + ": can't make the folder";
      return std::string();
    }
    return path_utf8(p);
  };
#ifdef _WIN32
  // FOLDERID_Downloads, spelled out so nothing links uuid.lib for it.
  static const GUID kDownloads = {
      0x374DE290, 0x123F, 0x4565, {0x91, 0x64, 0x39, 0xC4, 0x92, 0x5E, 0x46, 0x7B}};
  PWSTR p = nullptr;
  const HRESULT hr = SHGetKnownFolderPath(kDownloads, KF_FLAG_CREATE, nullptr, &p);
  if (SUCCEEDED(hr) && p) {
    const fs::path d(p);
    CoTaskMemFree(p);
    return made(d);
  }
  CoTaskMemFree(p);
  if (const wchar_t *home = _wgetenv(L"USERPROFILE"); home && *home)
    return made(fs::path(home) / L"Downloads");
  if (error)
    *error = "Windows did not say where the Downloads folder is";
  return std::string();
#else
  if (const char *x = std::getenv("XDG_DOWNLOAD_DIR"); x && *x == '/')
    return made(utf8_path(x));
  const char *home_env = std::getenv("HOME");
  if (!home_env || !*home_env) {
    if (error)
      *error = "there is no home folder (HOME) to find Downloads in";
    return std::string();
  }
  const std::string home = home_env;
  // user-dirs.dirs, as xdg-user-dirs writes it: XDG_DOWNLOAD_DIR="$HOME/Downloads".
  const char *cfg = std::getenv("XDG_CONFIG_HOME");
  const std::string dirs =
      (cfg && *cfg == '/' ? std::string(cfg) : home + "/.config") + "/user-dirs.dirs";
  std::ifstream in(utf8_path(dirs));
  std::string line;
  while (in && std::getline(in, line)) {
    const std::size_t a = line.find_first_not_of(" \t");
    if (a == std::string::npos || line.compare(a, 17, "XDG_DOWNLOAD_DIR=") != 0)
      continue;
    std::string v = line.substr(a + 17);
    while (!v.empty() && (v.back() == '\r' || v.back() == ' ' || v.back() == '\t'))
      v.pop_back();
    if (v.size() >= 2 && v.front() == '"' && v.back() == '"')
      v = v.substr(1, v.size() - 2);
    if (v.compare(0, 5, "$HOME") == 0)
      v = home + v.substr(5);
    if (!v.empty() && v.front() == '/' && v != home && v != home + "/")
      return made(utf8_path(v));
  }
  return made(utf8_path(home) / "Downloads");
#endif
}

namespace {

// Write all of `bytes` to the open descriptor `fd`; false on an error (errno says it).
bool write_all(int fd, std::string_view bytes) {
  std::size_t at = 0;
  while (at < bytes.size()) {
    const std::size_t n = std::min<std::size_t>(bytes.size() - at, 1u << 20);
#ifdef _WIN32
    const int w = _write(fd, bytes.data() + at, static_cast<unsigned int>(n));
#else
    const ssize_t w = ::write(fd, bytes.data() + at, n);
#endif
    if (w < 0) {
#ifndef _WIN32
      if (errno == EINTR)
        continue;
#endif
      return false;
    }
    at += static_cast<std::size_t>(w);
  }
  return true;
}

// Make `p` new (failing with EEXIST when it is there) and write `bytes` into it. 1: written, 0: it
// is there, -1: another error (in `error`).
int create_new(const fs::path &p, std::string_view bytes, std::string &error) {
#ifdef _WIN32
  const int fd =
      _wopen(p.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _S_IREAD | _S_IWRITE);
#else
  int flags = O_WRONLY | O_CREAT | O_EXCL;
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif
  const int fd = ::open(p.c_str(), flags, 0666);
#endif
  if (fd < 0) {
    if (errno == EEXIST)
      return 0;
    error = path_utf8(p) + ": " + std::strerror(errno);
    return -1;
  }
  const bool ok = write_all(fd, bytes);
  const int why = errno;
#ifdef _WIN32
  const bool closed = _close(fd) == 0;
#else
  const bool closed = ::close(fd) == 0;
#endif
  if (!ok || !closed) {
    error = path_utf8(p) + ": " + std::strerror(ok ? errno : why);
    std::error_code ec;
    fs::remove(p, ec);
    return -1;
  }
  return 1;
}

} // namespace

std::string save_new_file(const std::string &dir, const std::string &name, std::string_view bytes,
                          std::string *error) {
  std::string why;
  const std::string clean = clean_name(name).empty() ? std::string("download") : clean_name(name);
  const std::string ext = name_extension(clean);
  const std::string stem = ext.empty() ? clean : clean.substr(0, clean.size() - ext.size() - 1);
  const std::string dot_ext =
      ext.empty() ? std::string() : clean.substr(clean.size() - ext.size() - 1);
  for (int n = 0; n < 10000; ++n) {
    const std::string file = n == 0 ? clean : stem + " (" + std::to_string(n) + ")" + dot_ext;
    const fs::path p = utf8_path(dir) / utf8_path(file);
    const int r = create_new(p, bytes, why);
    if (r == 1)
      return path_utf8(p);
    if (r < 0)
      break;
  }
  if (error)
    *error = why.empty() ? std::string("every name like ") + clean + " is taken in " + dir : why;
  return std::string();
}

bool save_file_at(const std::string &path, std::string_view bytes, std::string *error) {
  const fs::path p = utf8_path(path);
  {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    if (out) {
      out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
      out.close();
      if (out)
        return true;
    }
  }
  if (error)
    *error = path + ": can't write it";
  std::error_code ec;
  fs::remove(p, ec);
  return false;
}

#ifdef __EMSCRIPTEN__

// The page's side: Module.cvcSave holds the download staged ({key, name, url, clicked}), the Save
// item armed under the pointer, and a release over an armed item whose bytes had not come yet
// (want). The pointer listeners run in the capture phase, before the canvas's own, so a download
// a release starts is inside its click.
// clang-format off
EM_JS(int, cvc_save_install, (), {
  if (typeof document === 'undefined' || !document.body || typeof URL === 'undefined' ||
      typeof Blob === 'undefined')
    return 0;
  if (Module.cvcSave) return 1;
  var S = Module.cvcSave = {staged: null, arm: null, downArm: null, want: null};
  S.click = function(st) {
    st.clicked = true;
    var a = document.createElement('a');
    a.href = st.url;
    a.download = st.name;
    a.rel = 'noopener';
    a.style.display = 'none';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    // The browser holds the Blob once its download has started; the URL goes a minute later.
    setTimeout(function() { URL.revokeObjectURL(st.url); }, 60000);
  };
  S.drop = function() {
    var st = S.staged;
    S.staged = null;
    if (st && !st.clicked) URL.revokeObjectURL(st.url);
  };
  window.addEventListener('pointerdown', function(e) {
    S.downArm = e.button === 0 ? S.arm : null;
  }, true);
  window.addEventListener('pointerup', function(e) {
    var k = S.arm;
    if (k === null || e.button !== 0 || (k !== S.downArm && e.pointerType === 'mouse')) return;
    var st = S.staged;
    if (st && st.key === k) {
      if (!st.clicked) S.click(st);
    } else {
      S.want = {key: k, t: Date.now()};
    }
  }, true);
  return 1;
});

EM_JS(int, cvc_save_stage, (const char *key, const char *name, const char *mime,
                            const unsigned char *bytes, double n), {
  var S = Module.cvcSave;
  if (!S) return 0;
  var k = UTF8ToString(key);
  S.drop();
  var type = UTF8ToString(mime);
  var blob = new Blob([HEAPU8.slice(bytes, bytes + n)], type ? {type: type} : {});
  var st = S.staged = {key: k, name: UTF8ToString(name), url: URL.createObjectURL(blob),
                       clicked: false};
  // A menu opened and left: its bytes (a video's can be large) go after two minutes.
  setTimeout(function() { if (S.staged === st && !st.clicked) S.drop(); }, 120000);
  if (S.want && S.want.key === k && Date.now() - S.want.t < 60000) {
    S.want = null;
    S.click(st); // the release over its item came before the bytes
  }
  return 1;
});

EM_JS(int, cvc_save_commit, (const char *key), {
  var S = Module.cvcSave;
  var st = S && S.staged;
  if (!st || st.key !== UTF8ToString(key)) return 0;
  if (!st.clicked) S.click(st);
  S.staged = null; // the download has it; the timer lets its URL go
  return 1;
});

EM_JS(void, cvc_save_drop, (const char *key), {
  var S = Module.cvcSave;
  if (S && S.staged && S.staged.key === UTF8ToString(key)) S.drop();
  if (S && S.want && S.want.key === UTF8ToString(key)) S.want = null;
});

EM_JS(void, cvc_save_arm, (const char *key), {
  var S = Module.cvcSave;
  if (S) S.arm = key ? UTF8ToString(key) : null;
});
// clang-format on

bool browser_save_stage(const std::string &key, const std::string &name, const std::string &mime,
                        std::string_view bytes) {
  if (!cvc_save_install())
    return false;
  return cvc_save_stage(key.c_str(), name.c_str(), mime.c_str(),
                        reinterpret_cast<const unsigned char *>(bytes.data()),
                        static_cast<double>(bytes.size())) != 0;
}

bool browser_save_commit(const std::string &key) {
  return cvc_save_install() && cvc_save_commit(key.c_str()) != 0;
}

void browser_save_drop(const std::string &key) {
  if (cvc_save_install())
    cvc_save_drop(key.c_str());
}

void browser_save_arm(const std::string *key) {
  if (cvc_save_install())
    cvc_save_arm(key ? key->c_str() : nullptr);
}

bool browser_save_file(const std::string &name, const std::string &mime, std::string_view bytes) {
  static constexpr const char *kKey = "cvc.save.now";
  return browser_save_stage(kKey, name, mime, bytes) && browser_save_commit(kKey);
}

#else

bool browser_save_stage(const std::string &, const std::string &, const std::string &,
                        std::string_view) {
  return false;
}
bool browser_save_commit(const std::string &) { return false; }
void browser_save_drop(const std::string &) {}
void browser_save_arm(const std::string *) {}
bool browser_save_file(const std::string &, const std::string &, std::string_view) { return false; }

#endif

} // namespace ariadne
} // namespace cvc

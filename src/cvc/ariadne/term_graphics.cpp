// Terminal pixel graphics: Kitty / Sixel encoders and the capability probe (see the header).

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib> // std::getenv -- $TERM_PROGRAM
#include <cvc/ariadne/term_graphics.h>
#include <iostream>
#include <string>
#include <vector>

// stb's JPEG encoder, for iTerm2 inline images: a static copy of its own (image/stb_io.cpp has
// another), so the terminal backend needs no image library.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#include "../image/third_party/stb_image_write.h"

#ifdef __EMSCRIPTEN__
#include <unistd.h> // isatty: $TERM_PROGRAM counts only for a terminal
#endif

#if !defined(_WIN32) && !defined(__EMSCRIPTEN__)
#define CVC_TERM_GRAPHICS_POSIX 1
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace cvc {
namespace ariadne {
namespace gfx {

namespace {

// The combining diacritics (class 230) kitty numbers placeholder rows and columns with: entry i
// stands for row / column i. From kitty's rowcolumn-diacritics.txt.
constexpr std::uint32_t kDiacritics[kKittyMaxCells] = {
    0x0305,  0x030D,  0x030E,  0x0310,  0x0312,  0x033D,  0x033E,  0x033F,  0x0346,  0x034A,
    0x034B,  0x034C,  0x0350,  0x0351,  0x0352,  0x0357,  0x035B,  0x0363,  0x0364,  0x0365,
    0x0366,  0x0367,  0x0368,  0x0369,  0x036A,  0x036B,  0x036C,  0x036D,  0x036E,  0x036F,
    0x0483,  0x0484,  0x0485,  0x0486,  0x0487,  0x0592,  0x0593,  0x0594,  0x0595,  0x0597,
    0x0598,  0x0599,  0x059C,  0x059D,  0x059E,  0x059F,  0x05A0,  0x05A1,  0x05A8,  0x05A9,
    0x05AB,  0x05AC,  0x05AF,  0x05C4,  0x0610,  0x0611,  0x0612,  0x0613,  0x0614,  0x0615,
    0x0616,  0x0617,  0x0657,  0x0658,  0x0659,  0x065A,  0x065B,  0x065D,  0x065E,  0x06D6,
    0x06D7,  0x06D8,  0x06D9,  0x06DA,  0x06DB,  0x06DC,  0x06DF,  0x06E0,  0x06E1,  0x06E2,
    0x06E4,  0x06E7,  0x06E8,  0x06EB,  0x06EC,  0x0730,  0x0732,  0x0733,  0x0735,  0x0736,
    0x073A,  0x073D,  0x073F,  0x0740,  0x0741,  0x0743,  0x0745,  0x0747,  0x0749,  0x074A,
    0x07EB,  0x07EC,  0x07ED,  0x07EE,  0x07EF,  0x07F0,  0x07F1,  0x07F3,  0x0816,  0x0817,
    0x0818,  0x0819,  0x081B,  0x081C,  0x081D,  0x081E,  0x081F,  0x0820,  0x0821,  0x0822,
    0x0823,  0x0825,  0x0826,  0x0827,  0x0829,  0x082A,  0x082B,  0x082C,  0x082D,  0x0951,
    0x0953,  0x0954,  0x0F82,  0x0F83,  0x0F86,  0x0F87,  0x135D,  0x135E,  0x135F,  0x17DD,
    0x193A,  0x1A17,  0x1A75,  0x1A76,  0x1A77,  0x1A78,  0x1A79,  0x1A7A,  0x1A7B,  0x1A7C,
    0x1B6B,  0x1B6D,  0x1B6E,  0x1B6F,  0x1B70,  0x1B71,  0x1B72,  0x1B73,  0x1CD0,  0x1CD1,
    0x1CD2,  0x1CDA,  0x1CDB,  0x1CE0,  0x1DC0,  0x1DC1,  0x1DC3,  0x1DC4,  0x1DC5,  0x1DC6,
    0x1DC7,  0x1DC8,  0x1DC9,  0x1DCB,  0x1DCC,  0x1DD1,  0x1DD2,  0x1DD3,  0x1DD4,  0x1DD5,
    0x1DD6,  0x1DD7,  0x1DD8,  0x1DD9,  0x1DDA,  0x1DDB,  0x1DDC,  0x1DDD,  0x1DDE,  0x1DDF,
    0x1DE0,  0x1DE1,  0x1DE2,  0x1DE3,  0x1DE4,  0x1DE5,  0x1DE6,  0x1DFE,  0x20D0,  0x20D1,
    0x20D4,  0x20D5,  0x20D6,  0x20D7,  0x20DB,  0x20DC,  0x20E1,  0x20E7,  0x20E9,  0x20F0,
    0x2CEF,  0x2CF0,  0x2CF1,  0x2DE0,  0x2DE1,  0x2DE2,  0x2DE3,  0x2DE4,  0x2DE5,  0x2DE6,
    0x2DE7,  0x2DE8,  0x2DE9,  0x2DEA,  0x2DEB,  0x2DEC,  0x2DED,  0x2DEE,  0x2DEF,  0x2DF0,
    0x2DF1,  0x2DF2,  0x2DF3,  0x2DF4,  0x2DF5,  0x2DF6,  0x2DF7,  0x2DF8,  0x2DF9,  0x2DFA,
    0x2DFB,  0x2DFC,  0x2DFD,  0x2DFE,  0x2DFF,  0xA66F,  0xA67C,  0xA67D,  0xA6F0,  0xA6F1,
    0xA8E0,  0xA8E1,  0xA8E2,  0xA8E3,  0xA8E4,  0xA8E5,  0xA8E6,  0xA8E7,  0xA8E8,  0xA8E9,
    0xA8EA,  0xA8EB,  0xA8EC,  0xA8ED,  0xA8EE,  0xA8EF,  0xA8F0,  0xA8F1,  0xAAB0,  0xAAB2,
    0xAAB3,  0xAAB7,  0xAAB8,  0xAABE,  0xAABF,  0xAAC1,  0xFE20,  0xFE21,  0xFE22,  0xFE23,
    0xFE24,  0xFE25,  0xFE26,  0x10A0F, 0x10A38, 0x1D185, 0x1D186, 0x1D187, 0x1D188, 0x1D189,
    0x1D1AA, 0x1D1AB, 0x1D1AC, 0x1D1AD, 0x1D242, 0x1D243, 0x1D244,
};

void put_utf8(std::string &s, std::uint32_t cp) {
  if (cp < 0x80) {
    s += static_cast<char>(cp);
  } else if (cp < 0x800) {
    s += static_cast<char>(0xC0 | (cp >> 6));
    s += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
    s += static_cast<char>(0xE0 | (cp >> 12));
    s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    s += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    s += static_cast<char>(0xF0 | (cp >> 18));
    s += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
    s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    s += static_cast<char>(0x80 | (cp & 0x3F));
  }
}

// The numeric parameters of a CSI sequence, "" fields as 0.
std::vector<int> csi_params(const std::string &s) {
  std::vector<int> out;
  int v = 0;
  bool any = false;
  for (const char c : s) {
    if (c >= '0' && c <= '9') {
      v = v * 10 + (c - '0');
      any = true;
    } else if (c == ';') {
      out.push_back(v);
      v = 0;
      any = false;
    }
  }
  if (any || !out.empty())
    out.push_back(v);
  return out;
}

} // namespace

// ---- probing --------------------------------------------------------------------------------

std::string probe_query(std::uint32_t kitty_id) {
  // A 1x1 RGB image is 3 bytes: base64 "AAAA".
  return "\x1b_Gi=" + std::to_string(kitty_id) + ",s=1,v=1,a=q,t=d,f=24;AAAA\x1b\\" + "\x1b[16t" +
         "\x1b[c";
}

std::string sixel_colors_query() { return "\x1b[?1;1;0S\x1b[c"; }

bool parse_probe_reply(const std::string &r, std::uint32_t kitty_id, TermGraphics &g) {
  bool done = false;
  const std::size_t n = r.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (r[i] != '\x1b' || i + 1 >= n)
      continue;
    if (r[i + 1] == '_') { // APC: ESC _ G <keys> ; <message> ESC \  (a kitty graphics reply)
      const std::size_t end = r.find("\x1b\\", i + 2);
      if (end == std::string::npos)
        break;
      const std::string body = r.substr(i + 2, end - i - 2);
      if (!body.empty() && body[0] == 'G') {
        const std::size_t semi = body.find(';');
        const std::string keys = body.substr(1, semi == std::string::npos ? semi : semi - 1);
        const std::string msg = semi == std::string::npos ? std::string() : body.substr(semi + 1);
        if (("," + keys + ",").find(",i=" + std::to_string(kitty_id) + ",") != std::string::npos)
          g.kitty = msg == "OK";
      }
      i = end + 1;
      continue;
    }
    if (r[i + 1] != '[')
      continue;
    // CSI: parameter bytes 0x30-0x3F, then a final byte 0x40-0x7E.
    std::size_t j = i + 2;
    while (j < n && r[j] >= 0x30 && r[j] <= 0x3F)
      ++j;
    while (j < n && r[j] >= 0x20 && r[j] <= 0x2F) // intermediate bytes
      ++j;
    if (j >= n)
      break;
    const char fin = r[j];
    const std::string params = r.substr(i + 2, j - i - 2);
    const bool question = !params.empty() && params[0] == '?';
    const std::vector<int> p = csi_params(params);
    if (fin == 't' && !question && p.size() >= 3 && p[0] == 6) { // CSI 6 ; h ; w t
      if (p[1] > 0 && p[2] > 0) {
        g.cell_h = p[1];
        g.cell_w = p[2];
      }
    } else if (fin == 'S' && question && p.size() >= 3 && p[0] == 1 && p[1] == 0) {
      g.sixel_colors = p[2];             // CSI ? 1 ; 0 ; registers S
    } else if (fin == 'c' && question) { // DA1: CSI ? <class> ; <feature> ... c
      g.answered = true;
      for (std::size_t k = 1; k < p.size(); ++k)
        if (p[k] == 4)
          g.sixel = true;
      done = true;
    }
    i = j;
  }
  return done;
}

bool cell_pixel_size(int fd, int &w, int &h) {
#ifdef CVC_TERM_GRAPHICS_POSIX
  struct winsize ws {};
  if (ioctl(fd, TIOCGWINSZ, &ws) != 0 || ws.ws_col == 0 || ws.ws_row == 0 || ws.ws_xpixel == 0 ||
      ws.ws_ypixel == 0)
    return false;
  w = ws.ws_xpixel / ws.ws_col;
  h = ws.ws_ypixel / ws.ws_row;
  return w > 0 && h > 0;
#else
  (void)fd;
  (void)w;
  (void)h;
  return false;
#endif
}

TermGraphics probe_terminal(int timeout_ms) {
  TermGraphics g;
#ifndef _WIN32
  // Not on Windows: ConPTY forwards an iTerm2 image ahead of the redraws it makes itself, so a
  // full-screen program's images land misplaced and are painted over (ARIADNE.md section 4).
  if (const char *tp = std::getenv("TERM_PROGRAM"); tp && isatty(STDOUT_FILENO))
    g.iterm = iterm_terminal(tp);
#endif
#ifdef CVC_TERM_GRAPHICS_POSIX
  if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO))
    return g;
  const int fd = open("/dev/tty", O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd < 0)
    return g;
  struct termios saved {};
  if (tcgetattr(fd, &saved) != 0) {
    close(fd);
    return g;
  }
  struct termios raw = saved;
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &raw);
  std::cout.flush(); // what the host printed before goes first
  std::fflush(stdout);

  // A random-ish id, so a reply meant for another program can't be taken for ours.
  const std::uint32_t id = 0x7A000000u | (static_cast<std::uint32_t>(getpid()) & 0xFFFFFFu);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  auto exchange = [&](const std::string &query) {
    for (std::size_t off = 0; off < query.size();) {
      const ssize_t k = write(fd, query.data() + off, query.size() - off);
      if (k <= 0)
        return;
      off += static_cast<std::size_t>(k);
    }
    std::string reply;
    TermGraphics seen = g;
    while (true) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                            deadline - std::chrono::steady_clock::now())
                            .count();
      if (left <= 0)
        break;
      struct pollfd pfd {
        fd, POLLIN, 0
      };
      if (poll(&pfd, 1, static_cast<int>(left)) <= 0)
        break;
      char buf[512];
      const ssize_t k = read(fd, buf, sizeof buf);
      if (k <= 0)
        break;
      reply.append(buf, static_cast<std::size_t>(k));
      seen = g;
      if (parse_probe_reply(reply, id, seen))
        break;
    }
    g = seen;
  };
  exchange(probe_query(id));
  if (g.sixel)
    exchange(sixel_colors_query());
  tcsetattr(fd, TCSANOW, &saved);
  int w = 0, h = 0;
  if (cell_pixel_size(fd, w, h)) { // the tty's own pixel size wins over CSI 16 t
    g.cell_w = w;
    g.cell_h = h;
  }
  close(fd);
#else
  (void)timeout_ms;
#endif
  return g;
}

// ---- kitty graphics protocol ----------------------------------------------------------------

std::uint32_t kitty_diacritic(int index) {
  return index >= 0 && index < kKittyMaxCells ? kDiacritics[index] : 0;
}

std::string kitty_placeholder(int row, int col) {
  if (row < 0 || row >= kKittyMaxCells || col < 0 || col >= kKittyMaxCells)
    return std::string();
  std::string s;
  put_utf8(s, 0x10EEEE);
  put_utf8(s, kDiacritics[row]);
  put_utf8(s, kDiacritics[col]);
  return s;
}

std::string base64_encode(const unsigned char *d, std::size_t n) {
  static const char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((n + 2) / 3 * 4);
  std::size_t i = 0;
  for (; i + 2 < n; i += 3) {
    const std::uint32_t v = (std::uint32_t(d[i]) << 16) | (std::uint32_t(d[i + 1]) << 8) | d[i + 2];
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += kAlphabet[(v >> 6) & 63];
    out += kAlphabet[v & 63];
  }
  if (i < n) {
    std::uint32_t v = std::uint32_t(d[i]) << 16;
    if (i + 1 < n)
      v |= std::uint32_t(d[i + 1]) << 8;
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += i + 1 < n ? kAlphabet[(v >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

std::string kitty_transmit(const unsigned char *rgb, int w, int h, std::size_t stride,
                           std::uint32_t id, int cols, int rows, std::size_t chunk) {
  if (w <= 0 || h <= 0 || !rgb)
    return std::string();
  const std::size_t row = static_cast<std::size_t>(w) * 3;
  if (stride == 0)
    stride = row;
  std::string payload;
  if (stride == row) {
    payload = base64_encode(rgb, row * static_cast<std::size_t>(h));
  } else {
    std::vector<unsigned char> packed(row * static_cast<std::size_t>(h));
    for (int y = 0; y < h; ++y)
      std::copy_n(rgb + static_cast<std::size_t>(y) * stride, row,
                  packed.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(y) * row));
    payload = base64_encode(packed.data(), packed.size());
  }
  chunk = std::max<std::size_t>(4, chunk / 4 * 4);
  std::string out;
  out.reserve(payload.size() + payload.size() / chunk * 16 + 96);
  const std::string keys = "a=T,U=1,f=24,s=" + std::to_string(w) + ",v=" + std::to_string(h) +
                           ",c=" + std::to_string(cols) + ",r=" + std::to_string(rows) +
                           ",i=" + std::to_string(id) + ",q=2,C=1";
  std::size_t pos = 0;
  do {
    const std::size_t len = std::min(chunk, payload.size() - pos);
    const bool more = pos + len < payload.size();
    out += "\x1b_G";
    out += pos == 0 ? keys + "," : std::string("q=2,");
    out += more ? "m=1;" : "m=0;";
    out.append(payload, pos, len);
    out += "\x1b\\";
    pos += len;
  } while (pos < payload.size());
  return out;
}

std::string kitty_delete(std::uint32_t id) {
  return "\x1b_Ga=d,d=I,i=" + std::to_string(id) + ",q=2\x1b\\";
}

// ---- iTerm2 inline images ----

bool iterm_terminal(const std::string &term_program) {
  // Not "vscode": its terminal's image support is off unless the user turns it on.
  return term_program == "WezTerm" || term_program == "iTerm.app" || term_program == "mintty";
}

std::string iterm_image(const unsigned char *rgb, int w, int h, std::size_t stride, int cols,
                        int rows, int quality) {
  if (!rgb || w <= 0 || h <= 0 || cols <= 0 || rows <= 0)
    return std::string();
  if (stride == 0)
    stride = static_cast<std::size_t>(w) * 3;
  // stb's JPEG writer takes packed rows.
  std::vector<unsigned char> packed;
  const unsigned char *src = rgb;
  if (stride != static_cast<std::size_t>(w) * 3) {
    packed.resize(static_cast<std::size_t>(w) * h * 3);
    for (int y = 0; y < h; ++y)
      std::copy_n(rgb + static_cast<std::size_t>(y) * stride, static_cast<std::size_t>(w) * 3,
                  packed.data() + static_cast<std::size_t>(y) * w * 3);
    src = packed.data();
  }
  std::string jpeg;
  stbi_write_jpg_to_func(
      [](void *out, void *data, int n) {
        static_cast<std::string *>(out)->append(static_cast<const char *>(data),
                                                static_cast<std::size_t>(n));
      },
      &jpeg, w, h, 3, src, std::clamp(quality, 1, 100));
  if (jpeg.empty())
    return std::string();
  return "\x1b]1337;File=inline=1;size=" + std::to_string(jpeg.size()) +
         ";width=" + std::to_string(cols) + ";height=" + std::to_string(rows) +
         ";preserveAspectRatio=0;doNotMoveCursor=1:" + // the cursor stays: no scroll at the bottom
         base64_encode(reinterpret_cast<const unsigned char *>(jpeg.data()), jpeg.size()) + "\x07";
}

// ---- DEC Sixel --------------------------------------------------------------------------------

SixelPalette sixel_palette(int colors) {
  static const SixelPalette kCubes[] = {{6, 7, 6}, {6, 6, 6}, {5, 6, 5}, {4, 5, 4}, {4, 4, 4},
                                        {3, 4, 3}, {3, 3, 3}, {2, 4, 2}, {2, 3, 2}};
  for (const SixelPalette &p : kCubes)
    if (p.size() <= colors)
      return p;
  return SixelPalette{2, 2, 2};
}

namespace {

void sixel_run(std::string &out, char c, int count) {
  if (count <= 0)
    return;
  if (count > 3) {
    out += '!';
    out += std::to_string(count);
    out += c;
  } else {
    out.append(static_cast<std::size_t>(count), c);
  }
}

} // namespace

std::string sixel_encode(const unsigned char *rgb, int w, int h, std::size_t stride, int colors) {
  if (w <= 0 || h <= 0 || !rgb)
    return std::string();
  if (stride == 0)
    stride = static_cast<std::size_t>(w) * 3;
  const SixelPalette pal = sixel_palette(colors);
  const int lv[3] = {pal.r, pal.g, pal.b};
  const int ncolors = pal.size();

  // Quantize: each channel to its cube level with a 4x4 ordered dither (Bayer thresholds), so a
  // smooth gradient becomes a pattern of the two nearest levels instead of bands.
  static const int kBayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  std::vector<unsigned char> idx(static_cast<std::size_t>(w) * h);
  std::vector<unsigned char> used(static_cast<std::size_t>(ncolors), 0);
  for (int y = 0; y < h; ++y) {
    const unsigned char *src = rgb + static_cast<std::size_t>(y) * stride;
    for (int x = 0; x < w; ++x) {
      const int t = 2 * kBayer[y & 3][x & 3] + 1; // (threshold + 0.5) * 2, of 32
      int q[3];
      for (int c = 0; c < 3; ++c) {
        const int l = lv[c] - 1;
        q[c] = std::min(l, (src[3 * x + c] * l * 32 + t * 255) / (255 * 32));
      }
      const int k = (q[0] * lv[1] + q[1]) * lv[2] + q[2];
      idx[static_cast<std::size_t>(y) * w + x] = static_cast<unsigned char>(k);
      used[static_cast<std::size_t>(k)] = 1;
    }
  }

  std::string out = "\x1bP0;1;0q\"1;1;" + std::to_string(w) + ";" + std::to_string(h);
  out.reserve(out.size() + static_cast<std::size_t>(w) * ((h + 5) / 6) * 4);
  for (int k = 0; k < ncolors; ++k) {
    if (!used[static_cast<std::size_t>(k)])
      continue;
    const int q[3] = {k / (lv[1] * lv[2]), (k / lv[2]) % lv[1], k % lv[2]};
    out += '#';
    out += std::to_string(k);
    out += ";2";
    for (int c = 0; c < 3; ++c) {
      out += ';';
      out += std::to_string((q[c] * 100 + (lv[c] - 1) / 2) / (lv[c] - 1));
    }
  }

  // Bands of six rows. For each colour present in a band, its sixel bits per column (and the
  // columns it spans), then one run-length-coded line per colour, '$' between them.
  std::vector<unsigned char> bits(static_cast<std::size_t>(ncolors) * w, 0);
  std::vector<int> lo(static_cast<std::size_t>(ncolors)), hi(static_cast<std::size_t>(ncolors));
  std::vector<int> band_of(static_cast<std::size_t>(ncolors), -1); // last band a colour was in
  std::vector<int> band_colors;
  for (int y0 = 0; y0 < h; y0 += 6) {
    band_colors.clear();
    const int rows = std::min(6, h - y0);
    for (int i = 0; i < rows; ++i) {
      const unsigned char *row = &idx[static_cast<std::size_t>(y0 + i) * w];
      for (int x = 0; x < w; ++x) {
        const std::size_t k = row[x];
        if (band_of[k] != y0) {
          band_of[k] = y0;
          band_colors.push_back(static_cast<int>(k));
          lo[k] = hi[k] = x;
        }
        lo[k] = std::min(lo[k], x);
        hi[k] = std::max(hi[k], x);
        bits[k * w + x] = static_cast<unsigned char>(bits[k * w + x] | (1u << i));
      }
    }
    std::sort(band_colors.begin(), band_colors.end());
    for (std::size_t ci = 0; ci < band_colors.size(); ++ci) {
      const int k = band_colors[ci];
      if (ci > 0)
        out += '$';
      out += '#';
      out += std::to_string(k);
      unsigned char *b = &bits[static_cast<std::size_t>(k) * w];
      const int x0 = lo[static_cast<std::size_t>(k)], x1 = hi[static_cast<std::size_t>(k)];
      sixel_run(out, '?', x0);
      char run_c = 0;
      int run_n = 0;
      for (int x = x0; x <= x1; ++x) {
        const char c = static_cast<char>(63 + b[x]);
        b[x] = 0;
        if (c == run_c) {
          ++run_n;
        } else {
          sixel_run(out, run_c, run_n);
          run_c = c;
          run_n = 1;
        }
      }
      sixel_run(out, run_c, run_n);
    }
    if (y0 + 6 < h)
      out += '-';
  }
  out += "\x1b\\";
  return out;
}

std::vector<CellRect> cover_rects(const std::vector<unsigned char> &mask, int cols, int rows) {
  std::vector<CellRect> out;
  if (cols <= 0 || rows <= 0 || mask.size() < static_cast<std::size_t>(cols) * rows)
    return out;
  std::vector<unsigned char> left(mask.begin(),
                                  mask.begin() + static_cast<std::ptrdiff_t>(cols) * rows);
  auto at = [&](int x, int y) -> unsigned char & {
    return left[static_cast<std::size_t>(y) * cols + x];
  };
  for (int x = 0; x < cols; ++x)
    for (int y = 0; y < rows; ++y) {
      if (!at(x, y))
        continue;
      int y1 = y;
      while (y1 < rows && at(x, y1))
        ++y1;
      int x1 = x + 1;
      while (x1 < cols) {
        bool whole = true;
        for (int k = y; k < y1 && whole; ++k)
          whole = at(x1, k) != 0;
        if (!whole)
          break;
        ++x1;
      }
      for (int k = x; k < x1; ++k)
        for (int yy = y; yy < y1; ++yy)
          at(k, yy) = 0;
      out.push_back(CellRect{x, y, x1 - x, y1 - y});
    }
  return out;
}

} // namespace gfx
} // namespace ariadne
} // namespace cvc

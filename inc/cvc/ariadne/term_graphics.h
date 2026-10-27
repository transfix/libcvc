#ifndef CVC_ARIADNE_TERM_GRAPHICS_H
#define CVC_ARIADNE_TERM_GRAPHICS_H

// Terminal pixel graphics: the encoders and the capability probe behind FtxuiBackend's Kitty and
// Sixel image modes (roadmap §16.6).
//
// Three protocols put real pixels in a terminal:
//   - the kitty graphics protocol (kitty, Ghostty, WezTerm, Konsole). An image is sent once, out of
//     band, in APC escapes (ESC _ G ... ESC \), and shown through Unicode PLACEHOLDER cells:
//     U+10EEEE with combining diacritics that name the image row and column, in a foreground
//     colour that names the image. The placeholders are ordinary text, so whatever draws text over
//     them (a window, a menu) hides that part of the image, and a full repaint redraws it for free.
//   - DEC Sixel (xterm -ti vt340, foot, mlterm, WezTerm, Windows Terminal, ...). A bitmap is drawn
//     at the cursor inside a DCS escape. It is not text, so it is re-sent after every repaint,
//     clipped to the cells nothing covers.
//   - iTerm2 inline images (iTerm2, WezTerm, mintty, Konsole, xterm.js's image addon): a PNG or
//     JPEG in an OSC 1337 escape, drawn at the cursor and scaled to the cells it names. Re-sent
//     like Sixel. It is the one of the three Windows' ConPTY passes on (it drops kitty's APC and
//     Sixel's DCS), so it is how a Windows console program shows pixels in WezTerm.
//
// Everything here except probe_terminal() and cell_pixel_size() is a pure function of its
// arguments, so it is testable without a terminal.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// What a terminal can draw beyond text, as probe_terminal() found it (or as a host sets it).
struct TermGraphics {
  bool kitty = false;         // answered a kitty graphics query
  bool sixel = false;         // its primary device attributes (DA1) list Sixel (4)
  bool iterm = false;         // $TERM_PROGRAM names a terminal that shows iTerm2 inline images
  int cell_w = 0, cell_h = 0; // one cell's size in pixels; 0 = unknown
  int sixel_colors = 0;       // Sixel colour registers (XTSMGRAPHICS); 0 = unknown
  bool answered = false;      // the terminal answered at all (its DA1 reply arrived)
};

namespace gfx {

// ---- probing ----

// The query probe_terminal() writes: a kitty graphics query for a 1x1 image (id `kitty_id`), the
// cell size in pixels (CSI 16 t), then DA1. Terminals answer in order, and every terminal answers
// DA1, so its reply marks the end.
std::string probe_query(std::uint32_t kitty_id);
// The follow-up query for a Sixel terminal: its colour register count (XTSMGRAPHICS), then DA1.
std::string sixel_colors_query();

// Parse the bytes a terminal sent back to probe_query() / sixel_colors_query(), merging what they
// say into `into`. Returns true once the DA1 reply has arrived (nothing more is coming). Bytes it
// does not recognise (keys typed meanwhile) are skipped.
bool parse_probe_reply(const std::string &reply, std::uint32_t kitty_id, TermGraphics &into);

// Ask the controlling terminal what it can draw. The queries are POSIX only: it opens /dev/tty,
// puts it in raw mode for the exchange and restores it, so call it BEFORE anything else owns the
// terminal (FtxuiBackend::run calls it before FTXUI starts). `timeout_ms` bounds the wait for a
// terminal that never answers. `iterm` comes from $TERM_PROGRAM where stdout is a terminal, and
// never on Windows (ConPTY misplaces a full-screen program's images). All-false where there is no
// terminal (and, but for `iterm`, in the browser).
TermGraphics probe_terminal(int timeout_ms = 300);

// One cell's size in pixels from the tty's window size (TIOCGWINSZ ws_xpixel / ws_col). False
// where the terminal does not report pixels (or on Windows / wasm).
bool cell_pixel_size(int fd, int &w, int &h);

// ---- kitty graphics protocol ----

// How many rows / columns one placement can address: the number of row/column diacritics.
constexpr int kKittyMaxCells = 297;

// The index-th row/column diacritic (0-based code point), or 0 out of range.
std::uint32_t kitty_diacritic(int index);

// The placeholder grapheme for cell (row, col) of a placement: U+10EEEE followed by the row and
// the column diacritics, UTF-8. "" when row or col is out of range.
std::string kitty_placeholder(int row, int col);

// Standard base64 (RFC 4648, padded).
std::string base64_encode(const unsigned char *data, std::size_t n);

// Transmit w x h RGB8 pixels (row-major, top-left origin; `stride` bytes per row, 0 = w * 3) as
// image `id` and create its virtual placement of cols x rows cells, for Unicode placeholders to
// show (a=T, U=1, f=24, q=2: the terminal sends no reply). The base64 payload is split into chunks
// of at most `chunk` bytes (rounded down to a multiple of 4), one APC escape each. Re-sending an
// id replaces that image.
std::string kitty_transmit(const unsigned char *rgb, int w, int h, std::size_t stride,
                           std::uint32_t id, int cols, int rows, std::size_t chunk = 4096);

// Delete image `id` and free its data (a=d, d=I).
std::string kitty_delete(std::uint32_t id);

// ---- iTerm2 inline images ----

// $TERM_PROGRAM's value for a terminal that shows iTerm2 inline images (WezTerm, iTerm.app,
// mintty, ...): what probe_terminal() sets TermGraphics::iterm from.
bool iterm_terminal(const std::string &term_program);

// w x h RGB8 pixels (`stride` bytes per row, 0 = w * 3) as one inline image, drawn at the cursor
// and scaled to cols x rows cells, the cursor left where it was (doNotMoveCursor, so an image at
// the bottom does not scroll the screen): OSC 1337 File=inline with a JPEG of `quality` (1-100).
std::string iterm_image(const unsigned char *rgb, int w, int h, std::size_t stride, int cols,
                        int rows, int quality = 85);

// ---- DEC Sixel ----

// The fixed palette sixel_encode quantizes to: an r x g x b colour cube with the most levels that
// fit in `colors` registers (6 x 7 x 6 = 252 for 256 or more, down to 2 x 2 x 2).
struct SixelPalette {
  int r = 2, g = 2, b = 2;
  int size() const { return r * g * b; }
};
SixelPalette sixel_palette(int colors);

// Encode w x h RGB8 pixels (`stride` bytes per row, 0 = w * 3) as one Sixel image: DCS with a
// 1:1 pixel aspect and raster attributes, the palette registers the image uses, then six-pixel
// bands, run-length coded. Colours are quantized to sixel_palette(colors) with a 4x4 ordered
// dither. Every pixel is painted, so the image is opaque.
std::string sixel_encode(const unsigned char *rgb, int w, int h, std::size_t stride,
                         int colors = 256);

// A cell rectangle.
struct CellRect {
  int x = 0, y = 0, w = 0, h = 0;
};

// Cover the set cells of a cols x rows mask (row-major) with disjoint rectangles, greedily and
// column first: the first set cell in column-major order starts a rectangle as tall as its run down
// the column and as wide as the columns to its right keep that whole run set. A Sixel image is a
// rectangle, so a partly covered image is sent as one image per rectangle of cells still showing.
// Column first, because a bitmap's height is whole six-pixel bands: a rectangle edge inside the
// showing cells is then usually a vertical one, which costs nothing.
std::vector<CellRect> cover_rects(const std::vector<unsigned char> &mask, int cols, int rows);

} // namespace gfx
} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_TERM_GRAPHICS_H

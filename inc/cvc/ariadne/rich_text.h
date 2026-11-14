#ifndef CVC_ARIADNE_RICH_TEXT_H
#define CVC_ARIADNE_RICH_TEXT_H
// Rich text (docs/ARIADNE.md §5.20): a `text:` with `runs:`, styled runs and links in one
// paragraph. The core builds the runs (Backend::rich_text gets them); this lays them out in lines,
// the same way in every backend, so a paragraph wraps where a terminal and a GL window both would:
//
//   - A line breaks at a space between two words, never inside a word: "bold" in a bold run glued
//     to "ly" in a plain one is one word. The space a line breaks at is dropped. A word wider than
//     the whole line is cut between two characters.
//   - A '\n' in a run ends its line. The spaces at the start of a line after a '\n' stay (an
//     indent); after a wrap they don't.
//   - A picture run (emoji:) is one unbreakable piece, as wide as image_width says.
//   - With max_lines > 0, the lines past it are dropped and the last one kept ends in `ellipsis`,
//     its own text cut back until both fit.
//
// A backend measures in its own unit (px, or terminal cells) through `measure`; width <= 0 does not
// wrap at all.
#include <cstddef>
#include <cvc/ariadne/backend.h> // TextRun
#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// A piece of one line: some of run `run`'s text (or its picture), `x` from the line's start and
// `w` wide. Consecutive text of one run on a line is one piece. `run` is kEllipsisRun for the
// ellipsis a cut paragraph ends in, which is plain text of no run.
struct RichPiece {
  static constexpr std::size_t kEllipsisRun = static_cast<std::size_t>(-1);
  std::size_t run = 0;
  std::string text; // the run's text in this piece (a picture's: its text)
  bool image = false;
  float x = 0.0f, w = 0.0f;
};
struct RichLine {
  std::vector<RichPiece> pieces;
  float width = 0.0f; // the line's extent: the end of its last piece
};
struct RichLayout {
  std::vector<RichLine> lines; // at least one (an empty paragraph is one empty line)
  bool cut = false;            // max_lines dropped some of it
  float width = 0.0f;          // the widest line
};

// `measure(text, style)`: how wide `text` is drawn in a run of that style (text_style bits).
// `image_width(run)`: how wide a picture run is drawn. `ellipsis`: what a cut paragraph ends in.
using RichMeasure = std::function<float(const std::string &text, unsigned style)>;
using RichImageWidth = std::function<float(const TextRun &run)>;
RichLayout layout_rich_text(const std::vector<TextRun> &runs, float width, int max_lines,
                            const RichMeasure &measure, const RichImageWidth &image_width,
                            const std::string &ellipsis = "\xE2\x80\xA6");

// The text_style bits a run's `style:` names: words among bold, italic and code (and plain,
// which is none), split at spaces or commas, in any case. A word it doesn't know is skipped, so a
// document that names a style a later host draws still shows on this one.
unsigned parse_text_style(const std::string &style);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_RICH_TEXT_H

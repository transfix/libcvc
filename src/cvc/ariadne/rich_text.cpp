// Rich text layout (§5.20) — see the header.
#include <cctype>
#include <cvc/ariadne/rich_text.h>

namespace cvc {
namespace ariadne {

namespace {

// The byte after the UTF-8 sequence starting at `i` (a stray continuation byte is one byte).
std::size_t next_glyph(const std::string &s, std::size_t i) {
  if (i >= s.size())
    return s.size();
  ++i;
  while (i < s.size() && (static_cast<unsigned char>(s[i]) & 0xC0) == 0x80)
    ++i;
  return i;
}
// The start of the UTF-8 sequence that ends at `i` (exclusive).
std::size_t prev_glyph(const std::string &s, std::size_t i) {
  if (i == 0)
    return 0;
  --i;
  while (i > 0 && (static_cast<unsigned char>(s[i]) & 0xC0) == 0x80)
    --i;
  return i;
}

bool is_space(char c) { return c == ' ' || c == '\t'; }

// A word, a run of spaces or a line break, in one run.
struct Token {
  enum class Kind { Word, Space, Newline } kind = Kind::Word;
  std::size_t run = 0;
  std::string text;
  bool image = false;
  float w = 0.0f;
};

class Liner {
public:
  Liner(const std::vector<TextRun> &runs, float width, int max_lines, const RichMeasure &measure)
      : runs_(runs), width_(width), max_lines_(max_lines), measure_(measure) {
    out_.lines.emplace_back();
  }

  // Place `text` of run `run` at the end of the current line. False once max_lines is full and
  // this would start the line after it: the paragraph is cut there.
  bool place(std::size_t run, const std::string &text, float w, bool image) {
    if (max_lines_ > 0 && static_cast<int>(out_.lines.size()) > max_lines_) {
      out_.cut = true;
      return false;
    }
    RichLine &l = out_.lines.back();
    if (!image && !l.pieces.empty() && l.pieces.back().run == run && !l.pieces.back().image) {
      l.pieces.back().text += text;
      l.pieces.back().w += w;
    } else {
      RichPiece p;
      p.run = run;
      p.text = text;
      p.image = image;
      p.x = l.width;
      p.w = w;
      l.pieces.push_back(std::move(p));
    }
    l.width += w;
    return true;
  }
  void new_line(bool soft) {
    out_.lines.emplace_back();
    soft_ = soft;
  }
  bool line_empty() const { return out_.lines.back().pieces.empty(); }
  float x() const { return out_.lines.back().width; }
  bool soft() const { return soft_; }
  bool wraps() const { return width_ > 0.0f; }
  float width() const { return width_; }
  float measure(const std::string &t, std::size_t run) const {
    return measure_ ? measure_(t, run < runs_.size() ? runs_[run].style : 0u) : 0.0f;
  }

  RichLayout finish(const std::string &ellipsis) {
    // Lines past max_lines that hold nothing (a closing '\n') are not a cut.
    if (max_lines_ > 0)
      while (static_cast<int>(out_.lines.size()) > max_lines_ && out_.lines.back().pieces.empty())
        out_.lines.pop_back();
    if (max_lines_ > 0 && static_cast<int>(out_.lines.size()) > max_lines_) {
      out_.lines.resize(static_cast<std::size_t>(max_lines_));
      out_.cut = true;
    }
    if (out_.cut && !out_.lines.empty())
      end_in(out_.lines.back(), ellipsis);
    for (const RichLine &l : out_.lines)
      out_.width = l.width > out_.width ? l.width : out_.width;
    return std::move(out_);
  }

private:
  // Cut `l` back until `ellipsis` fits after it, its trailing spaces too, then add it.
  void end_in(RichLine &l, const std::string &ellipsis) {
    const float ew = measure(ellipsis, RichPiece::kEllipsisRun);
    const auto trailing_space = [](const RichPiece &p) {
      return !p.image && !p.text.empty() && is_space(p.text.back());
    };
    while (!l.pieces.empty()) {
      RichPiece &p = l.pieces.back();
      const bool over = wraps() && l.width + ew > width_;
      if (!over && !trailing_space(p))
        break;
      if (p.image || p.text.empty()) {
        l.width -= p.w;
        l.pieces.pop_back();
        continue;
      }
      p.text.erase(prev_glyph(p.text, p.text.size()));
      const float w = p.text.empty() ? 0.0f : measure(p.text, p.run);
      l.width = p.x + w;
      p.w = w;
      if (p.text.empty())
        l.pieces.pop_back();
    }
    RichPiece e;
    e.run = RichPiece::kEllipsisRun;
    e.text = ellipsis;
    e.x = l.width;
    e.w = ew;
    l.width += ew;
    l.pieces.push_back(std::move(e));
  }

  const std::vector<TextRun> &runs_;
  float width_;
  int max_lines_;
  const RichMeasure &measure_;
  bool soft_ = false; // the current line began at a wrap
  RichLayout out_;
};

} // namespace

RichLayout layout_rich_text(const std::vector<TextRun> &runs, float width, int max_lines,
                            const RichMeasure &measure, const RichImageWidth &image_width,
                            const std::string &ellipsis) {
  // The runs as words, spaces and line breaks.
  std::vector<Token> toks;
  for (std::size_t r = 0; r < runs.size(); ++r) {
    const TextRun &run = runs[r];
    if (!run.image.empty()) {
      Token t;
      t.run = r;
      t.text = run.text;
      t.image = true;
      t.w = image_width ? image_width(run) : 0.0f;
      toks.push_back(std::move(t));
      continue;
    }
    const std::string &s = run.text;
    for (std::size_t i = 0; i < s.size();) {
      Token t;
      t.run = r;
      std::size_t j = i;
      if (s[i] == '\n') {
        t.kind = Token::Kind::Newline;
        j = i + 1;
      } else if (s[i] == '\r') { // a CRLF's CR: nothing
        i = i + 1;
        continue;
      } else if (is_space(s[i])) {
        t.kind = Token::Kind::Space;
        while (j < s.size() && is_space(s[j]))
          ++j;
      } else {
        while (j < s.size() && !is_space(s[j]) && s[j] != '\n' && s[j] != '\r')
          ++j;
      }
      t.text = s.substr(i, j - i);
      if (t.kind != Token::Kind::Newline)
        t.w = measure ? measure(t.text, run.style) : 0.0f;
      toks.push_back(std::move(t));
      i = j;
    }
  }

  Liner l(runs, width, max_lines, measure);
  std::vector<const Token *> spaces; // the spaces since the last word, placed with the next one
  bool ok = true;
  for (std::size_t i = 0; ok && i < toks.size();) {
    const Token &t = toks[i];
    if (t.kind == Token::Kind::Newline) {
      spaces.clear(); // a line's trailing spaces are dropped
      l.new_line(/*soft=*/false);
      ++i;
      continue;
    }
    if (t.kind == Token::Kind::Space) {
      if (!(l.line_empty() && l.soft())) // a wrapped line starts at its word
        spaces.push_back(&t);
      ++i;
      continue;
    }
    // A word: the word pieces up to the next space or line break, whatever their runs.
    std::size_t j = i;
    float word_w = 0.0f;
    while (j < toks.size() && toks[j].kind == Token::Kind::Word)
      word_w += toks[j++].w;
    float space_w = 0.0f;
    for (const Token *s : spaces)
      space_w += s->w;
    if (l.wraps() && !l.line_empty() && l.x() + space_w + word_w > l.width()) {
      spaces.clear();
      l.new_line(/*soft=*/true);
    }
    for (const Token *s : spaces)
      if (!(ok = l.place(s->run, s->text, s->w, false)))
        break;
    spaces.clear();
    const bool split = l.wraps() && l.x() + word_w > l.width();
    for (std::size_t k = i; ok && k < j; ++k) {
      const Token &p = toks[k];
      if (!split || p.image) {
        if (split && !l.line_empty() && l.x() + p.w > l.width())
          l.new_line(/*soft=*/true);
        ok = l.place(p.run, p.text, p.w, p.image);
        continue;
      }
      // Wider than a whole line: cut between characters.
      for (std::size_t a = 0; ok && a < p.text.size();) {
        const std::size_t b = next_glyph(p.text, a);
        const std::string g = p.text.substr(a, b - a);
        const float gw = l.measure(g, p.run);
        if (!l.line_empty() && l.x() + gw > l.width())
          l.new_line(/*soft=*/true);
        ok = l.place(p.run, g, gw, false);
        a = b;
      }
    }
    i = j;
  }
  return l.finish(ellipsis);
}

unsigned parse_text_style(const std::string &style) {
  unsigned bits = 0;
  std::string word;
  const auto take = [&] {
    if (word == "bold")
      bits |= text_style::bold;
    else if (word == "italic")
      bits |= text_style::italic;
    else if (word == "code")
      bits |= text_style::code;
    word.clear();
  };
  for (char c : style) {
    if (c == ' ' || c == ',' || c == '\t') {
      take();
      continue;
    }
    word += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  take();
  return bits;
}

} // namespace ariadne
} // namespace cvc

// Conformance tests for the state_exec grammar (docs/STATE_EXEC_GRAMMAR.md).
//
// Every case names the EBNF productions it exercises and the phase from which it holds:
//   0  holds for the pre-grammar (hand-written) parser too;
//   N  holds once phase PN of the grammar work lands (P2 = parser built from the grammar,
//      P3 = Unicode source rules + strict escapes, P4 = octal escapes, C escapes, raw strings).
// Cases for landed phases (<= kLandedPhase) must hold. Cases for pending phases must still
// DIFFER, so the table stays an exact list of what each phase changes; when a phase lands, bump
// kLandedPhase in the same PR. The doc's EBNF is checked for closure, and every production must
// have a case.
//
// Inputs and expectations are byte-exact: non-ASCII is written as \xNN escapes (MSVC builds
// without /utf-8), split with "" where a hex letter follows an escape.

#include "state_exec_grammar_dump.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace dump = cvc::state_exec::grammar_dump;

namespace {

constexpr int kLandedPhase = 0;

enum class mode { all, one }; // parse_all() or parse()

struct grammar_case {
  int phase;
  std::string rules; // space-separated EBNF production names
  mode how;
  std::string input;
  std::string expect; // grammar_dump output; "!error" matches any error position
};

std::string nest(int depth) { return std::string(depth, '(') + std::string(depth, ')'); }

const std::vector<grammar_case> &cases() {
  static const std::vector<grammar_case> kCases = {
      // --- holds today ----------------------------------------------------------------------
      {0, "program", mode::all, "", ""},
      {0, "program atmosphere", mode::all, "1 2\n3", "1\n2\n3"},
      {0, "single_datum", mode::one, "", "nil"},
      {0, "single_datum line_comment", mode::one, "  ; note\n 42 ", "42"},
      {0, "datum list", mode::all, "(a (b c) ())", "(a (b c) ())"},
      {0, "list", mode::all, nest(256), nest(256)},
      {0, "list", mode::all, "(a", "!error"},
      {0, "quoted", mode::all, "'x ' (1 2) ''a", "(quote x)\n(quote (1 2))\n(quote (quote a))"},
      {0, "line_comment line_end", mode::all, "; c\n(a ; c2\n b)", "(a b)"},
      {0, "whitespace", mode::all, "\t\n\v\f\r a", "a"},
      {0, "text_literal text_string string_char", mode::all, "\"hi there\"", "\"hi there\""},
      {0, "string_char", mode::all, "\"a\nb\"", R"("a\nb")"},
      {0, "text_string", mode::all, "\"abc", "!error"},
      {0, "text_escape simple_escape", mode::all, R"("\n\t\r\\\"")", R"("\n\t\r\\\"")"},
      {0, "hex_escape hex", mode::all, R"("\x41\xe9")", "\"A\xC3\xA9\""},
      {0, "hex_escape", mode::all, R"("\xZ1")", "!error"},
      {0, "unicode_escape", mode::all, R"("é\u{1F600}")", "\"\xC3\xA9\xF0\x9F\x98\x80\""},
      {0, "unicode_escape", mode::all, R"("\u12")", "!error"},
      {0, "unicode_escape", mode::all, R"("\u{}")", "!error"},
      {0, "unicode_escape", mode::all, R"("\u{1234567}")", "!error"},
      {0, "octal_escape", mode::all, R"("\0")", R"("\x00")"},
      {0, "bytes_literal byte_string byte_escape", mode::all, R"(b"ab\x00\xff\n")",
       R"(b"ab\x00\xff\x0a")"},
      {0, "byte_string", mode::all, "b\"\xC3\xA9\"", R"(b"\xc3\xa9")"},
      {0, "byte_string symbol", mode::all, "b (b) b\"\"", "b\n(b)\nb\"\""},
      {0, "boolean", mode::all, "#t true", "#t\n#t"},
      {0, "nil", mode::all, "#f false nil", "nil\nnil\nnil"},
      {0, "number integer dec_digits sign", mode::all, "42 -7 007 -9223372036854775808",
       "42\n-7\n7\n-9223372036854775808"},
      {0, "decimal_float exponent", mode::all, "1.5 .5 1. 1e3 -2.5E-3",
       "1.5\n0.5\n1.0\n1000.0\n-0.0025"},
      {0, "atom token constituent symbol", mode::all,
       "foo-bar? -> + - caf\xC3\xA9 \xCE\xBB \xE6\x97\xA5\xE6\x9C\xAC a.b:c",
       "foo-bar?\n->\n+\n-\ncaf\xC3\xA9\n\xCE\xBB\n\xE6\x97\xA5\xE6\x9C\xAC\na.b:c"},
      {0, "symbol", mode::all, "1+ ... #foo", "|1+|\n|...|\n|#foo|"},
      {0, "delimiter constituent", mode::all, "a\"b\"(c)'d;e", "a\n\"b\"\n(c)\n(quote d)"},
      {0, "raw_text symbol", mode::all, "r#foo", "r#foo"},

      // --- P2: parser built from the grammar, numbers included -----------------------------
      {2, "program", mode::all, "a)", "!error"},
      {2, "reserved", mode::all, "[a]", "!error"},
      {2, "reserved", mode::all, "a,b", "!error"},
      {2, "reserved", mode::all, "`a", "!error"},
      {2, "reserved", mode::all, "{}", "!error"},
      {2, "reserved", mode::all, "a|b", "!error"},
      {2, "reserved", mode::all, "a\\b", "!error"},
      {2, "single_datum", mode::one, "1 2", "!error"},
      {2, "quoted", mode::all, "'", "!error"},
      {2, "list", mode::all, nest(257), "!error"},
      {2, "line_end line_comment", mode::all, "; c\r1", "1"},
      {2, "radix_integer hex_digits", mode::all, "0x1F 0X1f #x1F #X1f 0xFF_FF",
       "31\n31\n31\n31\n65535"},
      {2, "radix_integer oct_digits bin_digits sign", mode::all, "0o17 #o17 0b101 #b101 -0x10 +0b1",
       "15\n15\n5\n5\n-16\n1"},
      {2, "radix_prefixed", mode::all, "0xZZ", "!error"},
      {2, "radix_prefixed", mode::all, "#bar", "!error"},
      {2, "radix_prefixed", mode::all, "0x", "!error"},
      {2, "integer sign", mode::all, "+5", "5"},
      {2, "dec_digits", mode::all, "1_000", "1000"},
      {2, "integer", mode::all, "9223372036854775808", "!error"},
      {2, "float_special", mode::all, "+inf.0 -inf.0 +nan.0 -nan.0",
       "+inf.0\n-inf.0\n+nan.0\n+nan.0"},
      {2, "symbol", mode::all, "inf nan infinity -INF", "inf\nnan\ninfinity\n-INF"},
      {2, "decimal_float", mode::all, "1e999", "!error"},
      {2, "decimal_float dec_digits", mode::all, "6.022_140e23", "6.02214e+23"},

      // --- P3: Unicode source rules, strict escapes ----------------------------------------
      {3, "scalar_value", mode::all,
       "a\xff"
       "b",
       "!error"},
      {3, "bom", mode::all, "\xEF\xBB\xBF(a)", "(a)"},
      {3, "other_space constituent", mode::all,
       "a\xC2\xA0"
       "b",
       "!error"},
      {3, "whitespace", mode::all,
       "a\xC2\x85"
       "b a\xE2\x80\xA8"
       "b",
       "a\nb\na\nb"},
      {3, "source_char bidi_control", mode::all,
       "\"a\xE2\x80\xAE"
       "b\"",
       "!error"},
      {3, "bidi_control line_comment", mode::all, "; \xE2\x81\xA6\n1", "!error"},
      {3, "control", mode::all,
       "\"a\x01"
       "b\"",
       "!error"},
      {3, "text_escape", mode::all, R"("\q")", "!error"},
      {3, "unicode_escape", mode::all, R"("\uD800")", "!error"},
      {3, "unicode_escape", mode::all, R"("\u{110000}")", "!error"},
      // The error is reported at the offending '"', counted in code points: 1:6, not 1:8.
      {3, "hex_escape", mode::all, "\"\xC3\xA9\\x4\"", "!error 1:6"},

      // --- P4: octal escapes, C escapes, raw strings ---------------------------------------
      {4, "octal_escape", mode::all, R"("\101\012")", R"("A\n")"},
      {4, "byte_escape octal_escape", mode::all, R"(b"\377")", R"(b"\xff")"},
      {4, "byte_escape", mode::all, R"(b"\400")", "!error"},
      {4, "simple_escape", mode::all, R"("\a\b\f\v")", R"("\x07\x08\x0c\x0b")"},
      {4, "raw_text raw_body raw_char", mode::all, R"(r"a\nb")", R"("a\\nb")"},
      {4, "hash_fence", mode::all, R"(r#"say "hi""#)", R"("say \"hi\"")"},
      {4, "hash_fence", mode::all, R"(r##"a"#b"##)", R"("a\"#b")"},
      {4, "raw_bytes", mode::all, R"(br"\x00")", R"(b"\\x00")"},
      {4, "raw_body", mode::all, R"(r#"abc")", "!error"},
  };
  return kCases;
}

std::string run(const grammar_case &c) {
  return c.how == mode::all ? dump::dump_all(c.input) : dump::dump_one(c.input);
}

bool matches(const std::string &actual, const std::string &expect) {
  if (expect == "!error")
    return actual.rfind("!error", 0) == 0;
  return actual == expect;
}

std::string describe(const grammar_case &c) {
  return "[P" + std::to_string(c.phase) + " " + c.rules + "] input: " + c.input;
}

fs::path grammar_doc() {
  // src/cvc/tests/<this file> -> repo root -> docs/
  return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "docs" /
         "STATE_EXEC_GRAMMAR.md";
}

struct ebnf {
  std::map<std::string, int> defined;          // production -> times defined
  std::map<std::string, std::set<std::string>> // production -> names its body references
      uses;
};

// Read the ```ebnf block of the grammar doc: `name ::= body` lines, continuation lines, and
// /* comments */. Names inside quotes, character classes and #xN code points are not references.
ebnf read_ebnf(const fs::path &doc) {
  std::ifstream in(doc);
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  const auto open = text.find("```ebnf\n");
  ebnf g;
  if (open == std::string::npos)
    return g;
  const auto close = text.find("```", open + 8);
  std::string body = text.substr(open + 8, close - open - 8);

  for (auto c = body.find("/*"); c != std::string::npos; c = body.find("/*")) // strip comments
    body.erase(c, body.find("*/", c) + 2 - c);

  auto is_name_char = [](char ch) { return (ch >= 'a' && ch <= 'z') || ch == '_'; };
  std::istringstream lines(body);
  std::string line, current;
  while (std::getline(lines, line)) {
    std::size_t i = 0;
    const auto def = line.find("::=");
    if (def != std::string::npos) {
      std::size_t end = line.find_first_of(" \t");
      current = line.substr(0, end);
      ++g.defined[current];
      i = def + 3;
    }
    while (i < line.size()) {
      const char ch = line[i];
      if (ch == '\'' || ch == '"') {
        i = line.find(ch, i + 1) + 1;
      } else if (ch == '[') {
        i = line.find(']', i + 1) + 1;
      } else if (ch == '#' && i + 1 < line.size() && line[i + 1] == 'x') {
        i += 2;
        while (i < line.size() && std::isxdigit(static_cast<unsigned char>(line[i])))
          ++i;
      } else if (is_name_char(ch)) {
        std::size_t j = i;
        while (j < line.size() && is_name_char(line[j]))
          ++j;
        g.uses[current].insert(line.substr(i, j - i));
        i = j;
      } else {
        ++i;
      }
    }
  }
  return g;
}

} // namespace

TEST(StateExecGrammar, CasesForLandedPhasesHold) {
  for (const grammar_case &c : cases()) {
    if (c.phase > kLandedPhase)
      continue;
    EXPECT_PRED2(matches, run(c), c.expect) << describe(c);
  }
}

TEST(StateExecGrammar, CasesForPendingPhasesStillDiffer) {
  for (const grammar_case &c : cases()) {
    if (c.phase <= kLandedPhase)
      continue;
    EXPECT_FALSE(matches(run(c), c.expect))
        << describe(c) << "\nalready holds; set its phase to the one that made it hold";
  }
}

TEST(StateExecGrammar, DocGrammarIsClosed) {
  const ebnf g = read_ebnf(grammar_doc());
  ASSERT_GT(g.defined.size(), 40u) << "no ```ebnf block found in " << grammar_doc();
  std::set<std::string> referenced;
  for (const auto &[name, names] : g.uses)
    referenced.insert(names.begin(), names.end());
  for (const auto &[name, count] : g.defined) {
    EXPECT_EQ(count, 1) << name << " is defined more than once";
    if (name != "program" && name != "single_datum")
      EXPECT_TRUE(referenced.count(name)) << name << " is defined but never used";
  }
  for (const std::string &name : referenced)
    EXPECT_TRUE(g.defined.count(name)) << name << " is used but never defined";
}

TEST(StateExecGrammar, EveryProductionHasACase) {
  const ebnf g = read_ebnf(grammar_doc());
  std::set<std::string> covered;
  for (const grammar_case &c : cases()) {
    std::istringstream names(c.rules);
    for (std::string n; names >> n;) {
      EXPECT_TRUE(g.defined.count(n)) << "case names unknown production '" << n << "'";
      covered.insert(n);
    }
  }
  for (const auto &[name, count] : g.defined)
    EXPECT_TRUE(covered.count(name)) << name << " has no case";
}

// Parses every DSL script the repo carries (state_exec_corpus/scripts.txt, a frozen snapshot made
// by gen_state_exec_corpus.py) and compares each parse with state_exec_corpus/expected.txt. The
// grammar work (docs/STATE_EXEC_GRAMMAR.md) replaces the parser; this pins how every existing
// script reads, so a change to that is visible in the same PR as an expected.txt diff.
//
// Regenerate expected.txt after an intended change:  CVC_UPDATE_GOLDENS=1 state_exec_corpus_test

#include "state_exec_grammar_dump.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
namespace dump = cvc::state_exec::grammar_dump;

namespace {

const std::string kMark = "@@@ script ";

fs::path corpus_dir() { return fs::path(__FILE__).parent_path() / "state_exec_corpus"; }

std::string read_file(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  std::string s = ss.str();
  // The files are LF-only (.gitattributes); undo a CRLF checkout anyway.
  for (auto i = s.find("\r\n"); i != std::string::npos; i = s.find("\r\n", i))
    s.erase(i, 1);
  return s;
}

// Split "@@@ script <source>\n<body>\n" entries into (source, body) pairs. A body is everything up
// to the next entry's mark, minus the one '\n' that ends it.
std::vector<std::pair<std::string, std::string>> read_entries(const fs::path &p) {
  const std::string text = read_file(p);
  std::vector<std::pair<std::string, std::string>> out;
  // The mark only counts at the start of a line (the header comment mentions it).
  std::size_t pos = text.compare(0, kMark.size(), kMark) == 0 ? 0 : text.find("\n" + kMark);
  if (pos != std::string::npos && text[pos] == '\n')
    ++pos;
  while (pos != std::string::npos) {
    const std::size_t eol = text.find('\n', pos);
    std::string source = text.substr(pos + kMark.size(), eol - pos - kMark.size());
    std::size_t next = text.find("\n" + kMark, eol);
    std::string body =
        text.substr(eol + 1, (next == std::string::npos ? text.size() : next + 1) - (eol + 1));
    if (!body.empty() && body.back() == '\n')
      body.pop_back();
    out.emplace_back(std::move(source), std::move(body));
    pos = next == std::string::npos ? next : next + 1;
  }
  return out;
}

} // namespace

TEST(StateExecCorpus, EveryScriptParsesAsRecorded) {
  const auto scripts = read_entries(corpus_dir() / "scripts.txt");
  ASSERT_GT(scripts.size(), 100u) << "corpus missing: " << corpus_dir() / "scripts.txt";

  std::vector<std::string> actual;
  std::size_t errors = 0;
  for (const auto &[source, script] : scripts) {
    actual.push_back(dump::dump_all(script));
    errors += actual.back().rfind("!error", 0) == 0;
  }
  std::cout << scripts.size() << " scripts, " << errors << " parse errors\n";

  const fs::path expected_path = corpus_dir() / "expected.txt";
  if (const char *update = std::getenv("CVC_UPDATE_GOLDENS"); update && *update == '1') {
    std::ofstream out(expected_path, std::ios::binary);
    out << "# Expected parses of scripts.txt, in the grammar_dump format of\n"
           "# state_exec_grammar_dump.h. Written by state_exec_corpus_test with\n"
           "# CVC_UPDATE_GOLDENS=1; do not edit by hand.\n";
    for (std::size_t i = 0; i < scripts.size(); ++i)
      out << kMark << scripts[i].first << '\n' << actual[i] << '\n';
    GTEST_SKIP() << "wrote " << expected_path;
  }

  const auto expected = read_entries(expected_path);
  ASSERT_EQ(expected.size(), scripts.size())
      << expected_path << " is stale; regenerate it with CVC_UPDATE_GOLDENS=1";
  int shown = 0;
  for (std::size_t i = 0; i < scripts.size(); ++i) {
    ASSERT_EQ(expected[i].first, scripts[i].first)
        << expected_path << " is stale; regenerate it with CVC_UPDATE_GOLDENS=1";
    if (actual[i] != expected[i].second && ++shown <= 20)
      ADD_FAILURE() << scripts[i].first << "\n--- script\n"
                    << scripts[i].second << "\n--- expected\n"
                    << expected[i].second << "\n--- actual\n"
                    << actual[i];
  }
  EXPECT_EQ(shown, 0) << shown << " scripts parse differently from " << expected_path;
}

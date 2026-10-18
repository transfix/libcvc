// PEGTL dependency check (phase P1 of the state_exec grammar work).
//
// The state_exec parser does not use PEGTL yet; its rules arrive in P2, in the
// private src/cvc/state/state_exec/grammar.h. This test only proves the
// dependency on every lane: find_package(pegtl) found a 3.x PEGTL, its headers
// compile in libcvc's C++20 build, and a small grammar parses, collects
// actions and reports an error position.

#include <gtest/gtest.h>
#include <string>
#include <string_view>
#include <tao/pegtl.hpp>
#include <vector>

namespace pegtl = tao::pegtl;

namespace {

// "(" symbol* ")" with optional whitespace; a toy, not the state_exec grammar.
struct ws : pegtl::star<pegtl::space> {};
struct symbol : pegtl::plus<pegtl::alpha> {};
struct close : pegtl::one<')'> {};
struct list : pegtl::seq<pegtl::one<'('>, ws, pegtl::star<symbol, ws>, pegtl::must<close>> {};
struct document : pegtl::seq<ws, list, ws, pegtl::eof> {};

template <typename Rule> struct collect : pegtl::nothing<Rule> {};

template <> struct collect<symbol> {
  template <typename ActionInput>
  static void apply(const ActionInput &in, std::vector<std::string> &out) {
    out.push_back(in.string());
  }
};

} // namespace

TEST(StateExecPegtlTest, IsVersion3) {
  EXPECT_EQ(std::string_view(TAO_PEGTL_VERSION).substr(0, 2), "3.");
}

TEST(StateExecPegtlTest, ParsesAndRunsActions) {
  std::vector<std::string> symbols;
  pegtl::memory_input in(" (foo bar  baz) ", "test");
  ASSERT_TRUE((pegtl::parse<document, collect>(in, symbols)));
  EXPECT_EQ(symbols, (std::vector<std::string>{"foo", "bar", "baz"}));
}

TEST(StateExecPegtlTest, RejectsWithoutMust) {
  std::vector<std::string> symbols;
  pegtl::memory_input in("foo", "test");
  EXPECT_FALSE((pegtl::parse<document, collect>(in, symbols)));
}

TEST(StateExecPegtlTest, MustReportsPosition) {
  std::vector<std::string> symbols;
  pegtl::memory_input in("(foo 1)", "test");
  try {
    pegtl::parse<document, collect>(in, symbols);
    FAIL() << "expected a parse_error";
  } catch (const pegtl::parse_error &e) {
    // The message names the rule via pegtl::demangle, which GCC 16.1 builds
    // spell as a mangled typeid name, so only the position is checked.
    ASSERT_FALSE(e.positions().empty());
    EXPECT_EQ(e.positions().front().line, 1u);
    EXPECT_EQ(e.positions().front().column, 6u);
  }
}

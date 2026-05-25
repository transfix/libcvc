#include <cvc/core/state_exec/parser.h>
#include <gtest/gtest.h>

using namespace cvc::state_exec;

// -- Basic atoms --

TEST(StateExecParserTest, ParseInteger) {
  auto v = parse("42");
  EXPECT_EQ(std::get<int64_t>(v.v), 42);
}

TEST(StateExecParserTest, ParseNegativeInteger) {
  auto v = parse("-7");
  EXPECT_EQ(std::get<int64_t>(v.v), -7);
}

TEST(StateExecParserTest, ParseFloat) {
  auto v = parse("3.14");
  EXPECT_DOUBLE_EQ(std::get<double>(v.v), 3.14);
}

TEST(StateExecParserTest, ParseNegativeFloat) {
  auto v = parse("-0.5");
  EXPECT_DOUBLE_EQ(std::get<double>(v.v), -0.5);
}

TEST(StateExecParserTest, ParseString) {
  auto v = parse("\"hello world\"");
  EXPECT_EQ(std::get<std::string>(v.v), "hello world");
}

TEST(StateExecParserTest, ParseStringEscapes) {
  auto v = parse("\"line1\\nline2\\ttab\\\\backslash\\\"quote\"");
  EXPECT_EQ(std::get<std::string>(v.v), "line1\nline2\ttab\\backslash\"quote");
}

TEST(StateExecParserTest, ParseSymbol) {
  auto v = parse("foo-bar");
  EXPECT_EQ(std::get<symbol>(v.v).name, "foo-bar");
}

TEST(StateExecParserTest, ParseBoolTrue) {
  auto v = parse("#t");
  EXPECT_EQ(std::get<bool>(v.v), true);
}

TEST(StateExecParserTest, ParseBoolFalse) {
  auto v = parse("#f");
  EXPECT_TRUE(v.is_nil());
}

TEST(StateExecParserTest, ParseNil) {
  auto v = parse("nil");
  EXPECT_TRUE(v.is_nil());
}

TEST(StateExecParserTest, ParseTrue) {
  auto v = parse("true");
  EXPECT_EQ(std::get<bool>(v.v), true);
}

// -- Lists --

TEST(StateExecParserTest, ParseEmptyList) {
  auto v = parse("()");
  auto *list = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(list, nullptr);
  EXPECT_TRUE((*list)->empty());
}

TEST(StateExecParserTest, ParseSimpleList) {
  auto v = parse("(+ 1 2)");
  auto *list = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(list, nullptr);
  ASSERT_EQ((*list)->size(), 3u);
  EXPECT_EQ(std::get<symbol>((**list)[0].v).name, "+");
  EXPECT_EQ(std::get<int64_t>((**list)[1].v), 1);
  EXPECT_EQ(std::get<int64_t>((**list)[2].v), 2);
}

TEST(StateExecParserTest, ParseNestedList) {
  auto v = parse("(if (> x 0) \"pos\" \"neg\")");
  auto *list = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(list, nullptr);
  ASSERT_EQ((*list)->size(), 4u);
  EXPECT_EQ(std::get<symbol>((**list)[0].v).name, "if");

  // Second element is (> x 0)
  auto *cond = std::get_if<list_ptr>(&(**list)[1].v);
  ASSERT_NE(cond, nullptr);
  ASSERT_EQ((*cond)->size(), 3u);
  EXPECT_EQ(std::get<symbol>((**cond)[0].v).name, ">");
}

TEST(StateExecParserTest, ParseDeeplyNested) {
  auto v = parse("(a (b (c (d))))");
  auto *l1 = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(l1, nullptr);
  ASSERT_EQ((*l1)->size(), 2u);
  auto *l2 = std::get_if<list_ptr>(&(**l1)[1].v);
  ASSERT_NE(l2, nullptr);
  ASSERT_EQ((*l2)->size(), 2u);
  auto *l3 = std::get_if<list_ptr>(&(**l2)[1].v);
  ASSERT_NE(l3, nullptr);
  ASSERT_EQ((*l3)->size(), 2u);
  auto *l4 = std::get_if<list_ptr>(&(**l3)[1].v);
  ASSERT_NE(l4, nullptr);
  ASSERT_EQ((*l4)->size(), 1u);
}

// -- Quote --

TEST(StateExecParserTest, ParseQuote) {
  auto v = parse("'x");
  auto *list = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(list, nullptr);
  ASSERT_EQ((*list)->size(), 2u);
  EXPECT_EQ(std::get<symbol>((**list)[0].v).name, "quote");
  EXPECT_EQ(std::get<symbol>((**list)[1].v).name, "x");
}

TEST(StateExecParserTest, ParseQuotedList) {
  auto v = parse("'(1 2 3)");
  auto *outer = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(outer, nullptr);
  ASSERT_EQ((*outer)->size(), 2u);
  EXPECT_EQ(std::get<symbol>((**outer)[0].v).name, "quote");

  auto *inner = std::get_if<list_ptr>(&(**outer)[1].v);
  ASSERT_NE(inner, nullptr);
  ASSERT_EQ((*inner)->size(), 3u);
}

// -- Comments --

TEST(StateExecParserTest, ParseWithComments) {
  auto v = parse("; this is a comment\n42");
  EXPECT_EQ(std::get<int64_t>(v.v), 42);
}

TEST(StateExecParserTest, ParseInlineComment) {
  auto v = parse("(+ 1 ; add one\n2)");
  auto *list = std::get_if<list_ptr>(&v.v);
  ASSERT_NE(list, nullptr);
  ASSERT_EQ((*list)->size(), 3u);
}

// -- parse_all --

TEST(StateExecParserTest, ParseAllMultipleExprs) {
  auto results = parse_all("(defun f (x) x) (f 42)");
  ASSERT_EQ(results.size(), 2u);

  auto *defun = std::get_if<list_ptr>(&results[0].v);
  ASSERT_NE(defun, nullptr);
  EXPECT_EQ(std::get<symbol>((**defun)[0].v).name, "defun");

  auto *call = std::get_if<list_ptr>(&results[1].v);
  ASSERT_NE(call, nullptr);
  EXPECT_EQ(std::get<symbol>((**call)[0].v).name, "f");
}

TEST(StateExecParserTest, ParseAllEmpty) {
  auto results = parse_all("");
  EXPECT_TRUE(results.empty());
}

TEST(StateExecParserTest, ParseAllCommentOnly) {
  auto results = parse_all("; just a comment\n");
  EXPECT_TRUE(results.empty());
}

// -- Errors --

TEST(StateExecParserTest, UnclosedParen) { EXPECT_THROW(parse("(+ 1 2"), parse_error); }

TEST(StateExecParserTest, UnterminatedString) { EXPECT_THROW(parse("\"hello"), parse_error); }

TEST(StateExecParserTest, ParseErrorLocation) {
  try {
    parse("(+ 1\n  (bad");
    FAIL() << "Expected parse_error";
  } catch (const parse_error &e) {
    // Should report line 2 for the unclosed inner paren
    EXPECT_EQ(e.line(), 2u);
  }
}

// -- Edge cases --

TEST(StateExecParserTest, WhitespaceHandling) {
  auto v = parse("  \n\t  42  \n  ");
  EXPECT_EQ(std::get<int64_t>(v.v), 42);
}

TEST(StateExecParserTest, OperatorSymbols) {
  auto syms = {"<=", ">=", "!=", "str-concat", "get-attr", "set-attr", "del-attr", "del-nth"};
  for (auto s : syms) {
    auto v = parse(s);
    EXPECT_EQ(std::get<symbol>(v.v).name, s) << "Failed for: " << s;
  }
}

TEST(StateExecParserTest, LargeInteger) {
  auto v = parse("9223372036854775807"); // INT64_MAX
  EXPECT_EQ(std::get<int64_t>(v.v), INT64_MAX);
}

TEST(StateExecParserTest, ScientificNotation) {
  auto v = parse("1.5e3");
  EXPECT_DOUBLE_EQ(std::get<double>(v.v), 1500.0);
}

TEST(StateExecParserTest, FullProgram) {
  auto exprs = parse_all(R"(
        ; Fibonacci function
        (defun fib (n)
          (if (<= n 1)
              n
              (+ (fib (- n 1))
                 (fib (- n 2)))))

        ; Test it
        (fib 10)
    )");
  ASSERT_EQ(exprs.size(), 2u);

  // defun form
  auto *defun = std::get_if<list_ptr>(&exprs[0].v);
  ASSERT_NE(defun, nullptr);
  EXPECT_EQ(std::get<symbol>((**defun)[0].v).name, "defun");
  EXPECT_EQ(std::get<symbol>((**defun)[1].v).name, "fib");

  // call form
  auto *call = std::get_if<list_ptr>(&exprs[1].v);
  ASSERT_NE(call, nullptr);
  EXPECT_EQ(std::get<symbol>((**call)[0].v).name, "fib");
  EXPECT_EQ(std::get<int64_t>((**call)[1].v), 10);
}

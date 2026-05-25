#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/types.h>
#include <gtest/gtest.h>

using namespace cvc::state_exec;

class BuiltinsTest : public ::testing::Test {
protected:
  environment_ptr env;
  void SetUp() override { env = builtins::make_default_environment(); }

  // call() wraps the brace-init-list → std::span conversion
  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *val = env->lookup(name);
    EXPECT_NE(val, nullptr) << "built-in '" << name << "' not found";
    auto &fn = std::get<native_fn>(val->v);
    return fn(args);
  }
};

// ─── Arithmetic ────────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, AddIntegers) {
  EXPECT_EQ(std::get<int64_t>(call("+", {int64_t{2}, int64_t{3}, int64_t{5}}).v), 10);
}

TEST_F(BuiltinsTest, AddMixed) {
  EXPECT_DOUBLE_EQ(std::get<double>(call("+", {int64_t{2}, 3.5}).v), 5.5);
}

TEST_F(BuiltinsTest, AddEmpty) { EXPECT_EQ(std::get<int64_t>(call("+", {}).v), 0); }

TEST_F(BuiltinsTest, AddFlattensList) {
  auto lst = make_list({value_t{int64_t{10}}, value_t{int64_t{20}}});
  EXPECT_EQ(std::get<int64_t>(call("+", {int64_t{5}, lst}).v), 35);
}

TEST_F(BuiltinsTest, SubUnary) { EXPECT_EQ(std::get<int64_t>(call("-", {int64_t{7}}).v), -7); }

TEST_F(BuiltinsTest, SubMultiple) {
  EXPECT_EQ(std::get<int64_t>(call("-", {int64_t{10}, int64_t{3}, int64_t{2}}).v), 5);
}

TEST_F(BuiltinsTest, MulIntegers) {
  EXPECT_EQ(std::get<int64_t>(call("*", {int64_t{3}, int64_t{4}}).v), 12);
}

TEST_F(BuiltinsTest, DivProducesDouble) {
  EXPECT_DOUBLE_EQ(std::get<double>(call("/", {int64_t{10}, int64_t{4}}).v), 2.5);
}

TEST_F(BuiltinsTest, DivByZeroThrows) {
  EXPECT_THROW(call("/", {int64_t{1}, int64_t{0}}), std::runtime_error);
}

TEST_F(BuiltinsTest, ModIntegers) {
  EXPECT_EQ(std::get<int64_t>(call("%", {int64_t{10}, int64_t{3}}).v), 1);
}

// ─── Comparison ────────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, LessThan) {
  EXPECT_TRUE(std::get<bool>(call("<", {int64_t{1}, int64_t{2}}).v));
  EXPECT_FALSE(std::get<bool>(call("<", {int64_t{2}, int64_t{1}}).v));
}

TEST_F(BuiltinsTest, GreaterThan) {
  EXPECT_TRUE(std::get<bool>(call(">", {int64_t{5}, int64_t{3}}).v));
}

TEST_F(BuiltinsTest, LessEqual) {
  EXPECT_TRUE(std::get<bool>(call("<=", {int64_t{3}, int64_t{3}}).v));
}

TEST_F(BuiltinsTest, Equality) {
  EXPECT_TRUE(std::get<bool>(call("=", {int64_t{42}, int64_t{42}}).v));
  EXPECT_FALSE(std::get<bool>(call("=", {int64_t{1}, int64_t{2}}).v));
}

TEST_F(BuiltinsTest, NotEqual) {
  EXPECT_TRUE(std::get<bool>(call("!=", {int64_t{1}, int64_t{2}}).v));
}

// ─── String ────────────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, StrConcat) {
  EXPECT_EQ(
      std::get<std::string>(
          call("str-concat", {std::string("hello"), std::string(" "), std::string("world")}).v),
      "hello world");
}

TEST_F(BuiltinsTest, Str) { EXPECT_EQ(std::get<std::string>(call("str", {int64_t{42}}).v), "42"); }

// ─── List ──────────────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, ListCreate) {
  auto r = call("list", {int64_t{1}, int64_t{2}, int64_t{3}});
  auto &lst = *std::get<list_ptr>(r.v);
  EXPECT_EQ(lst.size(), 3u);
  EXPECT_EQ(std::get<int64_t>(lst[0].v), 1);
}

TEST_F(BuiltinsTest, CarCdr) {
  auto lst = make_list({value_t{int64_t{10}}, value_t{int64_t{20}}, value_t{int64_t{30}}});
  EXPECT_EQ(std::get<int64_t>(call("car", {lst}).v), 10);
  auto tail = call("cdr", {lst});
  auto &tail_list = *std::get<list_ptr>(tail.v);
  ASSERT_EQ(tail_list.size(), 2u);
  EXPECT_EQ(std::get<int64_t>(tail_list[0].v), 20);
}

TEST_F(BuiltinsTest, Cons) {
  auto lst = make_list({value_t{int64_t{2}}, value_t{int64_t{3}}});
  auto r = call("cons", {int64_t{1}, lst});
  auto &result = *std::get<list_ptr>(r.v);
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(std::get<int64_t>(result[0].v), 1);
}

TEST_F(BuiltinsTest, Nth) {
  auto lst = make_list({value_t{int64_t{10}}, value_t{int64_t{20}}, value_t{int64_t{30}}});
  EXPECT_EQ(std::get<int64_t>(call("nth", {lst, int64_t{1}}).v), 20);
}

TEST_F(BuiltinsTest, SetNth) {
  auto lst = make_list({value_t{int64_t{10}}, value_t{int64_t{20}}});
  call("set-nth", {lst, int64_t{0}, int64_t{99}});
  EXPECT_EQ(std::get<int64_t>((*std::get<list_ptr>(lst.v))[0].v), 99);
}

TEST_F(BuiltinsTest, Length) {
  auto lst = make_list({value_t{int64_t{1}}, value_t{int64_t{2}}});
  EXPECT_EQ(std::get<int64_t>(call("length", {lst}).v), 2);
}

TEST_F(BuiltinsTest, LengthString) {
  EXPECT_EQ(std::get<int64_t>(call("length", {std::string("hello")}).v), 5);
}

TEST_F(BuiltinsTest, Append) {
  auto lst = make_list({value_t{int64_t{1}}});
  call("append", {lst, int64_t{2}});
  EXPECT_EQ(std::get<list_ptr>(lst.v)->size(), 2u);
}

TEST_F(BuiltinsTest, Slice) {
  auto lst = make_list(
      {value_t{int64_t{10}}, value_t{int64_t{20}}, value_t{int64_t{30}}, value_t{int64_t{40}}});
  auto r = call("slice", {lst, int64_t{1}, int64_t{3}});
  auto &sl = *std::get<list_ptr>(r.v);
  ASSERT_EQ(sl.size(), 2u);
  EXPECT_EQ(std::get<int64_t>(sl[0].v), 20);
  EXPECT_EQ(std::get<int64_t>(sl[1].v), 30);
}

TEST_F(BuiltinsTest, DelNth) {
  auto lst = make_list({value_t{int64_t{10}}, value_t{int64_t{20}}, value_t{int64_t{30}}});
  auto removed = call("del-nth", {lst, int64_t{1}});
  EXPECT_EQ(std::get<int64_t>(removed.v), 20);
  EXPECT_EQ(std::get<list_ptr>(lst.v)->size(), 2u);
}

// ─── Dict ──────────────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, DictCreate) {
  auto r = call("dict", {std::string("a"), int64_t{1}, std::string("b"), int64_t{2}});
  auto &d = *std::get<dict_ptr>(r.v);
  EXPECT_EQ(d.size(), 2u);
}

TEST_F(BuiltinsTest, GetSetAttr) {
  auto obj = call("dict", {std::string("x"), int64_t{10}});
  EXPECT_EQ(std::get<int64_t>(call("get-attr", {obj, std::string("x")}).v), 10);
  call("set-attr", {obj, std::string("x"), int64_t{99}});
  EXPECT_EQ(std::get<int64_t>(call("get-attr", {obj, std::string("x")}).v), 99);
}

TEST_F(BuiltinsTest, DelAttr) {
  auto obj = call("dict", {std::string("a"), int64_t{1}, std::string("b"), int64_t{2}});
  auto removed = call("del-attr", {obj, std::string("a")});
  EXPECT_EQ(std::get<int64_t>(removed.v), 1);
  EXPECT_EQ(std::get<dict_ptr>(obj.v)->size(), 1u);
}

// ─── Type predicates ───────────────────────────────────────────────────────

TEST_F(BuiltinsTest, IsNull) {
  EXPECT_TRUE(std::get<bool>(call("is-null", {nil_value}).v));
  EXPECT_FALSE(std::get<bool>(call("is-null", {int64_t{1}}).v));
}

TEST_F(BuiltinsTest, IsList) {
  auto lst = make_list({value_t{int64_t{1}}});
  EXPECT_TRUE(std::get<bool>(call("is-list", {lst}).v));
  EXPECT_FALSE(std::get<bool>(call("is-list", {int64_t{1}}).v));
}

TEST_F(BuiltinsTest, TypeOf) {
  EXPECT_EQ(std::get<std::string>(call("type-of", {int64_t{1}}).v), "int");
  EXPECT_EQ(std::get<std::string>(call("type-of", {nil_value}).v), "nil");
}

// ─── Logic ─────────────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, Not) {
  EXPECT_TRUE(std::get<bool>(call("not", {nil_value}).v));
  EXPECT_FALSE(std::get<bool>(call("not", {true_value}).v));
}

TEST_F(BuiltinsTest, And) {
  EXPECT_EQ(std::get<int64_t>(call("and", {true_value, int64_t{42}}).v), 42);
  EXPECT_FALSE(std::get<bool>(call("and", {true_value, false_value}).v));
}

TEST_F(BuiltinsTest, Or) {
  EXPECT_EQ(std::get<int64_t>(call("or", {false_value, int64_t{42}}).v), 42);
}

// ─── Higher-order ──────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, Apply) {
  auto *add_val = env->lookup("+");
  auto add_fn = std::get<native_fn>(add_val->v);
  auto args_list = make_list({value_t{int64_t{1}}, value_t{int64_t{2}}, value_t{int64_t{3}}});
  auto r = call("apply", {value_t{native_fn{add_fn}}, args_list});
  EXPECT_EQ(std::get<int64_t>(r.v), 6);
}

// ─── Environment ───────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, DefaultEnvironmentHasAllBuiltins) {
  std::vector<std::string> expected = {
      "+",        "-",        "*",        "/",          "%",      "<",       ">",       "<=",
      ">=",       "=",        "!=",       "str-concat", "str",    "list",    "car",     "cdr",
      "cons",     "nth",      "set-nth",  "length",     "append", "slice",   "del-nth", "dict",
      "get-attr", "set-attr", "del-attr", "send",       "apply",  "is-null", "is-list", "type-of",
      "print",    "not",      "and",      "or",
  };
  for (auto &name : expected) {
    EXPECT_NE(env->lookup(name), nullptr) << "missing: " << name;
  }
}

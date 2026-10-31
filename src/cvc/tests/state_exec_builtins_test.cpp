#include <cmath>
#include <cstdlib>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <limits>
#include <random>

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

TEST_F(BuiltinsTest, StrWritesAFloatInItsShortestRoundTripForm) {
  // A float prints as the shortest text that reads back as it, not six significant digits:
  // (str 1234567.5) was "1.23457e+06".
  const auto str = [&](value_t v) { return std::get<std::string>(call("str", {v}).v); };
  EXPECT_EQ(str(1234567.5), "1234567.5");
  EXPECT_EQ(str(0.1 + 0.2), "0.30000000000000004");
  EXPECT_EQ(str(0.1), "0.1");
  EXPECT_EQ(str(1e21), "1e+21");
  // A whole float prints as an integer's digits, as before and as state-set stores it.
  EXPECT_EQ(str(2.0), "2");
  EXPECT_EQ(str(-0.0), "-0");
  for (const double d : {1.0 / 3, 6.02214076e23, 5e-324, 123456789.123, -2.5e-8})
    EXPECT_EQ(std::strtod(str(d).c_str(), nullptr), d) << d;
  // A list's floats, and a float str-concat joins.
  EXPECT_EQ(str(make_list({value_t{1234567.5}, value_t{0.1 + 0.2}})),
            "(1234567.5 0.30000000000000004)");
  EXPECT_EQ(std::get<std::string>(call("str-concat", {std::string("SCRIP "), 1234567.5}).v),
            "SCRIP 1234567.5");
}

// ─── Type Conversion ───────────────────────────────────────────────────────

TEST_F(BuiltinsTest, IntFromString) {
  EXPECT_EQ(std::get<int64_t>(call("int", {std::string("123")}).v), 123);
}

TEST_F(BuiltinsTest, IntFromNegativeString) {
  EXPECT_EQ(std::get<int64_t>(call("int", {std::string("-42")}).v), -42);
}

TEST_F(BuiltinsTest, IntFromDouble) { EXPECT_EQ(std::get<int64_t>(call("int", {3.7}).v), 3); }

TEST_F(BuiltinsTest, IntFromBool) {
  EXPECT_EQ(std::get<int64_t>(call("int", {true}).v), 1);
  EXPECT_EQ(std::get<int64_t>(call("int", {false}).v), 0);
}

TEST_F(BuiltinsTest, IntFromInt) { EXPECT_EQ(std::get<int64_t>(call("int", {int64_t{99}}).v), 99); }

TEST_F(BuiltinsTest, IntBadStringThrows) {
  EXPECT_THROW(call("int", {std::string("abc")}), std::runtime_error);
}

TEST_F(BuiltinsTest, FloatFromString) {
  EXPECT_DOUBLE_EQ(std::get<double>(call("float", {std::string("3.14")}).v), 3.14);
}

TEST_F(BuiltinsTest, FloatFromInt) {
  EXPECT_DOUBLE_EQ(std::get<double>(call("float", {int64_t{5}}).v), 5.0);
}

TEST_F(BuiltinsTest, FloatFromBool) {
  EXPECT_DOUBLE_EQ(std::get<double>(call("float", {true}).v), 1.0);
}

TEST_F(BuiltinsTest, FloatFromDouble) {
  EXPECT_DOUBLE_EQ(std::get<double>(call("float", {2.5}).v), 2.5);
}

TEST_F(BuiltinsTest, FloatBadStringThrows) {
  EXPECT_THROW(call("float", {std::string("xyz")}), std::runtime_error);
}

TEST_F(BuiltinsTest, IsInt) {
  EXPECT_TRUE(std::get<bool>(call("is-int", {int64_t{5}}).v));
  EXPECT_FALSE(std::get<bool>(call("is-int", {3.14}).v));
  EXPECT_FALSE(std::get<bool>(call("is-int", {std::string("5")}).v));
}

TEST_F(BuiltinsTest, IsFloat) {
  EXPECT_TRUE(std::get<bool>(call("is-float", {2.5}).v));
  EXPECT_FALSE(std::get<bool>(call("is-float", {int64_t{5}}).v));
}

TEST_F(BuiltinsTest, IsString) {
  EXPECT_TRUE(std::get<bool>(call("is-string", {std::string("hi")}).v));
  EXPECT_FALSE(std::get<bool>(call("is-string", {int64_t{5}}).v));
}

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
      "+",          "-",           "*",       "/",          "%",          "<",        ">",
      "<=",         ">=",          "=",       "!=",         "str-concat", "str",      "list",
      "car",        "cdr",         "cons",    "nth",        "set-nth",    "length",   "append",
      "slice",      "del-nth",     "dict",    "get-attr",   "set-attr",   "del-attr", "send",
      "apply",      "is-null",     "is-list", "type-of",    "print",      "not",      "and",
      "or",         "generator",   "next",    "generator?", "gen-done?",  "range",    "collect",
      "json-parse", "json-encode",
  };
  for (auto &name : expected) {
    EXPECT_NE(env->lookup(name), nullptr) << "missing: " << name;
  }
}

// ─── Generator builtins ────────────────────────────────────────────────────

TEST_F(BuiltinsTest, RangeCreatesGenerator) {
  auto r = call("range", {int64_t{5}});
  EXPECT_TRUE(std::holds_alternative<generator_ptr>(r.v));
}

TEST_F(BuiltinsTest, RangeCollect) {
  auto gen = call("range", {int64_t{4}});
  auto result = call("collect", {gen});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 4u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 1);
  EXPECT_EQ(std::get<int64_t>((**lst)[2].v), 2);
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 3);
}

TEST_F(BuiltinsTest, RangeStartEnd) {
  auto gen = call("range", {int64_t{2}, int64_t{6}});
  auto result = call("collect", {gen});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 4u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 2);
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 5);
}

TEST_F(BuiltinsTest, RangeWithStep) {
  auto gen = call("range", {int64_t{0}, int64_t{10}, int64_t{3}});
  auto result = call("collect", {gen});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 4u); // 0 3 6 9
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 9);
}

TEST_F(BuiltinsTest, RangeNegativeStep) {
  auto gen = call("range", {int64_t{5}, int64_t{0}, int64_t{-1}});
  auto result = call("collect", {gen});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 5u); // 5 4 3 2 1
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 5);
  EXPECT_EQ(std::get<int64_t>((**lst)[4].v), 1);
}

TEST_F(BuiltinsTest, RangeZeroStepThrows) {
  EXPECT_THROW(call("range", {int64_t{0}, int64_t{5}, int64_t{0}}), std::runtime_error);
}

TEST_F(BuiltinsTest, NextOnExhaustedGeneratorReturnsNil) {
  auto gen = call("range", {int64_t{1}});
  auto v1 = call("next", {gen});
  EXPECT_EQ(std::get<int64_t>(v1.v), 0);
  auto v2 = call("next", {gen});
  EXPECT_TRUE(v2.is_nil());
  // Calling next again after exhaustion still returns nil
  auto v3 = call("next", {gen});
  EXPECT_TRUE(v3.is_nil());
}

TEST_F(BuiltinsTest, GeneratorPredicate) {
  auto gen = call("range", {int64_t{5}});
  auto r = call("generator?", {gen});
  EXPECT_TRUE(std::get<bool>(r.v));
  // Non-generator
  auto r2 = call("generator?", {int64_t{42}});
  EXPECT_FALSE(std::get<bool>(r2.v));
}

TEST_F(BuiltinsTest, GenDonePredicate) {
  auto gen = call("range", {int64_t{1}});
  EXPECT_FALSE(std::get<bool>(call("gen-done?", {gen}).v));
  call("next", {gen}); // yields 0
  call("next", {gen}); // exhausts
  EXPECT_TRUE(std::get<bool>(call("gen-done?", {gen}).v));
}

TEST_F(BuiltinsTest, TypeOfGenerator) {
  auto gen = call("range", {int64_t{3}});
  auto r = call("type-of", {gen});
  EXPECT_EQ(std::get<std::string>(r.v), "generator");
}

// ─── URL encoding ──────────────────────────────────────────────────────────

TEST_F(BuiltinsTest, UrlEncodeUnreservedPassesThrough) {
  EXPECT_EQ(std::get<std::string>(call("url-encode", {std::string("aZ0-_.~")}).v), "aZ0-_.~");
}

TEST_F(BuiltinsTest, UrlEncodeEscapesReservedAndSpace) {
  // space -> %20 (not '+'), and reserved delimiters are percent-escaped with uppercase hex
  EXPECT_EQ(std::get<std::string>(call("url-encode", {std::string("a b&c=d/e?f")}).v),
            "a%20b%26c%3Dd%2Fe%3Ff");
}

TEST_F(BuiltinsTest, UrlEncodeStringifiesNonString) {
  EXPECT_EQ(std::get<std::string>(call("url-encode", {int64_t{42}}).v), "42");
}

TEST_F(BuiltinsTest, UrlDecodeReversesPercentAndPlus) {
  EXPECT_EQ(std::get<std::string>(call("url-decode", {std::string("a%20b%26c")}).v), "a b&c");
  EXPECT_EQ(std::get<std::string>(call("url-decode", {std::string("a+b")}).v), "a b"); // form '+'
}

TEST_F(BuiltinsTest, UrlDecodeKeepsMalformedEscapeLiterally) {
  // a lone '%' or a truncated escape is preserved verbatim, never dropped
  EXPECT_EQ(std::get<std::string>(call("url-decode", {std::string("100%")}).v), "100%");
  EXPECT_EQ(std::get<std::string>(call("url-decode", {std::string("%zz")}).v), "%zz");
}

TEST_F(BuiltinsTest, UrlEncodeDecodeRoundTripsBytes) {
  // every byte value survives an encode -> decode round trip
  std::string all;
  for (int c = 1; c < 256; ++c)
    all.push_back(static_cast<char>(c));
  const value_t enc = call("url-encode", {all});
  const value_t dec = call("url-decode", {std::get<std::string>(enc.v)});
  EXPECT_EQ(std::get<std::string>(dec.v), all);
}

// ─── JSON ──────────────────────────────────────────────────────────────────

class JsonBuiltinsTest : public BuiltinsTest {
protected:
  value_t parse(const std::string &text) { return call("json-parse", {text}); }
  std::string text_of(const std::string &json) { return std::get<std::string>(parse(json).v); }
  std::string encode(const value_t &v) { return std::get<std::string>(call("json-encode", {v}).v); }
  std::string encode(const value_t &v, const value_t &indent) {
    return std::get<std::string>(call("json-encode", {v, indent}).v);
  }
  // The dsl_error `fn` throws: its class and message, with `line`/`col` fields as "L:C".
  template <class F> std::string error_of(F fn) {
    try {
      fn();
    } catch (const dsl_error &e) {
      std::string where;
      for (const auto &[k, v] : e.fields())
        if (k == "line")
          where = std::to_string(std::get<int64_t>(v.v)) + where;
        else if (k == "col")
          where += ":" + std::to_string(std::get<int64_t>(v.v));
      return e.class_name() + (where.empty() ? "" : " " + where) + " " + e.what();
    }
    return "no error";
  }
  std::string parse_error(const std::string &text) {
    return error_of([&] { parse(text); });
  }
};

const value_t &member(const value_t &dict, const std::string &key) {
  const auto &d = *std::get<dict_ptr>(dict.v);
  for (const auto &[k, v] : d)
    if (k == key)
      return v;
  ADD_FAILURE() << "no key " << key;
  return nil_value;
}

TEST_F(JsonBuiltinsTest, ParsesEveryKindOfValue) {
  const value_t v = parse(R"( {"s": "x", "i": -7, "f": 1.5, "t": true, "F": false, "n": null,
                 "a": [1, [2], {}], "o": {"k": []}} )");
  const auto &d = *std::get<dict_ptr>(v.v);
  std::vector<std::string> keys;
  for (const auto &[k, e] : d)
    keys.push_back(k);
  EXPECT_EQ(keys, (std::vector<std::string>{"s", "i", "f", "t", "F", "n", "a", "o"})); // in order
  EXPECT_EQ(std::get<std::string>(member(v, "s").v), "x");
  EXPECT_EQ(std::get<int64_t>(member(v, "i").v), -7);
  EXPECT_DOUBLE_EQ(std::get<double>(member(v, "f").v), 1.5);
  EXPECT_TRUE(std::get<bool>(member(v, "t").v));
  EXPECT_FALSE(std::get<bool>(member(v, "F").v)); // the bool false, not nil
  EXPECT_TRUE(member(v, "n").is_nil());
  EXPECT_TRUE(values_equal(
      member(v, "a"),
      make_list({value_t{int64_t{1}}, make_list({value_t{int64_t{2}}}), make_dict()})));
  EXPECT_TRUE(values_equal(member(v, "o"), make_dict({{"k", make_list()}})));
  EXPECT_EQ(std::get<std::string>(parse("\"top\"").v), "top"); // any value can be the document
  EXPECT_EQ(std::get<int64_t>(parse(" 42 ").v), 42);
  EXPECT_TRUE(parse("null").is_nil());
}

TEST_F(JsonBuiltinsTest, NumbersWithoutFractionOrExponentAreInts) {
  EXPECT_EQ(std::get<int64_t>(parse("0").v), 0);
  EXPECT_EQ(std::get<int64_t>(parse("-0").v), 0);
  EXPECT_DOUBLE_EQ(std::get<double>(parse("1.0").v), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(parse("1e2").v), 100.0);
  EXPECT_DOUBLE_EQ(std::get<double>(parse("-12.5E-1").v), -1.25);
  EXPECT_DOUBLE_EQ(std::get<double>(parse("2e+3").v), 2000.0);
}

TEST_F(JsonBuiltinsTest, IntegersPast64BitsAreFloats) {
  EXPECT_EQ(std::get<int64_t>(parse("9223372036854775807").v), std::numeric_limits<int64_t>::max());
  EXPECT_EQ(std::get<int64_t>(parse("-9223372036854775808").v),
            std::numeric_limits<int64_t>::min());
  EXPECT_DOUBLE_EQ(std::get<double>(parse("9223372036854775808").v), 9223372036854775808.0);
  EXPECT_DOUBLE_EQ(std::get<double>(parse("-9223372036854775809").v), -9223372036854775808.0);
  EXPECT_DOUBLE_EQ(std::get<double>(parse("123456789012345678901234567890").v),
                   1.2345678901234568e29);
  EXPECT_NE(
      parse_error("1e999").find("ValueError 1:1 json-parse: line 1, column 1: number out of range"),
      std::string::npos)
      << parse_error("1e999");
  EXPECT_DOUBLE_EQ(std::get<double>(parse("1e-999").v), 0.0); // underflow is 0
}

TEST_F(JsonBuiltinsTest, ARepeatedKeyKeepsItsPlaceAndTakesTheLastValue) {
  const value_t v = parse(R"({"a": 1, "b": 2, "a": 3})");
  EXPECT_TRUE(values_equal(v, make_dict({{"a", value_t{int64_t{3}}}, {"b", value_t{int64_t{2}}}})));
  // Past 16 keys the lookup is indexed; same rule.
  std::string text = "{";
  for (int i = 0; i < 40; ++i)
    text += "\"k" + std::to_string(i) + "\": " + std::to_string(i) + ", ";
  text += "\"k3\": \"again\", \"k39\": null}";
  const value_t big = parse(text);
  const auto &d = *std::get<dict_ptr>(big.v);
  ASSERT_EQ(d.size(), 40u);
  EXPECT_EQ(d[3].first, "k3");
  EXPECT_EQ(std::get<std::string>(d[3].second.v), "again");
  EXPECT_TRUE(d[39].second.is_nil());
  EXPECT_EQ(std::get<int64_t>(d[38].second.v), 38);
}

// The JSON texts are named constants: MSVC's preprocessor misreads a raw string with \" in it
// inside a macro's arguments (C2017).
TEST_F(JsonBuiltinsTest, StringEscapesAndUnicode) {
  const std::string simple = R"("\"\\\/\b\f\n\r\t")";
  const std::string unicode = R"("\u0041\u00e9\u20AC\ud83d\ude00")";
  const std::string nul = R"("\u0000")";
  const std::string key = R"({"\u00e9t\u00e9": 1})";
  EXPECT_EQ(text_of(simple), "\"\\/\b\f\n\r\t");
  EXPECT_EQ(text_of(unicode), "A\xC3\xA9\xE2\x82\xAC\xF0\x9F\x98\x80");
  // Raw UTF-8 passes through as it is.
  EXPECT_EQ(text_of("\"\xC3\xA9\xE2\x82\xAC\xF0\x9F\x98\x80\""),
            "\xC3\xA9\xE2\x82\xAC\xF0\x9F\x98\x80");
  EXPECT_EQ(text_of(nul), std::string(1, '\0'));
  // Keys are strings like any other.
  const value_t keyed = parse(key);
  EXPECT_TRUE(values_equal(keyed, make_dict({{"\xC3\xA9t\xC3\xA9", value_t{int64_t{1}}}})));
}

TEST_F(JsonBuiltinsTest, InvalidUtf8AndLoneSurrogatesBecomeTheReplacementCharacter) {
  const std::string fffd = "\xEF\xBF\xBD";
  EXPECT_EQ(std::get<std::string>(parse("\"a\xFF"
                                        "b\"")
                                      .v),
            "a" + fffd + "b");
  EXPECT_EQ(std::get<std::string>(parse("\"\xE2\x82\"").v), fffd + fffd); // truncated: a byte each
  const std::string high = R"("\ud83d")", high_then_x = R"("\ud83dx")", low = R"("\ude00")",
                    high_then_a = R"("\ud83d\u0041")", high_then_pair = R"("\ud83d\ud83d\ude00")";
  EXPECT_EQ(text_of(high), fffd); // a truncated emoji
  EXPECT_EQ(text_of(high_then_x), fffd + "x");
  EXPECT_EQ(text_of(low), fffd);
  EXPECT_EQ(text_of(high_then_a), fffd + "A");
  EXPECT_EQ(text_of(high_then_pair), fffd + "\xF0\x9F\x98\x80");
}

TEST_F(JsonBuiltinsTest, MalformedTextIsAValueErrorWithItsLineAndColumn) {
  EXPECT_EQ(parse_error("{\n  \"a\": tru\n}"),
            "ValueError 2:8 json-parse: line 2, column 8: expected a value");
  // Columns count code points.
  EXPECT_EQ(parse_error("[\"\xC3\xA9\xC3\xA9\", x]"),
            "ValueError 1:8 json-parse: line 1, column 8: expected a value");
  const std::pair<const char *, const char *> cases[] = {
      {"", "1:1 json-parse: line 1, column 1: expected a value"},
      {"   ", "expected a value"},
      {"[", "expected a value"},
      {"[1,]", "1:4 json-parse: line 1, column 4: expected a value"},
      {"[1 2]", "expected ',' or ']'"},
      {"{\"a\" 1}", "expected ':'"},
      {"{\"a\": 1,}", "expected a string key"},
      {"{1: 2}", "expected a string key or '}'"},
      {"{\"a\": 1", "expected ',' or '}'"},
      {"01", "unexpected text after the value"},
      {"[1]x", "unexpected text after the value"},
      {"1.", "invalid number"},
      {"-", "invalid number"},
      {"1e", "invalid number"},
      {".5", "expected a value"},
      {"+1", "expected a value"},
      {"NaN", "expected a value"},
      {"nul", "expected a value"},
      {"\"abc", "1:1 json-parse: line 1, column 1: unterminated string"},
      {"\"a\nb\"", "control character in a string"},
      {"\"\\x\"", "1:2 json-parse: line 1, column 2: invalid escape"},
      {"\"\\u12\"", "invalid \\u escape"},
      {"[\"\\", "unterminated string"},
  };
  for (const auto &[text, want] : cases) {
    const std::string got = parse_error(text);
    EXPECT_EQ(got.rfind("ValueError ", 0), 0u) << "'" << text << "': " << got;
    EXPECT_NE(got.find(want), std::string::npos) << "'" << text << "': " << got;
  }
}

TEST_F(JsonBuiltinsTest, NestingIsCapped) {
  const auto nested = [](std::size_t n) { return std::string(n, '[') + std::string(n, ']'); };
  value_t v = parse(nested(kJsonMaxDepth));
  for (std::size_t i = 1; i < kJsonMaxDepth; ++i)
    v = std::get<list_ptr>(v.v)->at(0);
  EXPECT_TRUE(std::get<list_ptr>(v.v)->empty());
  EXPECT_NE(parse_error(nested(kJsonMaxDepth + 1)).find("nested deeper than 512 levels"),
            std::string::npos);
  // A deep document is refused without recursion, however deep it claims to be.
  EXPECT_NE(parse_error(std::string(1000000, '[')).find("nested deeper than 512 levels"),
            std::string::npos);
  std::string objects;
  for (std::size_t i = 0; i <= kJsonMaxDepth; ++i)
    objects += "{\"a\":";
  EXPECT_NE(parse_error(objects).find("nested deeper"), std::string::npos);
}

TEST_F(JsonBuiltinsTest, ParseTakesBytesAndNothingElse) {
  EXPECT_TRUE(
      values_equal(call("json-parse", {make_bytes("[1]")}), make_list({value_t{int64_t{1}}})));
  EXPECT_EQ(error_of([&] { call("json-parse", {value_t{int64_t{1}}}); }).rfind("TypeError", 0), 0u);
  EXPECT_EQ(error_of([&] { call("json-parse", {}); }).rfind("TypeError", 0), 0u);
}

TEST_F(JsonBuiltinsTest, EncodesCompactlyInDictOrder) {
  const value_t v = make_dict({{"s", value_t{std::string("x\"y")}},
                               {"i", value_t{int64_t{-7}}},
                               {"f", value_t{1.5}},
                               {"whole", value_t{2.0}},
                               {"t", value_t{true}},
                               {"F", value_t{false}},
                               {"n", nil_value},
                               {"a", make_list({value_t{int64_t{1}}, make_list(), make_dict()})},
                               {"b", make_dict({{"k", value_t{std::string("v")}}})}});
  const std::string want = R"({"s":"x\"y","i":-7,"f":1.5,"whole":2.0,"t":true,"F":false,"n":null,)"
                           R"("a":[1,[],{}],"b":{"k":"v"}})";
  EXPECT_EQ(encode(v), want);
  EXPECT_EQ(encode(nil_value), "null");
  EXPECT_EQ(encode(value_t{std::string("")}), "\"\"");
}

TEST_F(JsonBuiltinsTest, EncodesIndented) {
  const value_t v = make_dict({{"a", make_list({value_t{int64_t{1}}, make_dict()})},
                               {"b", make_dict({{"c", nil_value}})},
                               {"d", make_list()}});
  EXPECT_EQ(encode(v, value_t{true}), "{\n"
                                      "  \"a\": [\n"
                                      "    1,\n"
                                      "    {}\n"
                                      "  ],\n"
                                      "  \"b\": {\n"
                                      "    \"c\": null\n"
                                      "  },\n"
                                      "  \"d\": []\n"
                                      "}");
  EXPECT_EQ(encode(make_list({value_t{int64_t{1}}}), value_t{int64_t{4}}), "[\n    1\n]");
  EXPECT_EQ(encode(v, value_t{int64_t{0}}), encode(v));
  EXPECT_EQ(encode(v, nil_value), encode(v));
  EXPECT_EQ(encode(value_t{int64_t{5}}, value_t{true}), "5");
}

TEST_F(JsonBuiltinsTest, EncodesStringsWithTheEscapesJsonNeeds) {
  const std::string escaped = R"("q\" b\\ s/ \b\f\n\r\t\u0001\u001f)"
                              "\x7f\"";
  EXPECT_EQ(encode(value_t{std::string("q\" b\\ s/ \b\f\n\r\t\x01\x1f\x7f")}), escaped);
  EXPECT_EQ(encode(value_t{std::string("\xC3\xA9\xF0\x9F\x98\x80")}),
            "\"\xC3\xA9\xF0\x9F\x98\x80\"");
  const std::string nul = R"("\u0000")";
  EXPECT_EQ(encode(value_t{std::string(1, '\0')}), nul);
}

TEST_F(JsonBuiltinsTest, FloatsEncodeInTheirShortestRoundTripForm) {
  EXPECT_EQ(encode(value_t{0.1}), "0.1");
  EXPECT_EQ(encode(value_t{1.0}), "1.0");
  EXPECT_EQ(encode(value_t{-0.0}), "-0.0");
  EXPECT_EQ(encode(value_t{100.0}), "100.0");
  for (double d : {0.1, 1.0 / 3.0, 1e300, -1e-300, 5e-324, 1e21, 1e-7, 123456789.125,
                   std::numeric_limits<double>::max()}) {
    const value_t back = parse(encode(value_t{d}));
    ASSERT_TRUE(std::holds_alternative<double>(back.v)) << encode(value_t{d});
    EXPECT_EQ(std::get<double>(back.v), d) << encode(value_t{d});
  }
}

TEST_F(JsonBuiltinsTest, WhatJsonCantHoldIsAnError) {
  const auto encode_error = [&](const value_t &v) { return error_of([&] { encode(v); }); };
  EXPECT_EQ(encode_error(value_t{symbol{"x"}}).rfind("TypeError", 0), 0u);
  EXPECT_EQ(encode_error(*env->lookup("+")).rfind("TypeError", 0), 0u);
  EXPECT_EQ(encode_error(make_list({make_bytes("\x01")})).rfind("TypeError", 0), 0u)
      << encode_error(make_list({make_bytes("\x01")}));
  EXPECT_EQ(encode_error(value_t{std::nan("")}).rfind("ValueError", 0), 0u);
  EXPECT_EQ(encode_error(value_t{std::numeric_limits<double>::infinity()}).rfind("ValueError", 0),
            0u);
  EXPECT_NE(encode_error(value_t{std::string("\xFF")}).find("isn't UTF-8"), std::string::npos);
  EXPECT_NE(encode_error(make_dict({{"\xFF", nil_value}})).find("isn't UTF-8"), std::string::npos);
  // A list that holds itself is an error; one list held twice is not.
  value_t self = make_list({value_t{int64_t{1}}});
  std::get<list_ptr>(self.v)->push_back(self);
  EXPECT_NE(encode_error(self).find("a list that contains itself"), std::string::npos);
  std::get<list_ptr>(self.v)->pop_back(); // break the cycle
  const value_t shared = make_list({value_t{int64_t{1}}});
  EXPECT_EQ(encode(make_list({shared, shared})), "[[1],[1]]");
  // As deep as json-parse reads, and no deeper.
  value_t deep = make_list();
  for (std::size_t i = 1; i < kJsonMaxDepth; ++i)
    deep = make_list({deep});
  EXPECT_EQ(encode(deep), std::string(kJsonMaxDepth, '[') + std::string(kJsonMaxDepth, ']'));
  EXPECT_NE(encode_error(make_list({deep})).find("nested deeper than 512"), std::string::npos);
  // The indent.
  const value_t one = value_t{int64_t{1}};
  EXPECT_EQ(error_of([&] { encode(one, value_t{int64_t{-1}}); }).rfind("ValueError", 0), 0u);
  EXPECT_EQ(error_of([&] { encode(one, value_t{int64_t{11}}); }).rfind("ValueError", 0), 0u);
  EXPECT_EQ(error_of([&] { encode(one, value_t{std::string("2")}); }).rfind("TypeError", 0), 0u);
  EXPECT_EQ(error_of([&] { call("json-encode", {}); }).rfind("TypeError", 0), 0u);
}

TEST_F(JsonBuiltinsTest, RoundTrips) {
  const value_t v = make_dict(
      {{"name", value_t{std::string("caf\xC3\xA9 \xF0\x9F\x98\x80 \"q\"\n")}},
       {"ints", make_list({value_t{std::numeric_limits<int64_t>::min()}, value_t{int64_t{0}},
                           value_t{std::numeric_limits<int64_t>::max()}})},
       {"floats", make_list({value_t{0.5}, value_t{-2.0}, value_t{6.02e23}})},
       {"flags", make_list({value_t{true}, value_t{false}, nil_value})},
       {"nested", make_dict({{"a", make_list({make_dict({{"b", make_list()}})})}})},
       {"", value_t{std::string("empty key")}}});
  for (const value_t &indent : {nil_value, value_t{true}, value_t{int64_t{3}}}) {
    const value_t back = parse(encode(v, indent));
    EXPECT_TRUE(values_equal(back, v)) << to_string(back);
    // -2.0 stays a float (values_equal would take the int -2 as equal).
    EXPECT_TRUE(
        std::holds_alternative<double>(std::get<list_ptr>(member(back, "floats").v)->at(1).v));
  }
  // Compact text comes back as it was.
  const std::string text = R"({"b":[1,2.5,"\u0001",{"c":null}],"a":true,"e":"\u00e9"})";
  EXPECT_EQ(encode(parse(text)),
            "{\"b\":[1,2.5,\"\\u0001\",{\"c\":null}],\"a\":true,\"e\":\"\xC3\xA9\"}");
  EXPECT_EQ(encode(parse(encode(parse(text)))), encode(parse(text)));
}

TEST_F(JsonBuiltinsTest, MangledTextParsesOrIsAValueErrorAndWhatParsesRoundTrips) {
  // Documents get JSON from outside: no input may crash the parser or escape as anything but a
  // ValueError, and whatever it reads, json-encode writes back as the same value.
  const std::string seed = R"({"a": [1, -2.5e3, "x\u00e9\ud83d\ude00\n", true, false, null],)"
                           R"( "b": {"c": {}, "d": [[], [0]]}, "\"k\"": 12345678901234567890})";
  const std::string alphabet = "{}[]\",:\\/ \n\tu0123456789abcdefABCDEF.eE+-truefalsn\x80\xC3\xFF";
  std::mt19937 rng(20261029);
  int parsed = 0;
  for (int round = 0; round < 3000; ++round) {
    std::string text = seed;
    const int edits = 1 + static_cast<int>(rng() % 4);
    for (int e = 0; e < edits; ++e) {
      const std::size_t at = rng() % (text.size() + 1);
      const char c = alphabet[rng() % alphabet.size()];
      switch (rng() % 3) {
      case 0:
        text.insert(text.begin() + static_cast<std::ptrdiff_t>(at), c);
        break;
      case 1:
        if (at < text.size())
          text.erase(at, 1);
        break;
      default:
        if (at < text.size())
          text[at] = c;
      }
    }
    try {
      const value_t v = parse(text);
      ++parsed;
      EXPECT_TRUE(values_equal(parse(encode(v)), v)) << text;
    } catch (const dsl_error &e) {
      EXPECT_EQ(e.class_name(), "ValueError") << text << ": " << e.what();
    }
  }
  EXPECT_GT(parsed, 100); // the mangling leaves some valid documents too
}

// ── sha256, sha256-hex, random-token ──────────────────────────────────────────────────────────

namespace {
// The DSL class an expression's native call raises, or "" when it returns.
template <class F> std::string raised_class(F f) {
  try {
    f();
  } catch (const dsl_error &e) { // what a builtin throws; the evaluator makes it an exception
    return e.class_name();
  }
  return "";
}
} // namespace

// FIPS 180-4 / NIST known answers.
TEST_F(BuiltinsTest, Sha256HexKnownAnswers) {
  EXPECT_EQ(std::get<std::string>(call("sha256-hex", {std::string("")}).v),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(std::get<std::string>(call("sha256-hex", {std::string("abc")}).v),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(std::get<std::string>(
                call("sha256-hex",
                     {std::string("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")})
                    .v),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  // A string hashes its UTF-8 bytes, the same as those bytes as a bytes value.
  EXPECT_EQ(std::get<std::string>(call("sha256-hex", {std::string("caf\xc3\xa9")}).v),
            std::get<std::string>(call("sha256-hex", {make_bytes("caf\xc3\xa9")}).v));
}

TEST_F(BuiltinsTest, Sha256ReturnsTheRawDigest) {
  const value_t d = call("sha256", {std::string("abc")});
  const auto *b = std::get_if<bytes_value>(&d.v);
  ASSERT_NE(b, nullptr);
  ASSERT_EQ(b->data.size(), 32u);
  EXPECT_EQ(static_cast<unsigned char>(b->data[0]), 0xba);
  EXPECT_EQ(static_cast<unsigned char>(b->data[31]), 0xad);
  EXPECT_EQ(raised_class([&] { call("sha256", {int64_t{1}}); }), "TypeError");
  EXPECT_EQ(raised_class([&] { call("sha256-hex", {}); }), "TypeError");
}

TEST_F(BuiltinsTest, RandomTokenLengthAndAlphabet) {
  for (int64_t n : {1, 2, 3, 31, 32, 33, 256}) {
    SCOPED_TRACE(n);
    const std::string b64 = std::get<std::string>(call("random-token", {n}).v);
    EXPECT_EQ(b64.size(), static_cast<std::size_t>((4 * n + 2) / 3)); // unpadded base64url
    EXPECT_EQ(b64.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
                                    "0123456789-_"),
              std::string::npos)
        << b64;
    const std::string hex = std::get<std::string>(call("random-token", {n, std::string("hex")}).v);
    EXPECT_EQ(hex.size(), static_cast<std::size_t>(2 * n));
    EXPECT_EQ(hex.find_first_not_of("0123456789abcdef"), std::string::npos) << hex;
  }
  // 32 random bytes twice over are never the same (a 2^-256 chance).
  EXPECT_NE(std::get<std::string>(call("random-token", {int64_t{32}}).v),
            std::get<std::string>(call("random-token", {int64_t{32}}).v));
}

TEST_F(BuiltinsTest, RandomTokenRejectsBadArguments) {
  EXPECT_EQ(raised_class([&] { call("random-token", {int64_t{0}}); }), "ValueError");
  EXPECT_EQ(raised_class([&] { call("random-token", {int64_t{257}}); }), "ValueError");
  EXPECT_EQ(raised_class([&] { call("random-token", {1.5}); }), "TypeError");
  EXPECT_EQ(raised_class([&] { call("random-token", {}); }), "TypeError");
  EXPECT_EQ(raised_class([&] { call("random-token", {int64_t{8}, std::string("b32")}); }),
            "ValueError");
}

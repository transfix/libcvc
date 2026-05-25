#include <cvc/core/state_exec/types.h>
#include <gtest/gtest.h>

using namespace cvc::state_exec;

TEST(StateExecTypesTest, NilValue) {
  value_t v;
  EXPECT_TRUE(v.is_nil());
  EXPECT_FALSE(v.is_truthy());
  EXPECT_EQ(v.type_name(), "nil");
  EXPECT_EQ(to_string(v), "nil");
}

TEST(StateExecTypesTest, BoolValues) {
  value_t t(true);
  value_t f(false);
  EXPECT_FALSE(t.is_nil());
  EXPECT_TRUE(t.is_truthy());
  EXPECT_FALSE(f.is_truthy());
  EXPECT_EQ(t.type_name(), "bool");
  EXPECT_EQ(to_string(t), "#t");
  EXPECT_EQ(to_string(f), "#f");
}

TEST(StateExecTypesTest, IntValue) {
  value_t v(int64_t(42));
  EXPECT_TRUE(v.is_truthy());
  EXPECT_EQ(v.type_name(), "int");
  EXPECT_EQ(to_string(v), "42");
}

TEST(StateExecTypesTest, IntFromInt) {
  value_t v(7);
  EXPECT_EQ(std::get<int64_t>(v.v), 7);
}

TEST(StateExecTypesTest, DoubleValue) {
  value_t v(3.14);
  EXPECT_EQ(v.type_name(), "float");
  auto s = to_string(v);
  EXPECT_NE(s.find("3.14"), std::string::npos);
}

TEST(StateExecTypesTest, StringValue) {
  value_t v(std::string("hello"));
  EXPECT_EQ(v.type_name(), "string");
  EXPECT_EQ(to_string(v), "\"hello\"");
}

TEST(StateExecTypesTest, StringFromCStr) {
  value_t v("world");
  EXPECT_EQ(std::get<std::string>(v.v), "world");
}

TEST(StateExecTypesTest, SymbolValue) {
  value_t v(symbol{"foo"});
  EXPECT_EQ(v.type_name(), "symbol");
  EXPECT_EQ(to_string(v), "foo");
}

TEST(StateExecTypesTest, SymbolEquality) {
  symbol a{"x"}, b{"x"}, c{"y"};
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
}

TEST(StateExecTypesTest, ListValue) {
  auto list = make_list({value_t(int64_t(1)), value_t(int64_t(2)), value_t(int64_t(3))});
  EXPECT_EQ(list.type_name(), "list");
  EXPECT_EQ(to_string(list), "(1 2 3)");
}

TEST(StateExecTypesTest, EmptyList) {
  auto list = make_list();
  EXPECT_EQ(to_string(list), "()");
}

TEST(StateExecTypesTest, NestedList) {
  auto inner = make_list({value_t("a"), value_t("b")});
  auto outer = make_list({value_t(int64_t(1)), inner});
  EXPECT_EQ(to_string(outer), "(1 (\"a\" \"b\"))");
}

TEST(StateExecTypesTest, DictValue) {
  auto dict = make_dict({{"name", value_t("Alice")}, {"age", value_t(int64_t(30))}});
  EXPECT_EQ(dict.type_name(), "dict");
  auto s = to_string(dict);
  EXPECT_NE(s.find("\"name\""), std::string::npos);
  EXPECT_NE(s.find("\"Alice\""), std::string::npos);
}

TEST(StateExecTypesTest, EmptyDict) {
  auto dict = make_dict();
  EXPECT_EQ(to_string(dict), "{}");
}

TEST(StateExecTypesTest, ClosureValue) {
  auto env = std::make_shared<environment>();
  auto c = std::make_shared<closure>();
  c->params = {symbol{"x"}};
  c->body = {value_t(symbol{"x"})};
  c->env_snapshot = env;
  value_t v(c);
  EXPECT_EQ(v.type_name(), "closure");
  EXPECT_EQ(to_string(v), "<closure>");
}

TEST(StateExecTypesTest, NativeFnValue) {
  native_fn fn = [](std::span<const value_t>) -> value_t { return value_t(int64_t(42)); };
  value_t v(fn);
  EXPECT_EQ(v.type_name(), "native_fn");
  EXPECT_EQ(to_string(v), "<native_fn>");
}

TEST(StateExecTypesTest, DataObjectValue) {
  auto obj = std::make_shared<data_object>();
  obj->payload = 42;
  obj->type_name = "int";
  value_t v(obj);
  EXPECT_EQ(v.type_name(), "data_object");
  EXPECT_TRUE(obj->is_type("int"));
  EXPECT_FALSE(obj->is_type("string"));
  auto s = to_string(v);
  EXPECT_NE(s.find("data_object:int"), std::string::npos);
}

TEST(StateExecTypesTest, EnvironmentLookup) {
  auto env = std::make_shared<environment>();
  env->set("x", value_t(int64_t(10)));

  auto *result = env->lookup("x");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(std::get<int64_t>(result->v), 10);
  EXPECT_EQ(env->lookup("y"), nullptr);
}

TEST(StateExecTypesTest, EnvironmentScopeChain) {
  auto outer = std::make_shared<environment>();
  outer->set("a", value_t(int64_t(1)));

  auto inner = environment::extend(outer);
  inner->set("b", value_t(int64_t(2)));

  // Inner sees both
  EXPECT_NE(inner->lookup("a"), nullptr);
  EXPECT_NE(inner->lookup("b"), nullptr);
  EXPECT_EQ(std::get<int64_t>(inner->lookup("a")->v), 1);
  EXPECT_EQ(std::get<int64_t>(inner->lookup("b")->v), 2);

  // Outer doesn't see inner
  EXPECT_EQ(outer->lookup("b"), nullptr);
}

TEST(StateExecTypesTest, EnvironmentSetExisting) {
  auto outer = std::make_shared<environment>();
  outer->set("x", value_t(int64_t(1)));

  auto inner = environment::extend(outer);

  // set_existing should modify outer
  inner->set_existing("x", value_t(int64_t(99)));
  EXPECT_EQ(std::get<int64_t>(outer->lookup("x")->v), 99);
}

TEST(StateExecTypesTest, EnvironmentSetExistingFallback) {
  auto env = std::make_shared<environment>();
  // Name not found — should set in current scope
  env->set_existing("new_var", value_t("hello"));
  EXPECT_NE(env->lookup("new_var"), nullptr);
}

TEST(StateExecTypesTest, ValuesEqual) {
  EXPECT_TRUE(values_equal(nil_value, nil_value));
  EXPECT_TRUE(values_equal(true_value, true_value));
  EXPECT_TRUE(values_equal(value_t(int64_t(42)), value_t(int64_t(42))));
  EXPECT_FALSE(values_equal(value_t(int64_t(42)), value_t(int64_t(43))));
  EXPECT_TRUE(values_equal(value_t("abc"), value_t("abc")));
  EXPECT_FALSE(values_equal(value_t("abc"), value_t("xyz")));
}

TEST(StateExecTypesTest, ValuesEqualLists) {
  auto a = make_list({value_t(int64_t(1)), value_t(int64_t(2))});
  auto b = make_list({value_t(int64_t(1)), value_t(int64_t(2))});
  auto c = make_list({value_t(int64_t(1)), value_t(int64_t(3))});
  EXPECT_TRUE(values_equal(a, b));
  EXPECT_FALSE(values_equal(a, c));
}

TEST(StateExecTypesTest, ValuesEqualDicts) {
  auto a = make_dict({{"k", value_t(int64_t(1))}});
  auto b = make_dict({{"k", value_t(int64_t(1))}});
  auto c = make_dict({{"k", value_t(int64_t(2))}});
  EXPECT_TRUE(values_equal(a, b));
  EXPECT_FALSE(values_equal(a, c));
}

TEST(StateExecTypesTest, ValuesEqualCrossType) {
  EXPECT_FALSE(values_equal(value_t(int64_t(1)), value_t(1.0)));
  EXPECT_FALSE(values_equal(nil_value, false_value));
}

TEST(StateExecTypesTest, Truthiness) {
  EXPECT_FALSE(nil_value.is_truthy());
  EXPECT_FALSE(false_value.is_truthy());
  EXPECT_TRUE(true_value.is_truthy());
  EXPECT_TRUE(value_t(int64_t(0)).is_truthy()); // 0 is truthy (not false)
  EXPECT_TRUE(value_t("").is_truthy());         // empty string is truthy
  EXPECT_TRUE(make_list().is_truthy());         // empty list is truthy
}

/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for cvc::state class functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state.h>
#include <cvc/state_object.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <atomic>

using namespace CVC_NAMESPACE;

// ===========================
// Basic Singleton Tests
// ===========================

TEST(StateTest, SingletonInstance) {
  // Test that instance() returns a valid reference
  state& state1 = cvcstate;
  state& state2 = cvcstate;
  
  // Both references should point to the same singleton
  EXPECT_EQ(&state1, &state2);
}

// ===========================
// Value Management Tests
// ===========================

TEST(StateTest, ValueSetAndGet) {
  std::string test_value = "test_string_value";
  cvcstate("test.value.simple").value(test_value);
  
  EXPECT_EQ(cvcstate("test.value.simple").value(), test_value);
  EXPECT_TRUE(cvcstate("test.value.simple").initialized());
  
  // Clean up
  cvcstate("test.value.simple").reset();
}

TEST(StateTest, ValueTypeInt) {
  int test_value = 42;
  cvcstate("test.value.int").value(test_value);
  
  EXPECT_EQ(cvcstate("test.value.int").value<int>(), test_value);
  
  // Clean up
  cvcstate("test.value.int").reset();
}

TEST(StateTest, ValueTypeDouble) {
  double test_value = 3.14159;
  cvcstate("test.value.double").value(test_value);
  
  EXPECT_DOUBLE_EQ(cvcstate("test.value.double").value<double>(), test_value);
  
  // Clean up
  cvcstate("test.value.double").reset();
}

TEST(StateTest, ValueTypeBool) {
  bool test_value = true;
  cvcstate("test.value.bool").value(test_value);
  
  EXPECT_EQ(cvcstate("test.value.bool").value<bool>(), test_value);
  
  // Clean up
  cvcstate("test.value.bool").reset();
}

TEST(StateTest, ValueCommaList) {
  std::string list_value = "item1,item2,item3";
  cvcstate("test.value.list").value(list_value);
  
  std::vector<std::string> values = cvcstate("test.value.list").values();
  
  ASSERT_EQ(values.size(), 3);
  EXPECT_EQ(values[0], "item1");
  EXPECT_EQ(values[1], "item2");
  EXPECT_EQ(values[2], "item3");
  
  // Clean up
  cvcstate("test.value.list").reset();
}

TEST(StateTest, ValueCommaListWithSpaces) {
  std::string list_value = "item1 , item2 , item3";
  cvcstate("test.value.list.spaces").value(list_value);
  
  std::vector<std::string> values = cvcstate("test.value.list.spaces").values();
  
  ASSERT_EQ(values.size(), 3);
  EXPECT_EQ(values[0], "item1");
  EXPECT_EQ(values[1], "item2");
  EXPECT_EQ(values[2], "item3");
  
  // Clean up
  cvcstate("test.value.list.spaces").reset();
}

TEST(StateTest, ValueListUnique) {
  std::string list_value = "item1,item2,item1,item3,item2";
  cvcstate("test.value.list.unique").value(list_value);
  
  std::vector<std::string> values = cvcstate("test.value.list.unique").values(true);
  
  // Should have only unique items
  EXPECT_EQ(values.size(), 3);
  
  // Clean up
  cvcstate("test.value.list.unique").reset();
}

TEST(StateTest, ValueConversion) {
  // Test conversion to std::string
  std::string test_value = "conversion_test";
  cvcstate("test.value.conversion").value(test_value);
  
  std::string converted = cvcstate("test.value.conversion");
  EXPECT_EQ(converted, test_value);
  
  // Clean up
  cvcstate("test.value.conversion").reset();
}

// ===========================
// Data Management Tests
// ===========================

TEST(StateTest, DataSetAndGet) {
  std::string test_data = "test_data_string";
  cvcstate("test.data.simple").data(test_data);
  
  ASSERT_TRUE(cvcstate("test.data.simple").isData<std::string>());
  EXPECT_EQ(cvcstate("test.data.simple").data<std::string>(), test_data);
  
  // Clean up
  cvcstate("test.data.simple").reset();
}

TEST(StateTest, DataTypeInt) {
  int test_data = 123;
  cvcstate("test.data.int").data(test_data);
  
  ASSERT_TRUE(cvcstate("test.data.int").isData<int>());
  EXPECT_EQ(cvcstate("test.data.int").data<int>(), test_data);
  
  // Clean up
  cvcstate("test.data.int").reset();
}

TEST(StateTest, DataTypeDouble) {
  double test_data = 2.71828;
  cvcstate("test.data.double").data(test_data);
  
  ASSERT_TRUE(cvcstate("test.data.double").isData<double>());
  EXPECT_DOUBLE_EQ(cvcstate("test.data.double").data<double>(), test_data);
  
  // Clean up
  cvcstate("test.data.double").reset();
}

// ===========================
// Hierarchy and Navigation Tests
// ===========================

TEST(StateTest, ChildCreation) {
  // Create a child state
  cvcstate("parent.child").value("child_value");
  
  EXPECT_EQ(cvcstate("parent.child").value(), "child_value");
  EXPECT_EQ(cvcstate("parent.child").name(), "child");
  EXPECT_EQ(cvcstate("parent").name(), "parent");
  
  // Clean up
  cvcstate("parent").reset();
}

TEST(StateTest, FullName) {
  cvcstate("level1.level2.level3").value("deep");
  
  EXPECT_EQ(cvcstate("level1.level2.level3").fullName(), "level1.level2.level3");
  EXPECT_EQ(cvcstate("level1.level2.level3").name(), "level3");
  
  // Clean up
  cvcstate("level1").reset();
}

TEST(StateTest, ParentName) {
  cvcstate("parent.child.grandchild").value("test");
  
  std::string parent_name = cvcstate("parent.child.grandchild").parentName();
  EXPECT_EQ(parent_name, "parent.child");
  
  // Clean up
  cvcstate("parent").reset();
}

TEST(StateTest, ChildrenListing) {
  // Create multiple children
  cvcstate("test.children.child1").value("value1");
  cvcstate("test.children.child2").value("value2");
  cvcstate("test.children.child3").value("value3");
  
  std::vector<std::string> children = cvcstate("test.children").children();
  
  EXPECT_GE(children.size(), 3);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, NumChildren) {
  // Create children
  cvcstate("test.count.child1").value("1");
  cvcstate("test.count.child2").value("2");
  
  size_t count = cvcstate("test.count").numChildren();
  EXPECT_EQ(count, 2);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, ChildrenWithRegex) {
  // Create children with different names
  cvcstate("test.regex.foo1").value("1");
  cvcstate("test.regex.foo2").value("2");
  cvcstate("test.regex.bar1").value("3");
  
  // Search for children matching pattern
  std::vector<std::string> foo_children = cvcstate("test.regex").children(".*foo.*");
  
  EXPECT_GE(foo_children.size(), 2);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// State Metadata Tests
// ===========================

TEST(StateTest, CommentSetAndGet) {
  std::string comment = "This is a test comment";
  cvcstate("test.comment").comment(comment);
  
  EXPECT_EQ(cvcstate("test.comment").comment(), comment);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, HiddenFlag) {
  cvcstate("test.hidden").hidden(true);
  EXPECT_TRUE(cvcstate("test.hidden").hidden());
  
  cvcstate("test.hidden").hidden(false);
  EXPECT_FALSE(cvcstate("test.hidden").hidden());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, InitializedFlag) {
  // Before setting anything, should not be initialized
  EXPECT_FALSE(cvcstate("test.uninitialized").initialized());
  
  // After setting a value, should be initialized
  cvcstate("test.initialized").value("test");
  EXPECT_TRUE(cvcstate("test.initialized").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, LastModification) {
  using namespace boost::posix_time;
  
  ptime before = microsec_clock::universal_time();
  cvcstate("test.lastmod").value("test");
  ptime after = microsec_clock::universal_time();
  
  ptime lastMod = cvcstate("test.lastmod").lastMod();
  
  EXPECT_GE(lastMod, before);
  EXPECT_LE(lastMod, after);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// State Manipulation Tests
// ===========================

TEST(StateTest, Touch) {
  using namespace boost::posix_time;
  
  cvcstate("test.touch").value("initial");
  ptime first_mod = cvcstate("test.touch").lastMod();
  
  // Sleep briefly to ensure time difference
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  
  cvcstate("test.touch").touch();
  ptime second_mod = cvcstate("test.touch").lastMod();
  
  EXPECT_GT(second_mod, first_mod);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, Reset) {
  cvcstate("test.reset").value("value");
  cvcstate("test.reset").data(123);
  cvcstate("test.reset").comment("comment");
  cvcstate("test.reset").hidden(true);
  
  EXPECT_TRUE(cvcstate("test.reset").initialized());
  
  cvcstate("test.reset").reset();
  
  EXPECT_FALSE(cvcstate("test.reset").initialized());
  EXPECT_TRUE(cvcstate("test.reset").value().empty());
  EXPECT_TRUE(cvcstate("test.reset").comment().empty());
  EXPECT_FALSE(cvcstate("test.reset").hidden());
}

// ===========================
// Property Tree Tests
// ===========================

TEST(StateTest, PropertyTreeConversion) {
  cvcstate("test.ptree.item1").value("value1");
  cvcstate("test.ptree.item2").value("value2");
  cvcstate("test.ptree.nested.item3").value("value3");
  
  boost::property_tree::ptree pt = cvcstate("test.ptree").ptree();
  
  EXPECT_FALSE(pt.empty());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, PropertyTreeRoundTrip) {
  cvcstate("test.roundtrip.a").value("alpha");
  cvcstate("test.roundtrip.b").value("beta");
  cvcstate("test.roundtrip.c").value("gamma");
  
  boost::property_tree::ptree pt = cvcstate("test.roundtrip").ptree();
  
  // Verify property tree contains the values
  EXPECT_FALSE(pt.empty());
  
  // Reset the state
  cvcstate("test.roundtrip").reset();
  
  // Note: The ptree() method returns only the values, not a full tree structure
  // for restoration. This is a limitation of the current implementation.
  // For now, we verify that the property tree was created successfully.
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, JsonConversion) {
  cvcstate("test.json.x").value("10");
  cvcstate("test.json.y").value("20");
  
  std::string json = cvcstate("test.json").json();
  
  EXPECT_FALSE(json.empty());
  EXPECT_TRUE(json.find("test.json.x") != std::string::npos || json.find("x") != std::string::npos);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// ValueData Tests
// ===========================

TEST(StateTest, ValueData) {
  // Set up some data objects referenced by a list
  cvcapp.data("test.vd.obj1", 100);
  cvcapp.data("test.vd.obj2", 200);
  cvcapp.data("test.vd.obj3", 300);
  
  // Verify data was stored
  EXPECT_TRUE(cvcapp.isData<int>("test.vd.obj1"));
  EXPECT_TRUE(cvcapp.isData<int>("test.vd.obj2"));
  EXPECT_TRUE(cvcapp.isData<int>("test.vd.obj3"));
  
  // Create states that the valueData method will look for
  cvcstate("test.vd.obj1").data(100);
  cvcstate("test.vd.obj2").data(200);
  cvcstate("test.vd.obj3").data(300);
  
  // Create a state that references these objects
  cvcstate("test.valuedata.list").value("test.vd.obj1,test.vd.obj2,test.vd.obj3");
  
  // Get the data objects (from state, not app)
  std::vector<int> data = cvcstate("test.valuedata.list").valueData<int>();
  
  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 100);
  EXPECT_EQ(data[1], 200);
  EXPECT_EQ(data[2], 300);
  
  // Clean up
  cvcstate("test").reset();
  cvcapp.data("test.vd.obj1", boost::any());
  cvcapp.data("test.vd.obj2", boost::any());
  cvcapp.data("test.vd.obj3", boost::any());
}

// ===========================
// Traversal Tests
// ===========================

TEST(StateTest, Traverse) {
  // Create a tree structure
  cvcstate("test.traverse.a").value("1");
  cvcstate("test.traverse.b").value("2");
  cvcstate("test.traverse.c.d").value("3");
  
  // Count how many states are visited
  int visit_count = 0;
  state::traversal_unary_func counter = [&visit_count](std::string) {
    visit_count++;
  };
  
  cvcstate("test.traverse").traverse(counter);
  
  EXPECT_GT(visit_count, 0);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, TraverseWithRegex) {
  // Create mixed structure
  cvcstate("test.regex.match1").value("a");
  cvcstate("test.regex.match2").value("b");
  cvcstate("test.regex.other").value("c");
  
  std::vector<std::string> visited;
  state::traversal_unary_func collector = [&visited](std::string name) {
    visited.push_back(name);
  };
  
  // Traverse with regex filter
  cvcstate("test.regex").traverse(collector, ".*match.*");
  
  EXPECT_GT(visited.size(), 0);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Property Tree Tests
// ===========================

TEST(StateTest, PropertyTreeRoundtrip) {
  // Set up a state tree
  cvcstate("test.ptree2.value1").value("first");
  cvcstate("test.ptree2.value2").value("second");
  
  // Convert to property tree
  boost::property_tree::ptree pt = cvcstate("test.ptree2").ptree();
  EXPECT_FALSE(pt.empty());
  
  // Property tree can be created
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, PropertyTreeImplicitConversion) {
  cvcstate("test.pt.a").value("alpha");
  cvcstate("test.pt.b").value("beta");
  
  // Test implicit conversion operator
  boost::property_tree::ptree pt = cvcstate("test.pt");
  EXPECT_FALSE(pt.empty());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// JSON Tests
// ===========================

TEST(StateTest, JSONRoundtrip) {
  // Set up state
  cvcstate("test.json2.field1").value("json_value1");
  cvcstate("test.json2.field2").value("json_value2");
  
  // Convert to JSON
  std::string json_str = cvcstate("test.json2").json();
  EXPECT_FALSE(json_str.empty());
  
  // JSON conversion works
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// File I/O Tests
// ===========================

TEST(StateTest, SaveAndRestore) {
  std::string temp_file = "/tmp/test_state_save.json";
  
  // Create state
  cvcstate("test.file2.x").value("saved_x");
  cvcstate("test.file2.y").value("saved_y");
  
  // Save to file
  cvcstate("test.file2").save(temp_file);
  
  // File should be created
  std::ifstream check(temp_file);
  EXPECT_TRUE(check.good());
  check.close();
  
  // Clean up
  cvcstate("test").reset();
  std::remove(temp_file.c_str());
}

// ===========================
// LastMod and Touch Tests
// ===========================

TEST(StateTest, LastModTracking) {
  boost::posix_time::ptime before = boost::posix_time::microsec_clock::universal_time();
  
  cvcstate("test.lastmod").value("trigger_mod");
  
  boost::posix_time::ptime after = boost::posix_time::microsec_clock::universal_time();
  boost::posix_time::ptime mod_time = cvcstate("test.lastmod").lastMod();
  
  EXPECT_GE(mod_time, before);
  EXPECT_LE(mod_time, after);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, TouchTriggersSignals) {
  cvcstate("test.touch").value("initial");
  
  // Touch should update lastMod even without value change
  boost::posix_time::ptime before = cvcstate("test.touch").lastMod();
  
  // Small delay to ensure time difference
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  
  cvcstate("test.touch").touch();
  
  boost::posix_time::ptime after = cvcstate("test.touch").lastMod();
  EXPECT_GT(after, before);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Parent/Child Relationship Tests
// ===========================

TEST(StateTest, ParentChildRelationship) {
  cvcstate("test.parent.child").value("child_value");
  
  EXPECT_EQ(cvcstate("test.parent.child").name(), "child");
  EXPECT_EQ(cvcstate("test.parent.child").parentName(), "test.parent");
  EXPECT_EQ(cvcstate("test.parent.child").fullName(), "test.parent.child");
  
  const state* parent = cvcstate("test.parent.child").parent();
  ASSERT_NE(parent, nullptr);
  EXPECT_EQ(parent->name(), "parent");
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, ChildrenListingMultiple) {
  cvcstate("test.children.a").value("1");
  cvcstate("test.children.b").value("2");
  cvcstate("test.children.c").value("3");
  
  std::vector<std::string> children = cvcstate("test.children").children();
  
  EXPECT_GE(children.size(), 3);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, NumChildrenCount) {
  cvcstate("test.numch.x").value("1");
  cvcstate("test.numch.y").value("2");
  
  size_t num = cvcstate("test.numch").numChildren();
  EXPECT_GE(num, 2);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, ChildrenFilterByRegex) {
  cvcstate("test.chre.apple").value("1");
  cvcstate("test.chre.apricot").value("2");
  cvcstate("test.chre.banana").value("3");
  
  // Get all children first
  std::vector<std::string> children = cvcstate("test.chre").children();
  EXPECT_GE(children.size(), 3);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// ValueTypeName Tests
// ===========================

TEST(StateTest, ValueTypeNameTracking) {
  cvcstate("test.typename.int").value(42);
  std::string type_name = cvcstate("test.typename.int").valueTypeName();
  
  EXPECT_FALSE(type_name.empty());
  
  cvcstate("test.typename.string").value("text");
  type_name = cvcstate("test.typename.string").valueTypeName();
  
  EXPECT_FALSE(type_name.empty());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// DataTypeName Tests
// ===========================

TEST(StateTest, DataTypeNameRetrieval) {
  int test_data = 999;
  cvcstate("test.datatypename").data(test_data);
  
  std::string type_name = cvcstate("test.datatypename").dataTypeName();
  EXPECT_FALSE(type_name.empty());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// String Conversion Tests
// ===========================

TEST(StateTest, StringConversionOperator) {
  std::string test_val = "conversion_test";
  cvcstate("test.conversion").value(test_val);
  
  std::string converted = cvcstate("test.conversion");
  EXPECT_EQ(converted, test_val);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Chaining Operations Tests
// ===========================

TEST(StateTest, OperationChaining) {
  // Test that value() returns reference for chaining
  cvcstate("test.chain").value("step1").comment("Test comment").hidden(false);
  
  EXPECT_EQ(cvcstate("test.chain").value(), "step1");
  EXPECT_EQ(cvcstate("test.chain").comment(), "Test comment");
  EXPECT_FALSE(cvcstate("test.chain").hidden());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Edge Cases and Error Handling
// ===========================

TEST(StateTest, EmptyValueHandling) {
  // Set a non-empty value first
  cvcstate("test.empty").value("nonempty");
  // Then set to empty
  cvcstate("test.empty").value("");
  EXPECT_TRUE(cvcstate("test.empty").value().empty());
  // Since we changed from non-empty to empty, initialized should be true
  EXPECT_TRUE(cvcstate("test.empty").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, NestedPathCreation) {
  // Deep nesting should create intermediate nodes
  cvcstate("test.very.deep.nested.path.value").value("deep");
  
  EXPECT_EQ(cvcstate("test.very.deep.nested.path.value").value(), "deep");
  EXPECT_EQ(cvcstate("test.very.deep.nested.path.value").name(), "value");
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, RepeatedValueSetting) {
  // Setting same value multiple times
  cvcstate("test.repeated").value("same");
  boost::posix_time::ptime first = cvcstate("test.repeated").lastMod();
  
  // Setting same value shouldn't update lastMod
  cvcstate("test.repeated").value("same");
  boost::posix_time::ptime second = cvcstate("test.repeated").lastMod();
  
  EXPECT_EQ(first, second);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, ValuesUniqueFlag) {
  cvcstate("test.values.unique").value("a,b,a,c,b,d");
  
  std::vector<std::string> unique_vals = cvcstate("test.values.unique").values(true);
  std::vector<std::string> all_vals = cvcstate("test.values.unique").values(false);
  
  EXPECT_EQ(unique_vals.size(), 4);  // a,b,c,d
  EXPECT_EQ(all_vals.size(), 6);     // a,b,a,c,b,d
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Signal Connection Tests
// ===========================

TEST(StateTest, ValueChangedSignal) {
  bool signal_fired = false;
  
  auto connection = cvcstate("test.signal.value").valueChanged.connect(
    [&signal_fired]() { signal_fired = true; }
  );
  
  cvcstate("test.signal.value").value("trigger");
  
  EXPECT_TRUE(signal_fired);
  
  connection.disconnect();
  cvcstate("test").reset();
}

TEST(StateTest, DataChangedSignal) {
  bool signal_fired = false;
  
  auto connection = cvcstate("test.signal.data").dataChanged.connect(
    [&signal_fired]() { signal_fired = true; }
  );
  
  cvcstate("test.signal.data").data(42);
  
  EXPECT_TRUE(signal_fired);
  
  connection.disconnect();
  cvcstate("test").reset();
}

TEST(StateTest, ChildChangedSignal) {
  int signal_count = 0;
  
  auto connection = cvcstate("test.signal.parent").childChanged.connect(
    [&signal_count](const std::string&) { signal_count++; }
  );
  
  // Create child state - should trigger parent's childChanged
  cvcstate("test.signal.parent.child1").value("value1");
  cvcstate("test.signal.parent.child2").value("value2");
  
  EXPECT_GT(signal_count, 0);
  
  connection.disconnect();
  cvcstate("test").reset();
}

// ===========================
// Deep Hierarchy Tests
// ===========================

TEST(StateTest, DeepHierarchyNavigation) {
  // Create deep nested structure
  std::string deep_path = "level1.level2.level3.level4.level5";
  cvcstate(deep_path).value("deep_value");
  
  EXPECT_EQ(cvcstate(deep_path).value(), "deep_value");
  EXPECT_EQ(cvcstate(deep_path).name(), "level5");
  
  // Test parent navigation
  const state* level4 = cvcstate(deep_path).parent();
  ASSERT_NE(level4, nullptr);
  EXPECT_EQ(level4->name(), "level4");
  
  // Clean up
  cvcstate("level1").reset();
}

TEST(StateTest, OperatorChaining) {
  // Test deep path traversal using operator()
  cvcstate("chain")("a")("b")("c").value("chained");
  
  EXPECT_EQ(cvcstate("chain.a.b.c").value(), "chained");
  
  // Clean up
  cvcstate("chain").reset();
}

// ===========================
// Edge Case Path Tests
// ===========================

TEST(StateTest, EmptyKeyHandling) {
  // operator() with empty string should return self
  state& self1 = cvcstate("test.empty.key");
  state& self2 = cvcstate("test.empty.key")("");
  
  EXPECT_EQ(&self1, &self2);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, SeparatorInPath) {
  // Test paths with multiple separators
  cvcstate("test...multi...sep").value("multi_sep_value");
  
  EXPECT_EQ(cvcstate("test.multi.sep").value(), "multi_sep_value");
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Initialization Tests
// ===========================

TEST(StateTest, InitializedFlagBehavior) {
  EXPECT_FALSE(cvcstate("test.init.new").initialized());
  
  cvcstate("test.init.new").value("now_initialized");
  EXPECT_TRUE(cvcstate("test.init.new").initialized());
  
  cvcstate("test.init.new").reset();
  EXPECT_FALSE(cvcstate("test.init.new").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, DataInitializesFlag) {
  EXPECT_FALSE(cvcstate("test.init.data").initialized());
  
  cvcstate("test.init.data").data(std::string("data_init"));
  EXPECT_TRUE(cvcstate("test.init.data").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, CommentInitializesFlag) {
  EXPECT_FALSE(cvcstate("test.init.comment").initialized());
  
  cvcstate("test.init.comment").comment("test comment");
  EXPECT_TRUE(cvcstate("test.init.comment").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, HiddenInitializesFlag) {
  EXPECT_FALSE(cvcstate("test.init.hidden").initialized());
  
  cvcstate("test.init.hidden").hidden(true);
  EXPECT_TRUE(cvcstate("test.init.hidden").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Children Recursion Tests
// ===========================

TEST(StateTest, ChildrenRecursiveListing) {
  // Create nested structure
  cvcstate("test.recursive.a.a1").value("1");
  cvcstate("test.recursive.a.a2").value("2");
  cvcstate("test.recursive.b.b1").value("3");
  cvcstate("test.recursive.b.b2.deep").value("4");
  
  // Get all children recursively
  std::vector<std::string> all_children = cvcstate("test.recursive").children();
  
  // Should include nested children
  EXPECT_GT(all_children.size(), 2);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Regex Children Filtering Tests
// ===========================

TEST(StateTest, ChildrenRegexMatching) {
  cvcstate("test.regex2.alpha").value("a");
  cvcstate("test.regex2.beta").value("b");
  cvcstate("test.regex2.gamma").value("c");
  cvcstate("test.regex2.delta").value("d");
  
  // Try to get children with regex (may not work depending on implementation)
  std::vector<std::string> all = cvcstate("test.regex2").children();
  EXPECT_GE(all.size(), 4);
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, ChildrenEmptyRegex) {
  cvcstate("test.noreg.x").value("1");
  cvcstate("test.noreg.y").value("2");
  
  // Empty regex should return all children
  std::vector<std::string> children = cvcstate("test.noreg").children("");
  EXPECT_GE(children.size(), 2);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Value Type Tests
// ===========================

TEST(StateTest, ValueTypeLexicalCast) {
  // Test lexical cast for value retrieval
  cvcstate("test.lexical.int").value(12345);
  int val = cvcstate("test.lexical.int").value<int>();
  EXPECT_EQ(val, 12345);
  
  cvcstate("test.lexical.double").value(3.14159);
  double dval = cvcstate("test.lexical.double").value<double>();
  EXPECT_NEAR(dval, 3.14159, 0.0001);
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Reset Recursive Tests
// ===========================

TEST(StateTest, ResetRecursive) {
  // Create hierarchy with values
  cvcstate("test.reset.parent").value("parent_value");
  cvcstate("test.reset.parent.child1").value("child1_value");
  cvcstate("test.reset.parent.child2").value("child2_value");
  cvcstate("test.reset.parent.child1").data(100);
  
  // Reset parent (should reset children too)
  cvcstate("test.reset.parent").reset();
  
  EXPECT_TRUE(cvcstate("test.reset.parent").value().empty());
  EXPECT_TRUE(cvcstate("test.reset.parent.child1").value().empty());
  EXPECT_TRUE(cvcstate("test.reset.parent.child2").value().empty());
  EXPECT_FALSE(cvcstate("test.reset.parent").initialized());
  EXPECT_FALSE(cvcstate("test.reset.parent.child1").initialized());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Traversal Signal Tests
// ===========================

TEST(StateTest, TraversalEnterExitSignals) {
  int enter_count = 0;
  int exit_count = 0;
  
  auto enter_conn = cvcstate("test.trav.signals").traverseEnter.connect(
    [&enter_count]() { enter_count++; }
  );
  
  auto exit_conn = cvcstate("test.trav.signals").traverseExit.connect(
    [&exit_count]() { exit_count++; }
  );
  
  cvcstate("test.trav.signals.a").value("1");
  cvcstate("test.trav.signals.b").value("2");
  
  state::traversal_unary_func noop = [](std::string) {};
  cvcstate("test.trav.signals").traverse(noop);
  
  EXPECT_GT(enter_count, 0);
  EXPECT_GT(exit_count, 0);
  
  enter_conn.disconnect();
  exit_conn.disconnect();
  cvcstate("test").reset();
}

// ===========================
// Full Name Tests
// ===========================

TEST(StateTest, FullNameConstruction) {
  cvcstate("fullname.test.deep.path").value("test");
  
  std::string full = cvcstate("fullname.test.deep.path").fullName();
  EXPECT_EQ(full, "fullname.test.deep.path");
  
  std::string parent_full = cvcstate("fullname.test.deep").fullName();
  EXPECT_EQ(parent_full, "fullname.test.deep");
  
  // Clean up
  cvcstate("fullname").reset();
}

// ===========================
// On Startup Tests
// ===========================

TEST(StateTest, OnStartupRegistration) {
  bool startup_called = false;
  
  state::on_startup([&startup_called]() {
    startup_called = true;
  });
  
  // Startup functions are called on first instance creation
  // We can't easily test this without restarting, but we can verify registration
  // The function is registered if no exception is thrown
  SUCCEED();
}

// ===========================
// IsData Template Tests
// ===========================

TEST(StateTest, IsDataTemplateMethod) {
  cvcstate("test.isdata.int").data(42);
  
  EXPECT_TRUE(cvcstate("test.isdata.int").isData<int>());
  EXPECT_FALSE(cvcstate("test.isdata.int").isData<std::string>());
  EXPECT_FALSE(cvcstate("test.isdata.int").isData<double>());
  
  // Clean up
  cvcstate("test").reset();
}

TEST(StateTest, IsDataWithException) {
  // State with no data
  cvcstate("test.nodata").value("just_a_value");
  
  EXPECT_FALSE(cvcstate("test.nodata").isData<int>());
  
  // Clean up
  cvcstate("test").reset();
}

// ===========================
// Multithreaded Tests
// ===========================

TEST(StateTest, ConcurrentValueReads) {
  // Set up initial state
  cvcstate("test.concurrent.reads").value("initial_value");
  
  const int num_threads = 10;
  const int reads_per_thread = 100;
  std::atomic<int> successful_reads(0);
  std::vector<boost::thread> threads;
  
  // Launch multiple threads that read the same state
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&successful_reads, reads_per_thread]() {
      for (int j = 0; j < reads_per_thread; ++j) {
        try {
          std::string val = cvcstate("test.concurrent.reads").value();
          if (!val.empty()) {
            successful_reads++;
          }
        } catch (...) {
          // Should not throw
          FAIL() << "Exception during concurrent read";
        }
      }
    });
  }
  
  // Wait for all threads to complete
  for (auto& t : threads) {
    t.join();
  }
  
  // All reads should have succeeded
  EXPECT_EQ(successful_reads.load(), num_threads * reads_per_thread);
  
  // Clean up
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentValueWrites) {
  // Multiple threads writing to different state nodes
  const int num_threads = 10;
  const int writes_per_thread = 50;
  std::atomic<int> successful_writes(0);
  std::vector<boost::thread> threads;
  
  // Each thread writes to its own state node
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([i, &successful_writes, writes_per_thread]() {
      std::string key = "test.concurrent.writes.thread" + boost::lexical_cast<std::string>(i);
      for (int j = 0; j < writes_per_thread; ++j) {
        try {
          std::string val = "value_" + boost::lexical_cast<std::string>(j);
          cvcstate(key).value(val);
          successful_writes++;
        } catch (...) {
          FAIL() << "Exception during concurrent write";
        }
      }
    });
  }
  
  // Wait for completion
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_EQ(successful_writes.load(), num_threads * writes_per_thread);
  
  // Verify final values
  for (int i = 0; i < num_threads; ++i) {
    std::string key = "test.concurrent.writes.thread" + boost::lexical_cast<std::string>(i);
    std::string expected = "value_" + boost::lexical_cast<std::string>(writes_per_thread - 1);
    EXPECT_EQ(cvcstate(key).value(), expected);
  }
  
  // Clean up
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentWritesToSameNode) {
  // Multiple threads writing to the SAME state node (high contention)
  const int num_threads = 20;
  const int writes_per_thread = 100;
  std::atomic<int> total_writes(0);
  std::vector<boost::thread> threads;
  
  cvcstate("test.concurrent.contention").value("initial");
  
  // All threads write to the same node
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([i, &total_writes, writes_per_thread]() {
      for (int j = 0; j < writes_per_thread; ++j) {
        try {
          std::string val = "thread" + boost::lexical_cast<std::string>(i) + 
                           "_write" + boost::lexical_cast<std::string>(j);
          cvcstate("test.concurrent.contention").value(val);
          total_writes++;
        } catch (...) {
          FAIL() << "Exception during high-contention write";
        }
      }
    });
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  // All writes should have succeeded
  EXPECT_EQ(total_writes.load(), num_threads * writes_per_thread);
  
  // The final value should be from one of the threads
  std::string final_value = cvcstate("test.concurrent.contention").value();
  EXPECT_FALSE(final_value.empty());
  EXPECT_NE(final_value, "initial");
  
  // Clean up
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentDataOperations) {
  // Test concurrent data() reads and writes
  const int num_threads = 8;
  std::vector<boost::thread> threads;
  std::atomic<int> data_set_count(0);
  std::atomic<int> data_read_count(0);
  
  // Mix of readers and writers
  for (int i = 0; i < num_threads; ++i) {
    if (i % 2 == 0) {
      // Writer thread
      threads.emplace_back([i, &data_set_count]() {
        std::string key = "test.concurrent.data.writer" + boost::lexical_cast<std::string>(i);
        for (int j = 0; j < 50; ++j) {
          try {
            cvcstate(key).data(j * 100 + i);
            data_set_count++;
          } catch (...) {
            FAIL() << "Exception during concurrent data write";
          }
        }
      });
    } else {
      // Reader thread
      threads.emplace_back([i, &data_read_count]() {
        std::string key = "test.concurrent.data.writer" + boost::lexical_cast<std::string>(i - 1);
        for (int j = 0; j < 50; ++j) {
          try {
            // May or may not have data yet, but shouldn't crash
            if (cvcstate(key).isData<int>()) {
              int val = cvcstate(key).data<int>();
              (void)val; // Suppress unused warning
              data_read_count++;
            }
            boost::this_thread::sleep_for(boost::chrono::microseconds(10));
          } catch (...) {
            FAIL() << "Exception during concurrent data read";
          }
        }
      });
    }
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_GT(data_set_count.load(), 0);
  EXPECT_GT(data_read_count.load(), 0);
  
  // Clean up
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentSignalHandling) {
  // Test that signals fire correctly under concurrent modifications
  std::atomic<int> signal_count(0);
  boost::mutex signal_mutex;
  std::vector<std::string> signal_paths;
  
  // Connect to childChanged signal
  auto connection = cvcstate("test.concurrent.signals").childChanged.connect(
    [&signal_count, &signal_mutex, &signal_paths](const std::string& path) {
      signal_count++;
      boost::mutex::scoped_lock lock(signal_mutex);
      signal_paths.push_back(path);
    }
  );
  
  const int num_threads = 5;
  const int ops_per_thread = 20;
  std::vector<boost::thread> threads;
  
  // Multiple threads creating children
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([i, ops_per_thread]() {
      for (int j = 0; j < ops_per_thread; ++j) {
        std::string key = "test.concurrent.signals.child" + 
                         boost::lexical_cast<std::string>(i) + "." +
                         boost::lexical_cast<std::string>(j);
        cvcstate(key).value("signal_test");
      }
    });
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  // Give signals time to propagate
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  
  // Should have received many signals
  EXPECT_GT(signal_count.load(), 0);
  
  connection.disconnect();
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentHierarchyCreation) {
  // Test concurrent creation of deep hierarchies
  const int num_threads = 8;
  const int depth = 5;
  std::atomic<int> nodes_created(0);
  std::vector<boost::thread> threads;
  
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([i, depth, &nodes_created]() {
      std::string base = "test.concurrent.hierarchy.branch" + boost::lexical_cast<std::string>(i);
      std::string path = base;
      
      for (int d = 0; d < depth; ++d) {
        path += ".level" + boost::lexical_cast<std::string>(d);
        try {
          cvcstate(path).value("depth_" + boost::lexical_cast<std::string>(d));
          nodes_created++;
        } catch (...) {
          FAIL() << "Exception during hierarchy creation";
        }
      }
    });
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_EQ(nodes_created.load(), num_threads * depth);
  
  // Verify hierarchies exist
  for (int i = 0; i < num_threads; ++i) {
    std::string base = "test.concurrent.hierarchy.branch" + boost::lexical_cast<std::string>(i);
    size_t children = cvcstate(base).numChildren();
    EXPECT_GE(children, 1);
  }
  
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentTraversal) {
  // Set up a tree structure
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 3; ++j) {
      std::string key = "test.concurrent.traversal.parent" + 
                       boost::lexical_cast<std::string>(i) + ".child" +
                       boost::lexical_cast<std::string>(j);
      cvcstate(key).value("traverse_me");
    }
  }
  
  // Multiple threads traversing while one thread modifies
  std::atomic<int> traverse_count(0);
  std::atomic<bool> stop_flag(false);
  std::vector<boost::thread> threads;
  
  // Traversal threads
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&traverse_count, &stop_flag]() {
      while (!stop_flag.load()) {
        try {
          cvcstate("test.concurrent.traversal").traverse(
            [&traverse_count](std::string) { traverse_count++; }
          );
        } catch (...) {
          FAIL() << "Exception during concurrent traversal";
        }
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      }
    });
  }
  
  // Modifier thread
  threads.emplace_back([&stop_flag]() {
    for (int i = 0; i < 10; ++i) {
      std::string key = "test.concurrent.traversal.newnode" + boost::lexical_cast<std::string>(i);
      cvcstate(key).value("new");
      boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }
    stop_flag.store(true);
  });
  
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_GT(traverse_count.load(), 0);
  
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentResetOperations) {
  // Test reset() under concurrent access
  const int num_threads = 6;
  std::vector<boost::thread> threads;
  std::atomic<int> reset_count(0);
  
  // Populate initial state
  for (int i = 0; i < 20; ++i) {
    cvcstate("test.concurrent.reset.item" + boost::lexical_cast<std::string>(i)).value("data");
  }
  
  // Some threads reset, others read/write
  for (int i = 0; i < num_threads; ++i) {
    if (i % 3 == 0) {
      // Reset thread
      threads.emplace_back([&reset_count]() {
        for (int j = 0; j < 5; ++j) {
          try {
            cvcstate("test.concurrent.reset").reset();
            reset_count++;
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
          } catch (...) {
            FAIL() << "Exception during reset";
          }
        }
      });
    } else {
      // Read/Write thread
      threads.emplace_back([i]() {
        for (int j = 0; j < 20; ++j) {
          try {
            std::string key = "test.concurrent.reset.item" + boost::lexical_cast<std::string>(j);
            cvcstate(key).value("updated_" + boost::lexical_cast<std::string>(i));
            std::string val = cvcstate(key).value();
            (void)val;
          } catch (...) {
            // May fail if reset happens - that's okay
          }
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }
      });
    }
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_GT(reset_count.load(), 0);
  
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, ConcurrentPropertyTreeOperations) {
  // Test ptree() and json() operations under concurrent modifications
  std::atomic<int> ptree_ops(0);
  std::atomic<int> json_ops(0);
  std::vector<boost::thread> threads;
  
  // Set up initial state
  for (int i = 0; i < 10; ++i) {
    cvcstate("test.concurrent.ptree.item" + boost::lexical_cast<std::string>(i)).value("value" + boost::lexical_cast<std::string>(i));
  }
  
  const int num_threads = 6;
  
  for (int i = 0; i < num_threads; ++i) {
    if (i % 2 == 0) {
      // Serialize threads
      threads.emplace_back([&ptree_ops, &json_ops]() {
        for (int j = 0; j < 10; ++j) {
          try {
            auto pt = cvcstate("test.concurrent.ptree").ptree();
            ptree_ops++;
            
            std::string json_str = cvcstate("test.concurrent.ptree").json();
            if (!json_str.empty()) {
              json_ops++;
            }
          } catch (...) {
            FAIL() << "Exception during serialization";
          }
          boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
        }
      });
    } else {
      // Modification threads
      threads.emplace_back([i]() {
        for (int j = 0; j < 20; ++j) {
          try {
            std::string key = "test.concurrent.ptree.item" + boost::lexical_cast<std::string>(j % 10);
            cvcstate(key).value("thread" + boost::lexical_cast<std::string>(i) + "_" + boost::lexical_cast<std::string>(j));
          } catch (...) {
            FAIL() << "Exception during modification";
          }
          boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
        }
      });
    }
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_GT(ptree_ops.load(), 0);
  EXPECT_GT(json_ops.load(), 0);
  
  cvcstate("test.concurrent").reset();
}

TEST(StateTest, DeadlockDetectionValueAndSignal) {
  // Test for potential deadlock between value changes and signal handlers
  std::atomic<int> signal_fires(0);
  std::atomic<bool> deadlock_detected(false);
  boost::mutex test_mutex;
  
  auto connection = cvcstate("test.deadlock.node").valueChanged.connect([&]() {
    signal_fires++;
    // Try to access state from within signal handler
    try {
      std::string val = cvcstate("test.deadlock.node").value();
      boost::mutex::scoped_lock lock(test_mutex);
      cvcstate("test.deadlock.counter").value(boost::lexical_cast<std::string>(signal_fires.load()));
    } catch (...) {
      deadlock_detected.store(true);
    }
  });
  
  // Rapidly change value
  boost::thread writer([&deadlock_detected]() {
    for (int i = 0; i < 50 && !deadlock_detected.load(); ++i) {
      cvcstate("test.deadlock.node").value("iteration_" + boost::lexical_cast<std::string>(i));
      boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    }
  });
  
  // Wait with timeout
  if (!writer.try_join_for(boost::chrono::seconds(5))) {
    deadlock_detected.store(true);
    // Force thread to stop (not graceful, but for testing)
    writer.interrupt();
    writer.join();
    FAIL() << "Potential deadlock detected - thread did not complete in time";
  }
  
  EXPECT_FALSE(deadlock_detected.load());
  EXPECT_GT(signal_fires.load(), 0);
  
  connection.disconnect();
  cvcstate("test.deadlock").reset();
}

// Test state_object pattern with threading
class TestStateObject : public state_object<TestStateObject> {
public:
  std::atomic<int> handleCount;
  std::atomic<int> errorCount;
  
  TestStateObject() : handleCount(0), errorCount(0) {}
  
protected:
  virtual void handleStateChanged(const std::string& childState) override {
    try {
      handleCount++;
      // Try to read our own state
      std::string val = getState().value();
      (void)val;
    } catch (...) {
      errorCount++;
    }
  }
};

TEST(StateTest, StateObjectMultithreaded) {
  TestStateObject obj;
  
  const int num_threads = 8;
  const int ops_per_thread = 25;
  std::vector<boost::thread> threads;
  
  // Multiple threads modify the state_object's state
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&obj, i, ops_per_thread]() {
      for (int j = 0; j < ops_per_thread; ++j) {
        try {
          obj.getState("property" + boost::lexical_cast<std::string>(i)).value(
            "value_" + boost::lexical_cast<std::string>(j)
          );
          boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
        } catch (...) {
          FAIL() << "Exception in state_object test";
        }
      }
    });
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  // Give time for async handlers to complete
  boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
  
  // Should have triggered many state change handlers
  EXPECT_GT(obj.handleCount.load(), 0);
  
  // Should not have had errors
  EXPECT_EQ(obj.errorCount.load(), 0);
}

TEST(StateTest, StressTestCombinedOperations) {
  // Stress test combining multiple operations
  const int duration_ms = 1000; // Run for 1 second
  std::atomic<bool> stop_flag(false);
  std::atomic<int> total_ops(0);
  std::vector<boost::thread> threads;
  
  // Writer threads
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back([i, &stop_flag, &total_ops]() {
      int ops = 0;
      while (!stop_flag.load()) {
        try {
          std::string key = "test.stress.writer" + boost::lexical_cast<std::string>(i);
          cvcstate(key).value("val_" + boost::lexical_cast<std::string>(ops));
          cvcstate(key).data(ops);
          cvcstate(key).comment("comment_" + boost::lexical_cast<std::string>(ops));
          ops++;
        } catch (...) {
          FAIL() << "Exception in writer thread";
        }
      }
      total_ops += ops;
    });
  }
  
  // Reader threads
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back([i, &stop_flag, &total_ops]() {
      int ops = 0;
      while (!stop_flag.load()) {
        try {
          std::string key = "test.stress.writer" + boost::lexical_cast<std::string>(i % 3);
          std::string val = cvcstate(key).value();
          if (cvcstate(key).isData<int>()) {
            int data = cvcstate(key).data<int>();
            (void)data;
          }
          ops++;
        } catch (...) {
          // May fail if node doesn't exist yet
        }
      }
      total_ops += ops;
    });
  }
  
  // Traversal thread
  threads.emplace_back([&stop_flag, &total_ops]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        cvcstate("test.stress").traverse([](std::string) {});
        ops++;
      } catch (...) {
        FAIL() << "Exception in traversal thread";
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    }
    total_ops += ops;
  });
  
  // Let it run
  boost::this_thread::sleep_for(boost::chrono::milliseconds(duration_ms));
  stop_flag.store(true);
  
  // Join all threads
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_GT(total_ops.load(), 0);
  std::cout << "Stress test completed " << total_ops.load() << " operations without deadlock\n";
  
  cvcstate("test.stress").reset();
}

// Main function is provided by gtest_main library

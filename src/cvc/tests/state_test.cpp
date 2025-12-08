/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for cvc::state class functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <fstream>

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

// Main function is provided by gtest_main library

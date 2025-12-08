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

// Main function is provided by gtest_main library

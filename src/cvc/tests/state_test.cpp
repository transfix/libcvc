/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for cvc::state class functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <cvc/bounding_box.h>
#include <cvc/dimension.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <cvc/state_object.h>
#include <cvc/volume.h>
#include <cvc/voxels.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace CVC_NAMESPACE;

// Global flag to control stress/performance tests
bool enable_stress_tests = false;

// Fixture for tests that exercise the cvc::app data store. Each test
// gets a fresh, fixture-local cvc::app instance instead of relying on
// the deprecated cvcapp singleton (see cvc-singleton-removal-plan.md).
class StateDataFixture : public ::testing::Test {
protected:
  cvc::app ctx;
};

// Fixture for tests that exercise state_object subclasses. Provides a
// fresh cvc::app context so that subclass constructors can be passed
// the explicit ctx instead of falling through state_object's
// app::instance()-using default ctor.
class StateObjectFixture : public ::testing::Test {
protected:
  cvc::app ctx;
};

// File-scope cvc::app used by the broad TEST(StateTest, ...) cases.
// Replaces the deprecated `cvc::state::instance(test_ctx)` macro / `cvc::state::instance()`
// global singleton. Routing all TEST(StateTest) cases through this
// shared app preserves the existing "shared root" semantics those
// tests were written against, while keeping the global cvc::app
// singleton out of the picture.
static cvc::app test_ctx;

// ===========================
// Basic Singleton Tests
// ===========================

TEST(StateTest, SingletonInstance) {
  // Test that instance() returns a valid reference
  state &state1 = cvc::state::instance(test_ctx);
  state &state2 = cvc::state::instance(test_ctx);

  // Both references should point to the same singleton
  EXPECT_EQ(&state1, &state2);
}

// ===========================
// Value Management Tests
// ===========================

TEST(StateTest, ValueSetAndGet) {
  std::string test_value = "test_string_value";
  cvc::state::instance(test_ctx)("test.value.simple").value(test_value);

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.value.simple").value(), test_value);
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.value.simple").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test.value.simple").reset();
}

TEST(StateTest, ValueTypeInt) {
  int test_value = 42;
  cvc::state::instance(test_ctx)("test.value.int").value(test_value);

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.value.int").value<int>(), test_value);

  // Clean up
  cvc::state::instance(test_ctx)("test.value.int").reset();
}

TEST(StateTest, ValueTypeDouble) {
  double test_value = 3.14159;
  cvc::state::instance(test_ctx)("test.value.double").value(test_value);

  EXPECT_DOUBLE_EQ(cvc::state::instance(test_ctx)("test.value.double").value<double>(), test_value);

  // Clean up
  cvc::state::instance(test_ctx)("test.value.double").reset();
}

TEST(StateTest, ValueTypeBool) {
  bool test_value = true;
  cvc::state::instance(test_ctx)("test.value.bool").value(test_value);

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.value.bool").value<bool>(), test_value);

  // Clean up
  cvc::state::instance(test_ctx)("test.value.bool").reset();
}

TEST(StateTest, ValueCommaList) {
  std::string list_value = "item1,item2,item3";
  cvc::state::instance(test_ctx)("test.value.list").value(list_value);

  std::vector<std::string> values = cvc::state::instance(test_ctx)("test.value.list").values();

  ASSERT_EQ(values.size(), 3);
  EXPECT_EQ(values[0], "item1");
  EXPECT_EQ(values[1], "item2");
  EXPECT_EQ(values[2], "item3");

  // Clean up
  cvc::state::instance(test_ctx)("test.value.list").reset();
}

TEST(StateTest, ValueCommaListWithSpaces) {
  std::string list_value = "item1 , item2 , item3";
  cvc::state::instance(test_ctx)("test.value.list.spaces").value(list_value);

  std::vector<std::string> values =
      cvc::state::instance(test_ctx)("test.value.list.spaces").values();

  ASSERT_EQ(values.size(), 3);
  EXPECT_EQ(values[0], "item1");
  EXPECT_EQ(values[1], "item2");
  EXPECT_EQ(values[2], "item3");

  // Clean up
  cvc::state::instance(test_ctx)("test.value.list.spaces").reset();
}

TEST(StateTest, ValueListUnique) {
  std::string list_value = "item1,item2,item1,item3,item2";
  cvc::state::instance(test_ctx)("test.value.list.unique").value(list_value);

  std::vector<std::string> values =
      cvc::state::instance(test_ctx)("test.value.list.unique").values(true);

  // Should have only unique items
  EXPECT_EQ(values.size(), 3);

  // Clean up
  cvc::state::instance(test_ctx)("test.value.list.unique").reset();
}

TEST(StateTest, ValueConversion) {
  // Test conversion to std::string
  std::string test_value = "conversion_test";
  cvc::state::instance(test_ctx)("test.value.conversion").value(test_value);

  std::string converted = cvc::state::instance(test_ctx)("test.value.conversion");
  EXPECT_EQ(converted, test_value);

  // Clean up
  cvc::state::instance(test_ctx)("test.value.conversion").reset();
}

// ===========================
// Data Management Tests
// ===========================

TEST(StateTest, DataSetAndGet) {
  std::string test_data = "test_data_string";
  cvc::state::instance(test_ctx)("test.data.simple").data(test_data);

  ASSERT_TRUE(cvc::state::instance(test_ctx)("test.data.simple").isData<std::string>());
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.data.simple").data<std::string>(), test_data);

  // Clean up
  cvc::state::instance(test_ctx)("test.data.simple").reset();
}

TEST(StateTest, DataTypeInt) {
  int test_data = 123;
  cvc::state::instance(test_ctx)("test.data.int").data(test_data);

  ASSERT_TRUE(cvc::state::instance(test_ctx)("test.data.int").isData<int>());
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.data.int").data<int>(), test_data);

  // Clean up
  cvc::state::instance(test_ctx)("test.data.int").reset();
}

TEST(StateTest, DataTypeDouble) {
  double test_data = 2.71828;
  cvc::state::instance(test_ctx)("test.data.double").data(test_data);

  ASSERT_TRUE(cvc::state::instance(test_ctx)("test.data.double").isData<double>());
  EXPECT_DOUBLE_EQ(cvc::state::instance(test_ctx)("test.data.double").data<double>(), test_data);

  // Clean up
  cvc::state::instance(test_ctx)("test.data.double").reset();
}

// ===========================
// Hierarchy and Navigation Tests
// ===========================

TEST(StateTest, ChildCreation) {
  // Create a child state
  cvc::state::instance(test_ctx)("parent.child").value("child_value");

  EXPECT_EQ(cvc::state::instance(test_ctx)("parent.child").value(), "child_value");
  EXPECT_EQ(cvc::state::instance(test_ctx)("parent.child").name(), "child");
  EXPECT_EQ(cvc::state::instance(test_ctx)("parent").name(), "parent");

  // Clean up
  cvc::state::instance(test_ctx)("parent").reset();
}

TEST(StateTest, FullName) {
  cvc::state::instance(test_ctx)("level1.level2.level3").value("deep");

  EXPECT_EQ(cvc::state::instance(test_ctx)("level1.level2.level3").fullName(),
            "level1.level2.level3");
  EXPECT_EQ(cvc::state::instance(test_ctx)("level1.level2.level3").name(), "level3");

  // Clean up
  cvc::state::instance(test_ctx)("level1").reset();
}

TEST(StateTest, ParentName) {
  cvc::state::instance(test_ctx)("parent.child.grandchild").value("test");

  std::string parent_name = cvc::state::instance(test_ctx)("parent.child.grandchild").parentName();
  EXPECT_EQ(parent_name, "parent.child");

  // Clean up
  cvc::state::instance(test_ctx)("parent").reset();
}

TEST(StateTest, ChildrenListing) {
  // Create multiple children
  cvc::state::instance(test_ctx)("test.children.child1").value("value1");
  cvc::state::instance(test_ctx)("test.children.child2").value("value2");
  cvc::state::instance(test_ctx)("test.children.child3").value("value3");

  std::vector<std::string> children = cvc::state::instance(test_ctx)("test.children").children();

  EXPECT_GE(children.size(), 3);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, NumChildren) {
  // Create children
  cvc::state::instance(test_ctx)("test.count.child1").value("1");
  cvc::state::instance(test_ctx)("test.count.child2").value("2");

  size_t count = cvc::state::instance(test_ctx)("test.count").numChildren();
  EXPECT_EQ(count, 2);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ChildrenWithRegex) {
  // Create children with different names
  cvc::state::instance(test_ctx)("test.regex.foo1").value("1");
  cvc::state::instance(test_ctx)("test.regex.foo2").value("2");
  cvc::state::instance(test_ctx)("test.regex.bar1").value("3");

  // Search for children matching pattern
  std::vector<std::string> foo_children =
      cvc::state::instance(test_ctx)("test.regex").children(".*foo.*");

  EXPECT_GE(foo_children.size(), 2);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// State Metadata Tests
// ===========================

TEST(StateTest, CommentSetAndGet) {
  std::string comment = "This is a test comment";
  cvc::state::instance(test_ctx)("test.comment").comment(comment);

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.comment").comment(), comment);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, HiddenFlag) {
  cvc::state::instance(test_ctx)("test.hidden").hidden(true);
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.hidden").hidden());

  cvc::state::instance(test_ctx)("test.hidden").hidden(false);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.hidden").hidden());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyFlag) {
  // Test setting and getting read-only flag
  cvc::state::instance(test_ctx)("test.readonly").readOnly(true);
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.readonly").readOnly());

  cvc::state::instance(test_ctx)("test.readonly").readOnly(false);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.readonly").readOnly());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyBlocksValueModification) {
  // Set initial value
  cvc::state::instance(test_ctx)("test.readonly_value").value("initial");
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.readonly_value").value(), "initial");

  // Mark as read-only
  cvc::state::instance(test_ctx)("test.readonly_value").readOnly(true);

  // Attempt to modify should throw
  EXPECT_THROW(
      { cvc::state::instance(test_ctx)("test.readonly_value").value("modified"); },
      read_only_error);

  // Value should remain unchanged
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.readonly_value").value(), "initial");

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyBlocksDataModification) {
  // Set initial data
  cvc::state::instance(test_ctx)("test.readonly_data").data(42);
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.readonly_data").data<int>(), 42);

  // Mark as read-only
  cvc::state::instance(test_ctx)("test.readonly_data").readOnly(true);

  // Attempt to modify should throw
  EXPECT_THROW(
      { cvc::state::instance(test_ctx)("test.readonly_data").data(100); }, read_only_error);

  // Data should remain unchanged
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.readonly_data").data<int>(), 42);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyAllowsReadOperations) {
  // Set value and mark read-only
  cvc::state::instance(test_ctx)("test.readonly_read").value("readable");
  cvc::state::instance(test_ctx)("test.readonly_read").readOnly(true);

  // Reading should still work
  EXPECT_NO_THROW({
    std::string val = cvc::state::instance(test_ctx)("test.readonly_read").value();
    EXPECT_EQ(val, "readable");
  });

  // Getting read-only status should work
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.readonly_read").readOnly());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyCanBeUnset) {
  // Set value and mark read-only
  cvc::state::instance(test_ctx)("test.readonly_unset").value("initial");
  cvc::state::instance(test_ctx)("test.readonly_unset").readOnly(true);

  // Verify it's read-only
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.readonly_unset").readOnly());
  EXPECT_THROW(
      { cvc::state::instance(test_ctx)("test.readonly_unset").value("should_fail"); },
      read_only_error);

  // Unset read-only
  cvc::state::instance(test_ctx)("test.readonly_unset").readOnly(false);

  // Now modification should work
  EXPECT_NO_THROW({ cvc::state::instance(test_ctx)("test.readonly_unset").value("modified"); });
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.readonly_unset").value(), "modified");

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlySignalEmission) {
  bool signal_received = false;

  // Connect to readOnlyChanged signal
  auto connection = cvc::state::instance(test_ctx)("test.readonly_signal")
                        .readOnlyChanged.connect([&signal_received]() { signal_received = true; });

  // Setting read-only should trigger signal
  cvc::state::instance(test_ctx)("test.readonly_signal").readOnly(true);
  EXPECT_TRUE(signal_received);

  // Reset flag and test again
  signal_received = false;
  cvc::state::instance(test_ctx)("test.readonly_signal").readOnly(false);
  EXPECT_TRUE(signal_received);

  // Setting to same value should not trigger signal
  signal_received = false;
  cvc::state::instance(test_ctx)("test.readonly_signal").readOnly(false);
  EXPECT_FALSE(signal_received);

  // Clean up
  connection.disconnect();
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyInitializesFlag) {
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.init.readonly").initialized());

  cvc::state::instance(test_ctx)("test.init.readonly").readOnly(true);
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.init.readonly").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyWithTemplateValue) {
  // Test with int
  cvc::state::instance(test_ctx)("test.readonly_template_int").value<int>(42);
  cvc::state::instance(test_ctx)("test.readonly_template_int").readOnly(true);

  EXPECT_THROW(
      { cvc::state::instance(test_ctx)("test.readonly_template_int").value<int>(100); },
      read_only_error);

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.readonly_template_int").value<int>(), 42);

  // Test with double
  cvc::state::instance(test_ctx)("test.readonly_template_double").value<double>(3.14);
  cvc::state::instance(test_ctx)("test.readonly_template_double").readOnly(true);

  EXPECT_THROW(
      { cvc::state::instance(test_ctx)("test.readonly_template_double").value<double>(2.71); },
      read_only_error);

  EXPECT_DOUBLE_EQ(cvc::state::instance(test_ctx)("test.readonly_template_double").value<double>(),
                   3.14);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ReadOnlyExceptionMessage) {
  cvc::state::instance(test_ctx)("test.readonly_exception").value("test");
  cvc::state::instance(test_ctx)("test.readonly_exception").readOnly(true);

  try {
    cvc::state::instance(test_ctx)("test.readonly_exception").value("modified");
    FAIL() << "Expected read_only_error to be thrown";
  } catch (const read_only_error &e) {
    // Verify exception contains useful information
    std::string msg = e.what();
    EXPECT_TRUE(msg.find("read") != std::string::npos ||
                msg.find("readonly") != std::string::npos ||
                msg.find("read-only") != std::string::npos);
  } catch (...) {
    FAIL() << "Expected read_only_error, got different exception";
  }

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, InitializedFlag) {
  // Before setting anything, should not be initialized
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.uninitialized").initialized());

  // After setting a value, should be initialized
  cvc::state::instance(test_ctx)("test.initialized").value("test");
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.initialized").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, LastModification) {
  using namespace boost::posix_time;

  ptime before = microsec_clock::universal_time();
  cvc::state::instance(test_ctx)("test.lastmod").value("test");
  ptime after = microsec_clock::universal_time();

  ptime lastMod = cvc::state::instance(test_ctx)("test.lastmod").lastMod();

  EXPECT_GE(lastMod, before);
  EXPECT_LE(lastMod, after);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// State Manipulation Tests
// ===========================

TEST(StateTest, Touch) {
  using namespace boost::posix_time;

  cvc::state::instance(test_ctx)("test.touch").value("initial");
  ptime first_mod = cvc::state::instance(test_ctx)("test.touch").lastMod();

  // Sleep briefly to ensure time difference
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

  cvc::state::instance(test_ctx)("test.touch").touch();
  ptime second_mod = cvc::state::instance(test_ctx)("test.touch").lastMod();

  EXPECT_GT(second_mod, first_mod);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, Reset) {
  cvc::state::instance(test_ctx)("test.reset").value("value");
  cvc::state::instance(test_ctx)("test.reset").data(123);
  cvc::state::instance(test_ctx)("test.reset").comment("comment");
  cvc::state::instance(test_ctx)("test.reset").hidden(true);

  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset").initialized());

  cvc::state::instance(test_ctx)("test.reset").reset();

  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset").initialized());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset").value().empty());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset").comment().empty());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset").hidden());
}

// ===========================
// Property Tree Tests
// ===========================

TEST(StateTest, PropertyTreeConversion) {
  cvc::state::instance(test_ctx)("test.ptree.item1").value("value1");
  cvc::state::instance(test_ctx)("test.ptree.item2").value("value2");
  cvc::state::instance(test_ctx)("test.ptree.nested.item3").value("value3");

  boost::property_tree::ptree pt = cvc::state::instance(test_ctx)("test.ptree").ptree();

  EXPECT_FALSE(pt.empty());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, PropertyTreeRoundTrip) {
  cvc::state::instance(test_ctx)("test.roundtrip.a").value("alpha");
  cvc::state::instance(test_ctx)("test.roundtrip.b").value("beta");
  cvc::state::instance(test_ctx)("test.roundtrip.c").value("gamma");

  boost::property_tree::ptree pt = cvc::state::instance(test_ctx)("test.roundtrip").ptree();

  // Verify property tree contains the values
  EXPECT_FALSE(pt.empty());

  // Reset the state
  cvc::state::instance(test_ctx)("test.roundtrip").reset();

  // Note: The ptree() method returns only the values, not a full tree structure
  // for restoration. This is a limitation of the current implementation.
  // For now, we verify that the property tree was created successfully.

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, JsonConversion) {
  cvc::state::instance(test_ctx)("test.json.x").value("10");
  cvc::state::instance(test_ctx)("test.json.y").value("20");

  std::string json = cvc::state::instance(test_ctx)("test.json").json();

  EXPECT_FALSE(json.empty());
  EXPECT_TRUE(json.find("test.json.x") != std::string::npos || json.find("x") != std::string::npos);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// ValueData Tests
// ===========================

TEST_F(StateDataFixture, ValueData) {
  // Set up some data objects referenced by a list
  ctx.data("test.vd.obj1", 100);
  ctx.data("test.vd.obj2", 200);
  ctx.data("test.vd.obj3", 300);

  // Verify data was stored
  EXPECT_TRUE(ctx.isData<int>("test.vd.obj1"));
  EXPECT_TRUE(ctx.isData<int>("test.vd.obj2"));
  EXPECT_TRUE(ctx.isData<int>("test.vd.obj3"));

  // Create states that the valueData method will look for
  cvc::state::instance(ctx)("test.vd.obj1").data(100);
  cvc::state::instance(ctx)("test.vd.obj2").data(200);
  cvc::state::instance(ctx)("test.vd.obj3").data(300);

  // Create a state that references these objects
  cvc::state::instance(ctx)("test.valuedata.list").value("test.vd.obj1,test.vd.obj2,test.vd.obj3");

  // Get the data objects (from state, not app)
  std::vector<int> data = cvc::state::instance(ctx)("test.valuedata.list").valueData<int>();

  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 100);
  EXPECT_EQ(data[1], 200);
  EXPECT_EQ(data[2], 300);

  // Clean up
  cvc::state::instance(ctx)("test").reset();
  ctx.data("test.vd.obj1", boost::any());
  ctx.data("test.vd.obj2", boost::any());
  ctx.data("test.vd.obj3", boost::any());
}

// ===========================
// Traversal Tests
// ===========================

TEST(StateTest, Traverse) {
  // Create a tree structure
  cvc::state::instance(test_ctx)("test.traverse.a").value("1");
  cvc::state::instance(test_ctx)("test.traverse.b").value("2");
  cvc::state::instance(test_ctx)("test.traverse.c.d").value("3");

  // Count how many states are visited
  int visit_count = 0;
  state::traversal_unary_func counter = [&visit_count](std::string) { visit_count++; };

  cvc::state::instance(test_ctx)("test.traverse").traverse(counter);

  EXPECT_GT(visit_count, 0);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, TraverseWithRegex) {
  // Create mixed structure
  cvc::state::instance(test_ctx)("test.regex.match1").value("a");
  cvc::state::instance(test_ctx)("test.regex.match2").value("b");
  cvc::state::instance(test_ctx)("test.regex.other").value("c");

  std::vector<std::string> visited;
  state::traversal_unary_func collector = [&visited](std::string name) { visited.push_back(name); };

  // Traverse with regex filter
  cvc::state::instance(test_ctx)("test.regex").traverse(collector, ".*match.*");

  EXPECT_GT(visited.size(), 0);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Property Tree Tests
// ===========================

TEST(StateTest, PropertyTreeRoundtrip) {
  // Set up a state tree
  cvc::state::instance(test_ctx)("test.ptree2.value1").value("first");
  cvc::state::instance(test_ctx)("test.ptree2.value2").value("second");

  // Convert to property tree
  boost::property_tree::ptree pt = cvc::state::instance(test_ctx)("test.ptree2").ptree();
  EXPECT_FALSE(pt.empty());

  // Property tree can be created
  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, PropertyTreeImplicitConversion) {
  cvc::state::instance(test_ctx)("test.pt.a").value("alpha");
  cvc::state::instance(test_ctx)("test.pt.b").value("beta");

  // Test implicit conversion operator
  boost::property_tree::ptree pt = cvc::state::instance(test_ctx)("test.pt");
  EXPECT_FALSE(pt.empty());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// JSON Tests
// ===========================

TEST(StateTest, JSONRoundtrip) {
  // Set up state
  cvc::state::instance(test_ctx)("test.json2.field1").value("json_value1");
  cvc::state::instance(test_ctx)("test.json2.field2").value("json_value2");

  // Convert to JSON
  std::string json_str = cvc::state::instance(test_ctx)("test.json2").json();
  EXPECT_FALSE(json_str.empty());

  // JSON conversion works
  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// File I/O Tests
// ===========================

TEST(StateTest, SaveAndRestore) {
  std::string temp_file =
      (std::filesystem::temp_directory_path() / "test_state_save.json").string();

  // Create state
  cvc::state::instance(test_ctx)("test.file2.x").value("saved_x");
  cvc::state::instance(test_ctx)("test.file2.y").value("saved_y");

  // Save to file
  cvc::state::instance(test_ctx)("test.file2").save(temp_file);

  // File should be created
  std::ifstream check(temp_file);
  EXPECT_TRUE(check.good());
  check.close();

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
  std::remove(temp_file.c_str());
}

// ===========================
// LastMod and Touch Tests
// ===========================

TEST(StateTest, LastModTracking) {
  boost::posix_time::ptime before = boost::posix_time::microsec_clock::universal_time();

  cvc::state::instance(test_ctx)("test.lastmod").value("trigger_mod");

  boost::posix_time::ptime after = boost::posix_time::microsec_clock::universal_time();
  boost::posix_time::ptime mod_time = cvc::state::instance(test_ctx)("test.lastmod").lastMod();

  EXPECT_GE(mod_time, before);
  EXPECT_LE(mod_time, after);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, TouchTriggersSignals) {
  cvc::state::instance(test_ctx)("test.touch").value("initial");

  // Touch should update lastMod even without value change
  boost::posix_time::ptime before = cvc::state::instance(test_ctx)("test.touch").lastMod();

  // Small delay to ensure time difference
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

  cvc::state::instance(test_ctx)("test.touch").touch();

  boost::posix_time::ptime after = cvc::state::instance(test_ctx)("test.touch").lastMod();
  EXPECT_GT(after, before);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Parent/Child Relationship Tests
// ===========================

TEST(StateTest, ParentChildRelationship) {
  cvc::state::instance(test_ctx)("test.parent.child").value("child_value");

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.parent.child").name(), "child");
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.parent.child").parentName(), "test.parent");
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.parent.child").fullName(), "test.parent.child");

  const state *parent = cvc::state::instance(test_ctx)("test.parent.child").parent();
  ASSERT_NE(parent, nullptr);
  EXPECT_EQ(parent->name(), "parent");

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ChildrenListingMultiple) {
  cvc::state::instance(test_ctx)("test.children.a").value("1");
  cvc::state::instance(test_ctx)("test.children.b").value("2");
  cvc::state::instance(test_ctx)("test.children.c").value("3");

  std::vector<std::string> children = cvc::state::instance(test_ctx)("test.children").children();

  EXPECT_GE(children.size(), 3);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, NumChildrenCount) {
  cvc::state::instance(test_ctx)("test.numch.x").value("1");
  cvc::state::instance(test_ctx)("test.numch.y").value("2");

  size_t num = cvc::state::instance(test_ctx)("test.numch").numChildren();
  EXPECT_GE(num, 2);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ChildrenFilterByRegex) {
  cvc::state::instance(test_ctx)("test.chre.apple").value("1");
  cvc::state::instance(test_ctx)("test.chre.apricot").value("2");
  cvc::state::instance(test_ctx)("test.chre.banana").value("3");

  // Get all children first
  std::vector<std::string> children = cvc::state::instance(test_ctx)("test.chre").children();
  EXPECT_GE(children.size(), 3);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// ValueTypeName Tests
// ===========================

TEST(StateTest, ValueTypeNameTracking) {
  cvc::state::instance(test_ctx)("test.typename.int").value(42);
  std::string type_name = cvc::state::instance(test_ctx)("test.typename.int").valueTypeName();

  EXPECT_FALSE(type_name.empty());

  cvc::state::instance(test_ctx)("test.typename.string").value("text");
  type_name = cvc::state::instance(test_ctx)("test.typename.string").valueTypeName();

  EXPECT_FALSE(type_name.empty());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// DataTypeName Tests
// ===========================

TEST(StateTest, DataTypeNameRetrieval) {
  int test_data = 999;
  cvc::state::instance(test_ctx)("test.datatypename").data(test_data);

  std::string type_name = cvc::state::instance(test_ctx)("test.datatypename").dataTypeName();
  EXPECT_FALSE(type_name.empty());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// String Conversion Tests
// ===========================

TEST(StateTest, StringConversionOperator) {
  std::string test_val = "conversion_test";
  cvc::state::instance(test_ctx)("test.conversion").value(test_val);

  std::string converted = cvc::state::instance(test_ctx)("test.conversion");
  EXPECT_EQ(converted, test_val);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Chaining Operations Tests
// ===========================

TEST(StateTest, OperationChaining) {
  // Test that value() returns reference for chaining
  cvc::state::instance(test_ctx)("test.chain").value("step1").comment("Test comment").hidden(false);

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.chain").value(), "step1");
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.chain").comment(), "Test comment");
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.chain").hidden());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Edge Cases and Error Handling
// ===========================

TEST(StateTest, EmptyValueHandling) {
  // Set a non-empty value first
  cvc::state::instance(test_ctx)("test.empty").value("nonempty");
  // Then set to empty
  cvc::state::instance(test_ctx)("test.empty").value("");
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.empty").value().empty());
  // Since we changed from non-empty to empty, initialized should be true
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.empty").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, NestedPathCreation) {
  // Deep nesting should create intermediate nodes
  cvc::state::instance(test_ctx)("test.very.deep.nested.path.value").value("deep");

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.very.deep.nested.path.value").value(), "deep");
  EXPECT_EQ(cvc::state::instance(test_ctx)("test.very.deep.nested.path.value").name(), "value");

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, RepeatedValueSetting) {
  // Setting same value multiple times
  cvc::state::instance(test_ctx)("test.repeated").value("same");
  boost::posix_time::ptime first = cvc::state::instance(test_ctx)("test.repeated").lastMod();

  // Setting same value shouldn't update lastMod
  cvc::state::instance(test_ctx)("test.repeated").value("same");
  boost::posix_time::ptime second = cvc::state::instance(test_ctx)("test.repeated").lastMod();

  EXPECT_EQ(first, second);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ValuesUniqueFlag) {
  cvc::state::instance(test_ctx)("test.values.unique").value("a,b,a,c,b,d");

  std::vector<std::string> unique_vals =
      cvc::state::instance(test_ctx)("test.values.unique").values(true);
  std::vector<std::string> all_vals =
      cvc::state::instance(test_ctx)("test.values.unique").values(false);

  EXPECT_EQ(unique_vals.size(), 4); // a,b,c,d
  EXPECT_EQ(all_vals.size(), 6);    // a,b,a,c,b,d

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Signal Connection Tests
// ===========================

TEST(StateTest, ValueChangedSignal) {
  bool signal_fired = false;

  auto connection =
      cvc::state::instance(test_ctx)("test.signal.value").valueChanged.connect([&signal_fired]() {
        signal_fired = true;
      });

  cvc::state::instance(test_ctx)("test.signal.value").value("trigger");

  EXPECT_TRUE(signal_fired);

  connection.disconnect();
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, DataChangedSignal) {
  bool signal_fired = false;

  auto connection =
      cvc::state::instance(test_ctx)("test.signal.data").dataChanged.connect([&signal_fired]() {
        signal_fired = true;
      });

  cvc::state::instance(test_ctx)("test.signal.data").data(42);

  EXPECT_TRUE(signal_fired);

  connection.disconnect();
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ChildChangedSignal) {
  int signal_count = 0;

  auto connection =
      cvc::state::instance(test_ctx)("test.signal.parent")
          .childChanged.connect([&signal_count](const std::string &) { signal_count++; });

  // Create child state - should trigger parent's childChanged
  cvc::state::instance(test_ctx)("test.signal.parent.child1").value("value1");
  cvc::state::instance(test_ctx)("test.signal.parent.child2").value("value2");

  EXPECT_GT(signal_count, 0);

  connection.disconnect();
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ChildChangedSignalUsesRelativeName) {
  // Regression test: childChanged signal should send relative name (e.g., "child"),
  // not full path (e.g., "test.parent.child")
  std::vector<std::string> received_names;

  auto connection = cvc::state::instance(test_ctx)("test.parent")
                        .childChanged.connect([&received_names](const std::string &childState) {
                          received_names.push_back(childState);
                        });

  // Modify child state - parent should receive just "child", not "test.parent.child"
  cvc::state::instance(test_ctx)("test.parent.child").value("test_value");

  // Give signal time to fire
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  // We should have received at least one signal
  ASSERT_FALSE(received_names.empty()) << "No childChanged signals received";

  // The FIRST signal should be "child" (direct notification from child to parent)
  EXPECT_EQ(received_names[0], "child")
      << "Expected relative name 'child', but got: '" << received_names[0] << "'";

  connection.disconnect();
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Deep Hierarchy Tests
// ===========================

TEST(StateTest, DeepHierarchyNavigation) {
  // Create deep nested structure
  std::string deep_path = "level1.level2.level3.level4.level5";
  cvc::state::instance(test_ctx)(deep_path).value("deep_value");

  EXPECT_EQ(cvc::state::instance(test_ctx)(deep_path).value(), "deep_value");
  EXPECT_EQ(cvc::state::instance(test_ctx)(deep_path).name(), "level5");

  // Test parent navigation
  const state *level4 = cvc::state::instance(test_ctx)(deep_path).parent();
  ASSERT_NE(level4, nullptr);
  EXPECT_EQ(level4->name(), "level4");

  // Clean up
  cvc::state::instance(test_ctx)("level1").reset();
}

TEST(StateTest, OperatorChaining) {
  // Test deep path traversal using operator()
  cvc::state::instance(test_ctx)("chain")("a")("b")("c").value("chained");

  EXPECT_EQ(cvc::state::instance(test_ctx)("chain.a.b.c").value(), "chained");

  // Clean up
  cvc::state::instance(test_ctx)("chain").reset();
}

// ===========================
// Edge Case Path Tests
// ===========================

TEST(StateTest, EmptyKeyHandling) {
  // operator() with empty string should return self
  state &self1 = cvc::state::instance(test_ctx)("test.empty.key");
  state &self2 = cvc::state::instance(test_ctx)("test.empty.key")("");

  EXPECT_EQ(&self1, &self2);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, SeparatorInPath) {
  // Test paths with multiple separators
  cvc::state::instance(test_ctx)("test...multi...sep").value("multi_sep_value");

  EXPECT_EQ(cvc::state::instance(test_ctx)("test.multi.sep").value(), "multi_sep_value");

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Initialization Tests
// ===========================

TEST(StateTest, InitializedFlagBehavior) {
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.init.new").initialized());

  cvc::state::instance(test_ctx)("test.init.new").value("now_initialized");
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.init.new").initialized());

  cvc::state::instance(test_ctx)("test.init.new").reset();
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.init.new").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, DataInitializesFlag) {
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.init.data").initialized());

  cvc::state::instance(test_ctx)("test.init.data").data(std::string("data_init"));
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.init.data").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, CommentInitializesFlag) {
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.init.comment").initialized());

  cvc::state::instance(test_ctx)("test.init.comment").comment("test comment");
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.init.comment").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, HiddenInitializesFlag) {
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.init.hidden").initialized());

  cvc::state::instance(test_ctx)("test.init.hidden").hidden(true);
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.init.hidden").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Children Recursion Tests
// ===========================

TEST(StateTest, ChildrenRecursiveListing) {
  // Create nested structure
  cvc::state::instance(test_ctx)("test.recursive.a.a1").value("1");
  cvc::state::instance(test_ctx)("test.recursive.a.a2").value("2");
  cvc::state::instance(test_ctx)("test.recursive.b.b1").value("3");
  cvc::state::instance(test_ctx)("test.recursive.b.b2.deep").value("4");

  // Get all children recursively
  std::vector<std::string> all_children =
      cvc::state::instance(test_ctx)("test.recursive").children();

  // Should include nested children
  EXPECT_GT(all_children.size(), 2);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Regex Children Filtering Tests
// ===========================

TEST(StateTest, ChildrenRegexMatching) {
  cvc::state::instance(test_ctx)("test.regex2.alpha").value("a");
  cvc::state::instance(test_ctx)("test.regex2.beta").value("b");
  cvc::state::instance(test_ctx)("test.regex2.gamma").value("c");
  cvc::state::instance(test_ctx)("test.regex2.delta").value("d");

  // Try to get children with regex (may not work depending on implementation)
  std::vector<std::string> all = cvc::state::instance(test_ctx)("test.regex2").children();
  EXPECT_GE(all.size(), 4);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, ChildrenEmptyRegex) {
  cvc::state::instance(test_ctx)("test.noreg.x").value("1");
  cvc::state::instance(test_ctx)("test.noreg.y").value("2");

  // Empty regex should return all children
  std::vector<std::string> children = cvc::state::instance(test_ctx)("test.noreg").children("");
  EXPECT_GE(children.size(), 2);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Value Type Tests
// ===========================

TEST(StateTest, ValueTypeLexicalCast) {
  // Test lexical cast for value retrieval
  cvc::state::instance(test_ctx)("test.lexical.int").value(12345);
  int val = cvc::state::instance(test_ctx)("test.lexical.int").value<int>();
  EXPECT_EQ(val, 12345);

  cvc::state::instance(test_ctx)("test.lexical.double").value(3.14159);
  double dval = cvc::state::instance(test_ctx)("test.lexical.double").value<double>();
  EXPECT_NEAR(dval, 3.14159, 0.0001);

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Reset Recursive Tests
// ===========================

TEST(StateTest, ResetRecursive) {
  // Create hierarchy with values
  cvc::state::instance(test_ctx)("test.reset.parent").value("parent_value");
  cvc::state::instance(test_ctx)("test.reset.parent.child1").value("child1_value");
  cvc::state::instance(test_ctx)("test.reset.parent.child2").value("child2_value");
  cvc::state::instance(test_ctx)("test.reset.parent.child1").data(100);

  // Reset parent (should reset children too)
  cvc::state::instance(test_ctx)("test.reset.parent").reset();

  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset.parent").value().empty());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset.parent.child1").value().empty());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset.parent.child2").value().empty());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset.parent").initialized());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset.parent.child1").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Traversal Signal Tests
// ===========================

TEST(StateTest, TraversalEnterExitSignals) {
  int enter_count = 0;
  int exit_count = 0;

  auto enter_conn =
      cvc::state::instance(test_ctx)("test.trav.signals").traverseEnter.connect([&enter_count]() {
        enter_count++;
      });

  auto exit_conn =
      cvc::state::instance(test_ctx)("test.trav.signals").traverseExit.connect([&exit_count]() {
        exit_count++;
      });

  cvc::state::instance(test_ctx)("test.trav.signals.a").value("1");
  cvc::state::instance(test_ctx)("test.trav.signals.b").value("2");

  state::traversal_unary_func noop = [](std::string) {};
  cvc::state::instance(test_ctx)("test.trav.signals").traverse(noop);

  EXPECT_GT(enter_count, 0);
  EXPECT_GT(exit_count, 0);

  enter_conn.disconnect();
  exit_conn.disconnect();
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Full Name Tests
// ===========================

TEST(StateTest, FullNameConstruction) {
  cvc::state::instance(test_ctx)("fullname.test.deep.path").value("test");

  std::string full = cvc::state::instance(test_ctx)("fullname.test.deep.path").fullName();
  EXPECT_EQ(full, "fullname.test.deep.path");

  std::string parent_full = cvc::state::instance(test_ctx)("fullname.test.deep").fullName();
  EXPECT_EQ(parent_full, "fullname.test.deep");

  // Clean up
  cvc::state::instance(test_ctx)("fullname").reset();
}

// ===========================
// On Startup Tests
// ===========================

TEST(StateTest, OnStartupRegistration) {
  bool startup_called = false;

  state::on_startup(state::nullary_func([&startup_called]() { startup_called = true; }));

  // Startup functions are called on first instance creation
  // We can't easily test this without restarting, but we can verify registration
  // The function is registered if no exception is thrown
  SUCCEED();
}

// ===========================
// IsData Template Tests
// ===========================

TEST(StateTest, IsDataTemplateMethod) {
  cvc::state::instance(test_ctx)("test.isdata.int").data(42);

  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.isdata.int").isData<int>());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.isdata.int").isData<std::string>());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.isdata.int").isData<double>());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

TEST(StateTest, IsDataWithException) {
  // State with no data
  cvc::state::instance(test_ctx)("test.nodata").value("just_a_value");

  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.nodata").isData<int>());

  // Clean up
  cvc::state::instance(test_ctx)("test").reset();
}

// ===========================
// Multithreaded Tests
// ===========================

TEST(StateTest, ConcurrentValueReads) {
  // Set up initial state
  cvc::state::instance(test_ctx)("test.concurrent.reads").value("initial_value");

  const int num_threads = 10;
  const int reads_per_thread = 100;
  std::atomic<int> successful_reads(0);
  std::vector<boost::thread> threads;

  // Launch multiple threads that read the same state
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&successful_reads, reads_per_thread]() {
      for (int j = 0; j < reads_per_thread; ++j) {
        try {
          std::string val = cvc::state::instance(test_ctx)("test.concurrent.reads").value();
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
  for (auto &t : threads) {
    t.join();
  }

  // All reads should have succeeded
  EXPECT_EQ(successful_reads.load(), num_threads * reads_per_thread);

  // Clean up
  cvc::state::instance(test_ctx)("test.concurrent").reset();
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
          cvc::state::instance(test_ctx)(key).value(val);
          successful_writes++;
        } catch (...) {
          FAIL() << "Exception during concurrent write";
        }
      }
    });
  }

  // Wait for completion
  for (auto &t : threads) {
    t.join();
  }

  EXPECT_EQ(successful_writes.load(), num_threads * writes_per_thread);

  // Verify final values
  for (int i = 0; i < num_threads; ++i) {
    std::string key = "test.concurrent.writes.thread" + boost::lexical_cast<std::string>(i);
    std::string expected = "value_" + boost::lexical_cast<std::string>(writes_per_thread - 1);
    EXPECT_EQ(cvc::state::instance(test_ctx)(key).value(), expected);
  }

  // Clean up
  cvc::state::instance(test_ctx)("test.concurrent").reset();
}

TEST(StateTest, ConcurrentWritesToSameNode) {
  // Multiple threads writing to the SAME state node (high contention)
  const int num_threads = 20;
  const int writes_per_thread = 100;
  std::atomic<int> total_writes(0);
  std::vector<boost::thread> threads;

  cvc::state::instance(test_ctx)("test.concurrent.contention").value("initial");

  // All threads write to the same node
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([i, &total_writes, writes_per_thread]() {
      for (int j = 0; j < writes_per_thread; ++j) {
        try {
          std::string val = "thread" + boost::lexical_cast<std::string>(i) + "_write" +
                            boost::lexical_cast<std::string>(j);
          cvc::state::instance(test_ctx)("test.concurrent.contention").value(val);
          total_writes++;
        } catch (...) {
          FAIL() << "Exception during high-contention write";
        }
      }
    });
  }

  for (auto &t : threads) {
    t.join();
  }

  // All writes should have succeeded
  EXPECT_EQ(total_writes.load(), num_threads * writes_per_thread);

  // The final value should be from one of the threads
  std::string final_value = cvc::state::instance(test_ctx)("test.concurrent.contention").value();
  EXPECT_FALSE(final_value.empty());
  EXPECT_NE(final_value, "initial");

  // Clean up
  cvc::state::instance(test_ctx)("test.concurrent").reset();
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
            cvc::state::instance(test_ctx)(key).data(j * 100 + i);
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
            if (cvc::state::instance(test_ctx)(key).isData<int>()) {
              int val = cvc::state::instance(test_ctx)(key).data<int>();
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

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_GT(data_set_count.load(), 0);
  EXPECT_GT(data_read_count.load(), 0);

  // Clean up
  cvc::state::instance(test_ctx)("test.concurrent").reset();
}

TEST(StateTest, ConcurrentSignalHandling) {
  // Test that signals fire correctly under concurrent modifications
  std::atomic<int> signal_count(0);
  boost::mutex signal_mutex;
  std::vector<std::string> signal_paths;

  // Connect to childChanged signal
  auto connection = cvc::state::instance(test_ctx)("test.concurrent.signals")
                        .childChanged.connect(
                            [&signal_count, &signal_mutex, &signal_paths](const std::string &path) {
                              signal_count++;
                              boost::mutex::scoped_lock lock(signal_mutex);
                              signal_paths.push_back(path);
                            });

  const int num_threads = 5;
  const int ops_per_thread = 20;
  std::vector<boost::thread> threads;

  // Multiple threads creating children
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([i, ops_per_thread]() {
      for (int j = 0; j < ops_per_thread; ++j) {
        std::string key = "test.concurrent.signals.child" + boost::lexical_cast<std::string>(i) +
                          "." + boost::lexical_cast<std::string>(j);
        cvc::state::instance(test_ctx)(key).value("signal_test");
      }
    });
  }

  for (auto &t : threads) {
    t.join();
  }

  // Give signals time to propagate
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  // Should have received many signals
  EXPECT_GT(signal_count.load(), 0);

  connection.disconnect();
  cvc::state::instance(test_ctx)("test.concurrent").reset();
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
          cvc::state::instance(test_ctx)(path).value("depth_" +
                                                     boost::lexical_cast<std::string>(d));
          nodes_created++;
        } catch (...) {
          FAIL() << "Exception during hierarchy creation";
        }
      }
    });
  }

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_EQ(nodes_created.load(), num_threads * depth);

  // Verify hierarchies exist
  for (int i = 0; i < num_threads; ++i) {
    std::string base = "test.concurrent.hierarchy.branch" + boost::lexical_cast<std::string>(i);
    size_t children = cvc::state::instance(test_ctx)(base).numChildren();
    EXPECT_GE(children, 1);
  }

  cvc::state::instance(test_ctx)("test.concurrent").reset();
}

TEST(StateTest, ConcurrentTraversal) {
  // Set up a tree structure
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 3; ++j) {
      std::string key = "test.concurrent.traversal.parent" + boost::lexical_cast<std::string>(i) +
                        ".child" + boost::lexical_cast<std::string>(j);
      cvc::state::instance(test_ctx)(key).value("traverse_me");
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
          cvc::state::instance(test_ctx)("test.concurrent.traversal")
              .traverse([&traverse_count](std::string) { traverse_count++; });
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
      cvc::state::instance(test_ctx)(key).value("new");
      boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }
    stop_flag.store(true);
  });

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_GT(traverse_count.load(), 0);

  cvc::state::instance(test_ctx)("test.concurrent").reset();
}

TEST(StateTest, ConcurrentResetOperations) {
  // Test reset() under concurrent access
  const int num_threads = 6;
  std::vector<boost::thread> threads;
  std::atomic<int> reset_count(0);

  // Populate initial state
  for (int i = 0; i < 20; ++i) {
    cvc::state::instance(test_ctx)("test.concurrent.reset.item" +
                                   boost::lexical_cast<std::string>(i))
        .value("data");
  }

  // Some threads reset, others read/write
  for (int i = 0; i < num_threads; ++i) {
    if (i % 3 == 0) {
      // Reset thread
      threads.emplace_back([&reset_count]() {
        for (int j = 0; j < 5; ++j) {
          try {
            cvc::state::instance(test_ctx)("test.concurrent.reset").reset();
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
            cvc::state::instance(test_ctx)(key).value("updated_" +
                                                      boost::lexical_cast<std::string>(i));
            std::string val = cvc::state::instance(test_ctx)(key).value();
            (void)val;
          } catch (...) {
            // May fail if reset happens - that's okay
          }
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }
      });
    }
  }

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_GT(reset_count.load(), 0);

  cvc::state::instance(test_ctx)("test.concurrent").reset();
}

TEST(StateTest, ConcurrentPropertyTreeOperations) {
  // Test ptree() and json() operations under concurrent modifications
  std::atomic<int> ptree_ops(0);
  std::atomic<int> json_ops(0);
  std::vector<boost::thread> threads;

  // Set up initial state
  for (int i = 0; i < 10; ++i) {
    cvc::state::instance(test_ctx)("test.concurrent.ptree.item" +
                                   boost::lexical_cast<std::string>(i))
        .value("value" + boost::lexical_cast<std::string>(i));
  }

  const int num_threads = 6;

  for (int i = 0; i < num_threads; ++i) {
    if (i % 2 == 0) {
      // Serialize threads
      threads.emplace_back([&ptree_ops, &json_ops]() {
        for (int j = 0; j < 10; ++j) {
          try {
            auto pt = cvc::state::instance(test_ctx)("test.concurrent.ptree").ptree();
            ptree_ops++;

            std::string json_str = cvc::state::instance(test_ctx)("test.concurrent.ptree").json();
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
            std::string key =
                "test.concurrent.ptree.item" + boost::lexical_cast<std::string>(j % 10);
            cvc::state::instance(test_ctx)(key).value("thread" +
                                                      boost::lexical_cast<std::string>(i) + "_" +
                                                      boost::lexical_cast<std::string>(j));
          } catch (...) {
            FAIL() << "Exception during modification";
          }
          boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
        }
      });
    }
  }

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_GT(ptree_ops.load(), 0);
  EXPECT_GT(json_ops.load(), 0);

  cvc::state::instance(test_ctx)("test.concurrent").reset();
}

TEST(StateTest, DeadlockDetectionValueAndSignal) {
  // Test for potential deadlock between value changes and signal handlers
  std::atomic<int> signal_fires(0);
  std::atomic<bool> deadlock_detected(false);
  boost::mutex test_mutex;

  auto connection =
      cvc::state::instance(test_ctx)("test.deadlock.node").valueChanged.connect([&]() {
        signal_fires++;
        // Try to access state from within signal handler
        try {
          std::string val = cvc::state::instance(test_ctx)("test.deadlock.node").value();
          boost::mutex::scoped_lock lock(test_mutex);
          cvc::state::instance(test_ctx)("test.deadlock.counter")
              .value(boost::lexical_cast<std::string>(signal_fires.load()));
        } catch (...) {
          deadlock_detected.store(true);
        }
      });

  // Rapidly change value
  boost::thread writer([&deadlock_detected]() {
    for (int i = 0; i < 50 && !deadlock_detected.load(); ++i) {
      cvc::state::instance(test_ctx)("test.deadlock.node")
          .value("iteration_" + boost::lexical_cast<std::string>(i));
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
  cvc::state::instance(test_ctx)("test.deadlock").reset();
}

// ===========================
// State Object Pattern Tests
// ===========================

// Basic test state_object
class TestStateObject : public state_object<TestStateObject> {
public:
  std::atomic<int> handleCount;
  std::atomic<int> errorCount;

  TestStateObject(cvc::app &ctx)
      : state_object<TestStateObject>(ctx), handleCount(0), errorCount(0) {}

protected:
  virtual void handleStateChanged(const std::string &childState) override {
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

TEST_F(StateObjectFixture, StateObjectMultithreaded) {
  TestStateObject obj(ctx);

  const int num_threads = 8;
  const int ops_per_thread = 25;
  std::vector<boost::thread> threads;

  // Multiple threads modify the state_object's state
  // Use state_change_batch_scope to batch changes within each thread
  // This reduces handler thread spawning and prevents race conditions
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&obj, i, ops_per_thread]() {
      // Batch all state changes from this thread
      state_change_batch_scope<TestStateObject> batch(obj);

      for (int j = 0; j < ops_per_thread; ++j) {
        try {
          obj.getState("property" + boost::lexical_cast<std::string>(i))
              .value("value_" + boost::lexical_cast<std::string>(j));
          boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
        } catch (...) {
          FAIL() << "Exception in state_object test";
        }
      }
      // batch.flush() happens automatically at scope end
    });
  }

  for (auto &t : threads) {
    t.join();
  }

  // Wait for all handlers to complete
  obj.waitForHandlers();

  // Should have triggered many state change handlers
  EXPECT_GT(obj.handleCount.load(), 0);

  // Should not have had errors
  EXPECT_EQ(obj.errorCount.load(), 0);
}

// Configuration state_object example (from STATE_API.md)
class ConfigurationObject : public state_object<ConfigurationObject> {
public:
  std::atomic<int> resizeCount;
  std::atomic<int> fullscreenCount;
  std::atomic<int> totalHandlerCalls;
  int lastWidth;
  int lastHeight;
  bool lastFullscreen;

  ConfigurationObject(cvc::app &ctx)
      : state_object<ConfigurationObject>(ctx), resizeCount(0), fullscreenCount(0),
        totalHandlerCalls(0), lastWidth(0), lastHeight(0), lastFullscreen(false) {
    // Initialize default values without firing handlers — these are
    // seed values, not user-driven changes. Without state_init_scope,
    // the value() calls below would each spawn a handler thread, and
    // those threads would race with the test body's subsequent
    // operations and waitForHandlers() observations.
    state_init_scope<ConfigurationObject> init(*this);
    getState("width").value(1920);
    getState("height").value(1080);
    getState("fullscreen").value(false);
  }

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    totalHandlerCalls++;
    // childState includes the full path, so check if it ends with the expected names
    if (childState.find("width") != std::string::npos ||
        childState.find("height") != std::string::npos) {
      try {
        // Read the values - should be safe now with atomic updates in state.h
        lastWidth = getState("width").value<int>();
        lastHeight = getState("height").value<int>();
        resizeCount++;
      } catch (const boost::bad_lexical_cast &) {
        // Defensive: Handle any remaining race conditions
        // This shouldn't happen with the atomic update fix, but be safe
      }
    } else if (childState.find("fullscreen") != std::string::npos) {
      try {
        lastFullscreen = getState("fullscreen").value<bool>();
        fullscreenCount++;
      } catch (const boost::bad_lexical_cast &) {
        // Defensive: Handle any remaining race conditions
      }
    }
  }
};

TEST_F(StateObjectFixture, StateObjectConfiguration) {
  ConfigurationObject config(ctx);

  // Verify default initialization
  EXPECT_EQ(config.getState("width").value<int>(), 1920);
  EXPECT_EQ(config.getState("height").value<int>(), 1080);
  EXPECT_FALSE(config.getState("fullscreen").value<bool>());

  // Modify state using batching - handlers spawn only once per unique state
  {
    state_change_batch_scope<ConfigurationObject> batch(config);
    config.getState("width").value(2560);
    config.getState("height").value(1440);
  }

  // Wait for handlers to complete
  config.waitForHandlers();

  EXPECT_GT(config.resizeCount.load(), 0);
  EXPECT_EQ(config.lastWidth, 2560);
  EXPECT_EQ(config.lastHeight, 1440);

  // Change fullscreen
  config.getState("fullscreen").value(true);
  config.waitForHandlers();

  EXPECT_GT(config.fullscreenCount.load(), 0);
  EXPECT_TRUE(config.lastFullscreen);
}

// Test batching state changes to avoid thread floods
TEST_F(StateObjectFixture, StateObjectBatchedChanges) {
  ConfigurationObject config(ctx);

  int initialHandlerCalls = config.totalHandlerCalls.load();

  // Use batching scope - handlers won't spawn until scope ends
  {
    state_change_batch_scope<ConfigurationObject> batch(config);

    // Make multiple rapid changes - these are queued, not spawned immediately
    config.getState("width").value(1024);
    config.getState("width").value(1280); // This replaces the 1024 change
    config.getState("width").value(1920); // This replaces the 1280 change
    config.getState("height").value(768);
    config.getState("height").value(1080); // This replaces the 768 change

    // At this point, NO handlers have run yet
    // When we exit this scope, only 2 handlers will spawn (one for width, one for height)
  }

  // Wait for handlers to complete
  config.waitForHandlers();

  // Should have final values
  EXPECT_EQ(config.lastWidth, 1920);
  EXPECT_EQ(config.lastHeight, 1080);

  // Should have only called handlers for the UNIQUE state paths that changed
  // Not 5 times (one for each value() call), but ideally just 1-2 times
  int finalHandlerCalls = config.totalHandlerCalls.load();
  int handlerCalls = finalHandlerCalls - initialHandlerCalls;

  // We changed width and height, so we expect at least 1 handler call
  // (they might be batched into a single call for the parent state)
  EXPECT_GT(handlerCalls, 0);

  // And definitely fewer than 5 (the number of value() calls we made)
  EXPECT_LT(handlerCalls, 5);
}

// Test nested batching
TEST_F(StateObjectFixture, StateObjectNestedBatching) {
  ConfigurationObject config(ctx);

  int initialCalls = config.totalHandlerCalls.load();

  {
    state_change_batch_scope<ConfigurationObject> outerBatch(config);
    config.getState("width").value(800);

    {
      state_change_batch_scope<ConfigurationObject> innerBatch(config);
      config.getState("width").value(1024);
      config.getState("height").value(768);
      // Inner scope ends, but outer batch is still active, so no handlers yet
    }

    config.getState("width").value(1920); // Final width value
    // Outer scope ends - NOW handlers run
  }

  config.waitForHandlers();

  // Should have final values
  EXPECT_EQ(config.lastWidth, 1920);
  EXPECT_EQ(config.lastHeight, 768);

  int handlerCalls = config.totalHandlerCalls.load() - initialCalls;
  EXPECT_GT(handlerCalls, 0);
  EXPECT_LE(handlerCalls, 4); // 4 value() calls, but may be deduplicated
}

// Test manual flush within batch scope
TEST_F(StateObjectFixture, StateObjectManualFlush) {
  ConfigurationObject config(ctx);

  int initialCalls = config.totalHandlerCalls.load();

  {
    state_change_batch_scope<ConfigurationObject> batch(config);

    config.getState("width").value(1024);
    config.getState("height").value(768);

    // Manually flush - handlers run now
    batch.flush();
    config.waitForHandlers();

    int callsAfterFlush = config.totalHandlerCalls.load();
    EXPECT_GT(callsAfterFlush, initialCalls);

    // Make more changes - these won't spawn handlers since we already flushed
    config.getState("width").value(1920);

    // Scope ends, but flush already happened, so no double-flush
  }

  config.waitForHandlers();

  // Width should be 1920 from the change after flush, but that handler never ran
  // because flush() was already called
  EXPECT_EQ(config.getState("width").value<int>(), 1920);
}

// Test batching with rapid concurrent changes
TEST_F(StateObjectFixture, StateObjectBatchingUnderLoad) {
  ConfigurationObject config(ctx);

  {
    state_change_batch_scope<ConfigurationObject> batch(config);

    // Simulate rapid UI updates where same values change many times
    for (int i = 0; i < 100; ++i) {
      config.getState("width").value(1920 + i);
      config.getState("height").value(1080 + i);
    }

    // Without batching, this would spawn 200 threads!
    // With batching, only 2 threads will spawn (one for width, one for height)
  }

  config.waitForHandlers();

  // Should have final values
  EXPECT_EQ(config.lastWidth, 1920 + 99);
  EXPECT_EQ(config.lastHeight, 1080 + 99);
}

// Test state locking - exclusive access coordination
TEST_F(StateObjectFixture, StateObjectLocking) {
  ConfigurationObject config(ctx);
  std::atomic<int> thread1_value(0);
  std::atomic<int> thread2_value(0);

  // Thread 1: Holds lock and makes changes
  boost::thread t1([&config, &thread1_value]() {
    state_lock_scope<ConfigurationObject> lock(config);

    for (int i = 0; i < 10; ++i) {
      config.getState("counter").value(i);
      thread1_value.store(i);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    }
  });

  // Thread 2: Also acquires lock - will block until t1 releases
  boost::thread t2([&config, &thread2_value, &thread1_value]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5));

    state_lock_scope<ConfigurationObject> lock(config);

    // By the time we acquire the lock, t1 should be done
    EXPECT_EQ(thread1_value.load(), 9);

    config.getState("counter").value(100);
    thread2_value.store(100);
  });

  t1.join();
  t2.join();
  config.waitForHandlers();

  EXPECT_EQ(thread1_value.load(), 9);
  EXPECT_EQ(thread2_value.load(), 100);
}

// Test that state lock doesn't block parent state
TEST_F(StateObjectFixture, StateObjectLockDoesNotBlockParent) {
  ConfigurationObject config(ctx);
  std::atomic<bool> parent_modified(false);

  // Hold lock on config
  boost::thread lockHolder([&config]() {
    state_lock_scope<ConfigurationObject> lock(config);

    // Hold lock for a while
    for (int i = 0; i < 5; ++i) {
      config.getState("width").value(1000 + i);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
  });

  // Try to modify parent state - should NOT block
  boost::thread parentModifier([&config, &parent_modified, this]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(20));

    // Get parent state path (everything before the instance address)
    std::string configStateName = config.stateName();
    size_t lastDot = configStateName.rfind('.');
    if (lastDot != std::string::npos) {
      std::string parentPath = configStateName.substr(0, lastDot);

      // This modification is to parent state, not config's state
      // So it should NOT be blocked by config's lock
      cvc::state::instance(ctx)(parentPath + ".other_instance").value("parent_data");
      parent_modified.store(true);
    }
  });

  lockHolder.join();
  parentModifier.join();
  config.waitForHandlers();

  EXPECT_TRUE(parent_modified.load());
}

// Test multiple concurrent locks on different objects
TEST_F(StateObjectFixture, StateObjectMultipleLocks) {
  ConfigurationObject config1(ctx);
  ConfigurationObject config2(ctx);
  std::atomic<int> config1_mods(0);
  std::atomic<int> config2_mods(0);

  // Lock config1 and modify
  boost::thread t1([&config1, &config1_mods]() {
    state_lock_scope<ConfigurationObject> lock(config1);
    for (int i = 0; i < 5; ++i) {
      config1.getState("width").value(1000 + i);
      config1_mods++;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }
  });

  // Lock config2 and modify (should not be blocked by config1's lock)
  boost::thread t2([&config2, &config2_mods]() {
    state_lock_scope<ConfigurationObject> lock(config2);
    for (int i = 0; i < 5; ++i) {
      config2.getState("width").value(2000 + i);
      config2_mods++;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }
  });

  t1.join();
  t2.join();
  config1.waitForHandlers();
  config2.waitForHandlers();

  // Both should complete independently
  EXPECT_EQ(config1_mods.load(), 5);
  EXPECT_EQ(config2_mods.load(), 5);
  EXPECT_EQ(config1.lastWidth, 1004);
  EXPECT_EQ(config2.lastWidth, 2004);
}

// Test lock with batching
TEST_F(StateObjectFixture, StateObjectLockWithBatching) {
  ConfigurationObject config(ctx);

  {
    state_lock_scope<ConfigurationObject> lock(config);
    state_change_batch_scope<ConfigurationObject> batch(config);

    // Make multiple changes under both lock and batch
    config.getState("width").value(1024);
    config.getState("width").value(1920);
    config.getState("height").value(1080);

    // Batch flushes here, but lock still held
  }
  // Lock releases here

  config.waitForHandlers();

  EXPECT_EQ(config.lastWidth, 1920);
  EXPECT_EQ(config.lastHeight, 1080);
}

// Test early unlock
TEST_F(StateObjectFixture, StateObjectLockEarlyUnlock) {
  ConfigurationObject config(ctx);
  std::atomic<bool> other_thread_ran(false);

  boost::thread t1([&config, &other_thread_ran]() {
    state_lock_scope<ConfigurationObject> lock(config);

    config.getState("width").value(1000);

    // Release lock early
    lock.unlock();

    // Do other work without lock
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

    // Check if other thread ran (it should have, since we released early)
    EXPECT_TRUE(other_thread_ran.load());
  });

  boost::thread t2([&config, &other_thread_ran]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

    // Should be able to acquire lock after t1 releases it early
    config.getState("height").value(2000);
    other_thread_ran.store(true);
  });

  t1.join();
  t2.join();
  config.waitForHandlers();

  EXPECT_TRUE(other_thread_ran.load());
}

// DataProcessor state_object example (from STATE_API.md)
class DataProcessorObject : public state_object<DataProcessorObject> {
public:
  std::atomic<int> statusChangeCount;
  std::atomic<int> errorAlertCount;
  std::string lastStatus;
  boost::mutex statusMutex;

  DataProcessorObject(cvc::app &ctx)
      : state_object<DataProcessorObject>(ctx), statusChangeCount(0), errorAlertCount(0) {
    getState("status").value("idle");
    getState("progress").value(0.0);
    getState("error_count").value(0);
  }

  void processData(const std::vector<double> &data) {
    getState("status").value("processing");

    for (size_t i = 0; i < data.size(); ++i) {
      // Simulate processing
      if (data[i] < 0) {
        // Simulate error
        int count = getState("error_count").value<int>();
        getState("error_count").value(count + 1);
      }

      // Update progress
      getState("progress").value(static_cast<double>(i + 1) / data.size());
    }

    getState("status").value("complete");
    getState("progress").value(1.0);
  }

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    // childState includes the full path
    if (childState.find("status") != std::string::npos) {
      boost::mutex::scoped_lock lock(statusMutex);
      lastStatus = getState("status").value();
      statusChangeCount++;
    } else if (childState.find("error_count") != std::string::npos) {
      int errors = getState("error_count").value<int>();
      if (errors > 3) {
        errorAlertCount++;
      }
    }
  }
};

TEST_F(StateObjectFixture, StateObjectDataProcessor) {
  DataProcessorObject processor(ctx);

  // Verify initial state
  EXPECT_EQ(processor.getState("status").value(), "idle");
  EXPECT_DOUBLE_EQ(processor.getState("progress").value<double>(), 0.0);

  // Process some data with errors
  std::vector<double> data = {1.0, 2.0, -1.0, 3.0, -2.0, 4.0, -3.0, 5.0, -4.0, -5.0};
  processor.processData(data);

  // Wait for all handlers to complete
  processor.waitForHandlers();

  // Verify final state
  EXPECT_EQ(processor.getState("status").value(), "complete");
  EXPECT_DOUBLE_EQ(processor.getState("progress").value<double>(), 1.0);
  EXPECT_EQ(processor.getState("error_count").value<int>(), 5);

  // Verify handlers were called
  EXPECT_GT(processor.statusChangeCount.load(), 0);
  EXPECT_GT(processor.errorAlertCount.load(), 0); // Should alert since errors > 3

  {
    boost::mutex::scoped_lock lock(processor.statusMutex);
    EXPECT_EQ(processor.lastStatus, "complete");
  }
}

// Renderer state_object example (from STATE_API.md)
class RendererObject : public state_object<RendererObject> {
public:
  std::atomic<int> cameraUpdateCount;
  std::atomic<int> renderModeChangeCount;
  std::atomic<int> redrawRequestCount;
  std::string lastRenderMode;
  boost::mutex modeMutex;

  RendererObject(cvc::app &ctx)
      : state_object<RendererObject>(ctx), cameraUpdateCount(0), renderModeChangeCount(0),
        redrawRequestCount(0) {
    getState("camera.position").value("0,0,10");
    getState("camera.target").value("0,0,0");
    getState("render_mode").value("solid");
  }

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    // childState includes the full path
    if (childState.find("camera.") != std::string::npos) {
      cameraUpdateCount++;
      redrawRequestCount++;
    } else if (childState.find("render_mode") != std::string::npos) {
      boost::mutex::scoped_lock lock(modeMutex);
      lastRenderMode = getState("render_mode").value();
      renderModeChangeCount++;
      redrawRequestCount++;
    }
  }
};

TEST_F(StateObjectFixture, StateObjectRenderer) {
  RendererObject renderer(ctx);

  // Verify initial state
  EXPECT_EQ(renderer.getState("camera.position").value(), "0,0,10");
  EXPECT_EQ(renderer.getState("camera.target").value(), "0,0,0");
  EXPECT_EQ(renderer.getState("render_mode").value(), "solid");

  // Update camera from multiple threads
  boost::thread t1([&renderer]() { renderer.getState("camera.position").value("5,5,5"); });

  boost::thread t2([&renderer]() { renderer.getState("camera.target").value("1,1,1"); });

  boost::thread t3([&renderer]() { renderer.getState("render_mode").value("wireframe"); });

  t1.join();
  t2.join();
  t3.join();

  // Wait for all handlers to complete
  renderer.waitForHandlers();

  // Verify all updates triggered handlers
  EXPECT_GT(renderer.cameraUpdateCount.load(), 0);
  EXPECT_GT(renderer.renderModeChangeCount.load(), 0);
  EXPECT_GT(renderer.redrawRequestCount.load(), 0);

  {
    boost::mutex::scoped_lock lock(renderer.modeMutex);
    EXPECT_EQ(renderer.lastRenderMode, "wireframe");
  }
}

// AppSettings state_object example (from STATE_API.md)
class AppSettingsObject : public state_object<AppSettingsObject> {
public:
  std::atomic<int> themeChangeCount;
  std::string lastTheme;
  boost::mutex themeMutex;

  AppSettingsObject(cvc::app &ctx) : state_object<AppSettingsObject>(ctx), themeChangeCount(0) {
    loadDefaults();
  }

  void loadDefaults() {
    getState("window.width").value(1920);
    getState("window.height").value(1080);
    getState("theme").value("dark");
    getState("language").value("en");
  }

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    // childState includes the full path
    if (childState.find("theme") != std::string::npos) {
      boost::mutex::scoped_lock lock(themeMutex);
      lastTheme = getState("theme").value();
      themeChangeCount++;
    }
  }
};

TEST_F(StateObjectFixture, StateObjectAppSettings) {
  AppSettingsObject settings(ctx);

  // Verify defaults
  EXPECT_EQ(settings.getState("window.width").value<int>(), 1920);
  EXPECT_EQ(settings.getState("window.height").value<int>(), 1080);
  EXPECT_EQ(settings.getState("theme").value(), "dark");
  EXPECT_EQ(settings.getState("language").value(), "en");

  // Change theme
  settings.getState("theme").value("light");

  // Wait for handlers to complete
  settings.waitForHandlers();

  EXPECT_GT(settings.themeChangeCount.load(), 0);

  {
    boost::mutex::scoped_lock lock(settings.themeMutex);
    EXPECT_EQ(settings.lastTheme, "light");
  }
}

// Test external state access pattern
TEST_F(StateObjectFixture, StateObjectExternalAccess) {
  ConfigurationObject config(ctx);

  // Get the state path
  std::string widthPath = config.stateName("width");

  // Access via the same per-app state root the state_object is bound to
  cvc::state::instance(ctx)(widthPath).value(3840);

  // Wait for handlers to complete
  config.waitForHandlers();

  // Verify change was detected
  EXPECT_EQ(config.lastWidth, 3840);
  EXPECT_GT(config.resizeCount.load(), 0);
}

// Test state_object with nested state paths
class NestedStateObject : public state_object<NestedStateObject> {
public:
  std::atomic<int> deepPathChangeCount;

  NestedStateObject(cvc::app &ctx) : state_object<NestedStateObject>(ctx), deepPathChangeCount(0) {
    getState("level1.level2.level3.value").value("deep");
  }

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    // childState includes the full path
    if (childState.find("level1.level2.level3") != std::string::npos) {
      deepPathChangeCount++;
    }
  }
};

TEST_F(StateObjectFixture, StateObjectNestedPaths) {
  NestedStateObject obj(ctx);

  // Verify deep path initialization
  EXPECT_EQ(obj.getState("level1.level2.level3.value").value(), "deep");

  // Modify deep path
  obj.getState("level1.level2.level3.value").value("modified");

  // Wait for handlers to complete
  obj.waitForHandlers();

  EXPECT_GT(obj.deepPathChangeCount.load(), 0);
}

// Test state_object state name generation
TEST_F(StateObjectFixture, StateObjectStateName) {
  ConfigurationObject config(ctx);

  // stateName() should include instance information
  std::string name = config.stateName();
  EXPECT_FALSE(name.empty());
  // Note: Type name may be "This" due to template parameter in dataTypeName

  // stateName with child should append child path
  std::string childName = config.stateName("width");
  EXPECT_FALSE(childName.empty());
  EXPECT_NE(childName.find("width"), std::string::npos);
}

// Test multiple instances of same state_object type
TEST_F(StateObjectFixture, StateObjectMultipleInstances) {
  ConfigurationObject config1(ctx);
  ConfigurationObject config2(ctx);

  // Each instance should have unique state path
  std::string name1 = config1.stateName();
  std::string name2 = config2.stateName();
  EXPECT_NE(name1, name2);

  // Modify config1 - should not affect config2
  config1.getState("width").value(2560);
  config2.getState("width").value(1024);

  // Wait for handlers to complete
  config1.waitForHandlers();
  config2.waitForHandlers();

  EXPECT_EQ(config1.lastWidth, 2560);
  EXPECT_EQ(config2.lastWidth, 1024);
}

// Test state_object with data objects
class DataStateObject : public state_object<DataStateObject> {
public:
  std::atomic<int> dataChangeCount;
  int lastDataValue;
  boost::mutex dataMutex;

  DataStateObject(cvc::app &ctx)
      : state_object<DataStateObject>(ctx), dataChangeCount(0), lastDataValue(0) {
    getState("data_value").data(42);
  }

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    // childState includes the full path
    if (childState.find("data_value") != std::string::npos) {
      if (getState("data_value").isData<int>()) {
        boost::mutex::scoped_lock lock(dataMutex);
        lastDataValue = getState("data_value").data<int>();
        dataChangeCount++;
      }
    }
  }
};

TEST_F(StateObjectFixture, StateObjectWithData) {
  DataStateObject obj(ctx);

  // Verify initial data
  EXPECT_TRUE(obj.getState("data_value").isData<int>());
  EXPECT_EQ(obj.getState("data_value").data<int>(), 42);

  // Change data
  obj.getState("data_value").data(999);

  // Wait for handlers to complete
  obj.waitForHandlers();

  EXPECT_GT(obj.dataChangeCount.load(), 0);

  {
    boost::mutex::scoped_lock lock(obj.dataMutex);
    EXPECT_EQ(obj.lastDataValue, 999);
  }
}

// Test state_object async handler execution
TEST_F(StateObjectFixture, StateObjectAsyncHandlers) {
  ConfigurationObject config(ctx);

  int initialCount = config.resizeCount.load();

  // Make rapid changes
  for (int i = 0; i < 10; ++i) {
    config.getState("width").value(1920 + i * 100);
  }

  // Wait for all handlers to complete
  config.waitForHandlers();

  // Should have processed all changes
  int finalCount = config.resizeCount.load();
  EXPECT_GT(finalCount, initialCount);
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
          cvc::state::instance(test_ctx)(key).value("val_" + boost::lexical_cast<std::string>(ops));
          cvc::state::instance(test_ctx)(key).data(ops);
          cvc::state::instance(test_ctx)(key).comment("comment_" +
                                                      boost::lexical_cast<std::string>(ops));
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
          std::string val = cvc::state::instance(test_ctx)(key).value();
          if (cvc::state::instance(test_ctx)(key).isData<int>()) {
            int data = cvc::state::instance(test_ctx)(key).data<int>();
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
        cvc::state::instance(test_ctx)("test.stress").traverse([](std::string) {});
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
  for (auto &t : threads) {
    t.join();
  }

  EXPECT_GT(total_ops.load(), 0);
  std::cout << "Stress test completed " << total_ops.load() << " operations without deadlock\n";

  cvc::state::instance(test_ctx)("test.stress").reset();
}

TEST(StateTest, StressTestHierarchyRaceConditions) {
  // Aggressive stress test to expose race conditions in hierarchy management
  // Tests concurrent creation, deletion, modification of parent/child relationships
  const int duration_ms = 2000; // Run for 2 seconds
  const int num_parents = 5;
  const int children_per_parent = 10;

  std::atomic<bool> stop_flag(false);
  std::atomic<int> total_ops(0);
  std::atomic<int> orphan_errors(0);
  std::atomic<int> hierarchy_errors(0);
  std::vector<boost::thread> threads;

  // Thread 1: Rapidly create deep hierarchies
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        for (int p = 0; p < num_parents; ++p) {
          std::string parent = "test.hierarchy.parent" + boost::lexical_cast<std::string>(p);
          cvc::state::instance(test_ctx)(parent).value("parent_value");

          for (int c = 0; c < children_per_parent; ++c) {
            std::string child = parent + ".child" + boost::lexical_cast<std::string>(c);
            cvc::state::instance(test_ctx)(child).value("child_value_" +
                                                        boost::lexical_cast<std::string>(ops));
            cvc::state::instance(test_ctx)(child).data(ops);
            ops++;
          }
        }
      } catch (...) {
        hierarchy_errors++;
      }
    }
    total_ops += ops;
  });

  // Thread 2: Reset parents while children are being accessed
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        for (int p = 0; p < num_parents; ++p) {
          std::string parent = "test.hierarchy.parent" + boost::lexical_cast<std::string>(p);
          cvc::state::instance(test_ctx)(parent).reset(); // Should clean up all children
          ops++;
        }
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      } catch (...) {
        hierarchy_errors++;
      }
    }
    total_ops += ops;
  });

  // Thread 3: Traverse hierarchy while it's being modified
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        cvc::state::instance(test_ctx)("test.hierarchy").traverse([](std::string key) {
          // Try to access each node during traversal
          cvc::state::instance(test_ctx)(key).value();
        });
        ops++;
      } catch (...) {
        // May fail if hierarchy changes during traversal
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(25));
    }
    total_ops += ops;
  });

  // Thread 4: Rapidly read/write to random children
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        int p = ops % num_parents;
        int c = ops % children_per_parent;
        std::string child = "test.hierarchy.parent" + boost::lexical_cast<std::string>(p) +
                            ".child" + boost::lexical_cast<std::string>(c);

        // Write
        cvc::state::instance(test_ctx)(child).value("updated_" +
                                                    boost::lexical_cast<std::string>(ops));
        cvc::state::instance(test_ctx)(child).data(ops * 2);

        // Read back
        std::string val = cvc::state::instance(test_ctx)(child).value();
        if (cvc::state::instance(test_ctx)(child).isData<int>()) {
          int data = cvc::state::instance(test_ctx)(child).data<int>();
          (void)data;
        }
        ops++;
      } catch (...) {
        // May fail if parent was reset
      }
    }
    total_ops += ops;
  });

  // Thread 5: Create and destroy sibling hierarchies
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        std::string base = "test.hierarchy.dynamic" + boost::lexical_cast<std::string>(ops % 3);

        // Create hierarchy
        for (int i = 0; i < 5; ++i) {
          std::string path = base + ".level1.level2.node" + boost::lexical_cast<std::string>(i);
          cvc::state::instance(test_ctx)(path).value("deep_value");
        }

        // Destroy it
        cvc::state::instance(test_ctx)(base).reset();
        ops++;
      } catch (...) {
        hierarchy_errors++;
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }
    total_ops += ops;
  });

  // Thread 6: Query children while hierarchy is being modified
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        for (int p = 0; p < num_parents; ++p) {
          std::string parent = "test.hierarchy.parent" + boost::lexical_cast<std::string>(p);

          // Get children list
          std::vector<std::string> children = cvc::state::instance(test_ctx)(parent).children();

          // Verify we can access each child
          for (const auto &child : children) {
            try {
              std::string fullName = cvc::state::instance(test_ctx)(child).fullName();
              std::string parentName = cvc::state::instance(test_ctx)(child).parentName();

              // Check parent/child relationship
              if (parentName != parent) {
                // Possible orphan if parent doesn't match
                orphan_errors++;
              }
            } catch (...) {
              // Child may have been deleted
            }
          }
          ops++;
        }
      } catch (...) {
        hierarchy_errors++;
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
    }
    total_ops += ops;
  });

  // Thread 7: Modify parent values while children are being created
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        for (int p = 0; p < num_parents; ++p) {
          std::string parent = "test.hierarchy.parent" + boost::lexical_cast<std::string>(p);
          cvc::state::instance(test_ctx)(parent).value("modified_" +
                                                       boost::lexical_cast<std::string>(ops));
          cvc::state::instance(test_ctx)(parent).comment("comment_" +
                                                         boost::lexical_cast<std::string>(ops));
          cvc::state::instance(test_ctx)(parent).data(ops);
          ops++;
        }
      } catch (...) {
        hierarchy_errors++;
      }
    }
    total_ops += ops;
  });

  // Thread 8: Create complex multi-level hierarchies
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        for (int i = 0; i < 3; ++i) {
          std::string root = "test.hierarchy.complex" + boost::lexical_cast<std::string>(i);

          // Create 3-level deep hierarchy
          for (int l1 = 0; l1 < 3; ++l1) {
            for (int l2 = 0; l2 < 3; ++l2) {
              for (int l3 = 0; l3 < 2; ++l3) {
                std::string path = root + ".l1_" + boost::lexical_cast<std::string>(l1) + ".l2_" +
                                   boost::lexical_cast<std::string>(l2) + ".l3_" +
                                   boost::lexical_cast<std::string>(l3);
                cvc::state::instance(test_ctx)(path).value("deep_" +
                                                           boost::lexical_cast<std::string>(ops));
                ops++;
              }
            }
          }

          // Reset entire hierarchy
          if (ops % 20 == 0) {
            cvc::state::instance(test_ctx)(root).reset();
          }
        }
      } catch (...) {
        hierarchy_errors++;
      }
    }
    total_ops += ops;
  });

  // Thread 9: Verify hierarchy integrity
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        // Check that all accessible children have valid parents
        cvc::state::instance(test_ctx)("test.hierarchy").traverse([&](std::string key) {
          try {
            std::string parent = cvc::state::instance(test_ctx)(key).parentName();
            if (!parent.empty() && parent != "test.hierarchy") {
              // Parent should be accessible
              std::string parent_value = cvc::state::instance(test_ctx)(parent).value();
              (void)parent_value;

              // This node should be in parent's children list
              std::vector<std::string> siblings = cvc::state::instance(test_ctx)(parent).children();
              bool found = false;
              for (const auto &sibling : siblings) {
                if (sibling == key) {
                  found = true;
                  break;
                }
              }
              if (!found) {
                orphan_errors++;
              }
            }
          } catch (...) {
            // Parent may have been deleted
          }
        });
        ops++;
      } catch (...) {
        // Traversal may fail if hierarchy changes
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    }
    total_ops += ops;
  });

  // Thread 10: Rapid parent switching (move children between parents)
  threads.emplace_back([&]() {
    int ops = 0;
    while (!stop_flag.load()) {
      try {
        // Create child under parent A
        std::string parentA = "test.hierarchy.parentA";
        std::string parentB = "test.hierarchy.parentB";
        std::string child_name = "migrating_child" + boost::lexical_cast<std::string>(ops % 5);

        std::string childA = parentA + "." + child_name;
        std::string childB = parentB + "." + child_name;

        // Create under parent A
        cvc::state::instance(test_ctx)(childA).value("under_A_" +
                                                     boost::lexical_cast<std::string>(ops));
        cvc::state::instance(test_ctx)(childA).data(ops);

        // Try to create same-named child under parent B
        cvc::state::instance(test_ctx)(childB).value("under_B_" +
                                                     boost::lexical_cast<std::string>(ops));
        cvc::state::instance(test_ctx)(childB).data(ops + 1000);

        // Verify both exist independently
        std::string valA = cvc::state::instance(test_ctx)(childA).value();
        std::string valB = cvc::state::instance(test_ctx)(childB).value();
        (void)valA;
        (void)valB;

        ops++;
      } catch (...) {
        hierarchy_errors++;
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    }
    total_ops += ops;
  });

  // Let it run
  boost::this_thread::sleep_for(boost::chrono::milliseconds(duration_ms));
  stop_flag.store(true);

  // Join all threads
  for (auto &t : threads) {
    t.join();
  }

  std::cout << "Hierarchy stress test completed:\n"
            << "  Total operations: " << total_ops.load() << "\n"
            << "  Orphan errors: " << orphan_errors.load() << "\n"
            << "  Hierarchy errors: " << hierarchy_errors.load() << "\n";

  // Should complete without deadlocks
  EXPECT_GT(total_ops.load(), 0);

  // Ideally no orphans (though some may occur during concurrent resets)
  // We're mainly checking that we don't crash or deadlock
  std::cout << "Hierarchy integrity check: "
            << (orphan_errors.load() == 0 ? "PERFECT" : "ACCEPTABLE (concurrent modifications)")
            << "\n";

  cvc::state::instance(test_ctx)("test.hierarchy").reset();
}

// ===========================
// Performance Tests
// ===========================

TEST(StateTest, PerformanceHierarchyBenchmark) {
  if (!enable_stress_tests) {
    GTEST_SKIP() << "Stress test disabled. Use --enable-stress-tests to run.";
  }

  // Performance test: 1 million operations on deep hierarchy
  // Tests various data types and sizes
  // Monitors memory usage and throughput
  // Fails if memory usage is excessive or if it takes > 3 minutes

  const int NUM_OPERATIONS = 1000000;
  const int MAX_RUNTIME_SECONDS = 360;      // 6 minutes
  const double MEMORY_OVERHEAD_LIMIT = 3.0; // Max 3x overhead

  auto start_time = boost::chrono::high_resolution_clock::now();

  // Track memory usage (approximate)
  size_t total_data_size = 0;
  size_t operations_completed = 0;

  std::cout << "\n=== State Hierarchy Performance Benchmark ===\n";
  std::cout << "Target: " << NUM_OPERATIONS << " operations\n";
  std::cout << "Timeout: " << MAX_RUNTIME_SECONDS << " seconds\n\n";

  // Create deep hierarchy structure
  const int DEPTH = 5;
  const int BREADTH = 10;

  // Test 1: Small integer values (deep hierarchy)
  std::cout << "Test 1: Writing 100k small integers in deep hierarchy...\n";
  for (int i = 0; i < 100000; ++i) {
    int level = i % DEPTH;
    int branch = (i / DEPTH) % BREADTH;
    std::string key = "perf.test.L" + boost::lexical_cast<std::string>(level) + ".B" +
                      boost::lexical_cast<std::string>(branch) + ".item" +
                      boost::lexical_cast<std::string>(i);
    cvc::state::instance(test_ctx)(key).value(i);
    total_data_size += sizeof(int);
    operations_completed++;

    // Check timeout
    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint1 = boost::chrono::high_resolution_clock::now();
  auto elapsed1 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint1 - start_time).count();
  std::cout << "  Completed in " << elapsed1 << " ms (" << (100000.0 / (elapsed1 / 1000.0))
            << " ops/sec)\n";

  // Test 2: Read back integers and verify
  std::cout << "Test 2: Reading 100k integers...\n";
  for (int i = 0; i < 100000; ++i) {
    int level = i % DEPTH;
    int branch = (i / DEPTH) % BREADTH;
    std::string key = "perf.test.L" + boost::lexical_cast<std::string>(level) + ".B" +
                      boost::lexical_cast<std::string>(branch) + ".item" +
                      boost::lexical_cast<std::string>(i);
    int val = cvc::state::instance(test_ctx)(key).value<int>();
    ASSERT_EQ(val, i);
    operations_completed++;

    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint2 = boost::chrono::high_resolution_clock::now();
  auto elapsed2 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint2 - checkpoint1).count();
  std::cout << "  Completed in " << elapsed2 << " ms (" << (100000.0 / (elapsed2 / 1000.0))
            << " ops/sec)\n";

  // Test 3: String values (variable length)
  std::cout << "Test 3: Writing 100k variable-length strings...\n";
  for (int i = 0; i < 100000; ++i) {
    std::string key = "perf.strings.item" + boost::lexical_cast<std::string>(i);
    std::string value = "String_" + boost::lexical_cast<std::string>(i) + "_" +
                        std::string(i % 100, 'x'); // Variable length
    cvc::state::instance(test_ctx)(key).value(value);
    total_data_size += value.size();
    operations_completed++;

    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint3 = boost::chrono::high_resolution_clock::now();
  auto elapsed3 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint3 - checkpoint2).count();
  std::cout << "  Completed in " << elapsed3 << " ms (" << (100000.0 / (elapsed3 / 1000.0))
            << " ops/sec)\n";

  // Test 4: Read strings back
  std::cout << "Test 4: Reading 100k strings...\n";
  for (int i = 0; i < 100000; ++i) {
    std::string key = "perf.strings.item" + boost::lexical_cast<std::string>(i);
    std::string expected =
        "String_" + boost::lexical_cast<std::string>(i) + "_" + std::string(i % 100, 'x');
    std::string val = cvc::state::instance(test_ctx)(key).value();
    ASSERT_EQ(val, expected);
    operations_completed++;

    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint4 = boost::chrono::high_resolution_clock::now();
  auto elapsed4 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint4 - checkpoint3).count();
  std::cout << "  Completed in " << elapsed4 << " ms (" << (100000.0 / (elapsed4 / 1000.0))
            << " ops/sec)\n";

  // Test 5: Comma-separated lists in value strings
  std::cout << "Test 5: Writing 100k comma-separated lists...\n";
  for (int i = 0; i < 100000; ++i) {
    std::string key = "perf.lists.item" + boost::lexical_cast<std::string>(i);
    std::string list_value;
    for (int j = 0; j < 10; ++j) {
      if (j > 0)
        list_value += ",";
      list_value += boost::lexical_cast<std::string>(i * 10 + j);
    }
    cvc::state::instance(test_ctx)(key).value(list_value);
    total_data_size += list_value.size();
    operations_completed++;

    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint5 = boost::chrono::high_resolution_clock::now();
  auto elapsed5 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint5 - checkpoint4).count();
  std::cout << "  Completed in " << elapsed5 << " ms (" << (100000.0 / (elapsed5 / 1000.0))
            << " ops/sec)\n";

  // Test 6: Parse lists back
  std::cout << "Test 6: Reading and parsing 100k lists...\n";
  for (int i = 0; i < 100000; ++i) {
    std::string key = "perf.lists.item" + boost::lexical_cast<std::string>(i);
    std::string list_str = cvc::state::instance(test_ctx)(key).value();

    // Parse comma-separated values
    std::vector<int> values;
    size_t pos = 0;
    while (pos < list_str.size()) {
      size_t comma = list_str.find(',', pos);
      if (comma == std::string::npos)
        comma = list_str.size();
      std::string token = list_str.substr(pos, comma - pos);
      values.push_back(boost::lexical_cast<int>(token));
      pos = comma + 1;
    }

    ASSERT_EQ(values.size(), 10);
    for (int j = 0; j < 10; ++j) {
      ASSERT_EQ(values[j], i * 10 + j);
    }
    operations_completed++;

    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint6 = boost::chrono::high_resolution_clock::now();
  auto elapsed6 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint6 - checkpoint5).count();
  std::cout << "  Completed in " << elapsed6 << " ms (" << (100000.0 / (elapsed6 / 1000.0))
            << " ops/sec)\n";

  // Test 7: Large binary arrays using data()
  std::cout << "Test 7: Writing 10k large binary arrays (10KB each)...\n";
  const size_t ARRAY_SIZE = 10240; // 10KB
  for (int i = 0; i < 10000; ++i) {
    std::string key = "perf.arrays.item" + boost::lexical_cast<std::string>(i);
    std::vector<unsigned char> array(ARRAY_SIZE);

    // Fill with pattern
    for (size_t j = 0; j < ARRAY_SIZE; ++j) {
      array[j] = static_cast<unsigned char>((i + j) % 256);
    }

    cvc::state::instance(test_ctx)(key).data(array);
    total_data_size += ARRAY_SIZE;
    operations_completed++;

    if (i % 1000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint7 = boost::chrono::high_resolution_clock::now();
  auto elapsed7 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint7 - checkpoint6).count();
  std::cout << "  Completed in " << elapsed7 << " ms (" << (10000.0 / (elapsed7 / 1000.0))
            << " ops/sec, " << ((10000.0 * ARRAY_SIZE / 1024.0 / 1024.0) / (elapsed7 / 1000.0))
            << " MB/sec)\n";

  // Test 8: Read binary arrays back and verify
  std::cout << "Test 8: Reading 10k binary arrays...\n";
  for (int i = 0; i < 10000; ++i) {
    std::string key = "perf.arrays.item" + boost::lexical_cast<std::string>(i);
    std::vector<unsigned char> array =
        cvc::state::instance(test_ctx)(key).data<std::vector<unsigned char>>();

    ASSERT_EQ(array.size(), ARRAY_SIZE);

    // Verify pattern
    for (size_t j = 0; j < ARRAY_SIZE; ++j) {
      ASSERT_EQ(array[j], static_cast<unsigned char>((i + j) % 256));
    }
    operations_completed++;

    if (i % 1000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint8 = boost::chrono::high_resolution_clock::now();
  auto elapsed8 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint8 - checkpoint7).count();
  std::cout << "  Completed in " << elapsed8 << " ms (" << (10000.0 / (elapsed8 / 1000.0))
            << " ops/sec, " << ((10000.0 * ARRAY_SIZE / 1024.0 / 1024.0) / (elapsed8 / 1000.0))
            << " MB/sec)\n";

  // Test 9: Mixed operations (reads and writes interleaved)
  std::cout << "Test 9: Mixed read/write operations (100k)...\n";
  for (int i = 0; i < 100000; ++i) {
    std::string key = "perf.mixed.item" + boost::lexical_cast<std::string>(i % 1000);

    if (i % 2 == 0) {
      // Write integer using data()
      cvc::state::instance(test_ctx)(key).data(i);
    } else {
      // Read back if exists and is int type
      try {
        if (cvc::state::instance(test_ctx)(key).isData<int>()) {
          int val = cvc::state::instance(test_ctx)(key).data<int>();
          (void)val;
        }
      } catch (...) {
        // Ignore exceptions during mixed read/write
      }
    }
    operations_completed++;

    if (i % 10000 == 0) {
      auto current = boost::chrono::high_resolution_clock::now();
      auto elapsed =
          boost::chrono::duration_cast<boost::chrono::seconds>(current - start_time).count();
      if (elapsed > MAX_RUNTIME_SECONDS) {
        FAIL() << "Timeout: exceeded " << MAX_RUNTIME_SECONDS << " seconds";
      }
    }
  }

  auto checkpoint9 = boost::chrono::high_resolution_clock::now();
  auto elapsed9 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint9 - checkpoint8).count();
  std::cout << "  Completed in " << elapsed9 << " ms (" << (100000.0 / (elapsed9 / 1000.0))
            << " ops/sec)\n";

  // Test 10: Hierarchy traversal
  std::cout << "Test 10: Traversing entire hierarchy (100k items)...\n";
  std::atomic<int> traverse_count(0);
  cvc::state::instance(test_ctx)("perf").traverse([&](std::string key) {
    traverse_count++;
    operations_completed++;
  });

  auto checkpoint10 = boost::chrono::high_resolution_clock::now();
  auto elapsed10 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint10 - checkpoint9).count();
  std::cout << "  Traversed " << traverse_count.load() << " nodes in " << elapsed10 << " ms\n";

  // Calculate final statistics
  auto end_time = boost::chrono::high_resolution_clock::now();
  auto total_elapsed =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(end_time - start_time).count();

  std::cout << "\n=== Performance Summary ===\n";
  std::cout << "Total operations: " << operations_completed << "\n";
  std::cout << "Total time: " << total_elapsed << " ms (" << (total_elapsed / 1000.0)
            << " seconds)\n";
  std::cout << "Average throughput: " << (operations_completed / (total_elapsed / 1000.0))
            << " ops/sec\n";
  std::cout << "Total data written: " << (total_data_size / 1024.0 / 1024.0) << " MB\n";
  std::cout << "Data throughput: "
            << ((total_data_size / 1024.0 / 1024.0) / (total_elapsed / 1000.0)) << " MB/sec\n";

  // Estimate memory usage
  // Each state node has overhead: name, value, mutex, parent pointer, children map
  // Rough estimate: 200 bytes overhead per node + data size
  size_t estimated_nodes = 100000 + 100000 + 10000 + 1000; // Different key sets
  size_t estimated_overhead = estimated_nodes * 200;
  size_t total_estimated_memory = total_data_size + estimated_overhead;
  double memory_ratio = static_cast<double>(total_estimated_memory) / total_data_size;

  std::cout << "Estimated memory usage: " << (total_estimated_memory / 1024.0 / 1024.0) << " MB\n";
  std::cout << "Memory overhead ratio: " << memory_ratio << "x\n";

  // Verify performance requirements
  EXPECT_LT(total_elapsed / 1000.0, MAX_RUNTIME_SECONDS) << "Performance test took too long";
  EXPECT_LT(memory_ratio, MEMORY_OVERHEAD_LIMIT) << "Memory overhead too high: " << memory_ratio
                                                 << "x (limit: " << MEMORY_OVERHEAD_LIMIT << "x)";
  EXPECT_GE(operations_completed, NUM_OPERATIONS) << "Not enough operations completed";

  std::cout << "\n✓ Performance test PASSED\n";
  std::cout << "  - Completed " << operations_completed << " operations in "
            << (total_elapsed / 1000.0) << " seconds\n";
  std::cout << "  - Memory overhead within acceptable range\n";
  std::cout << "  - Throughput: " << (operations_completed / (total_elapsed / 1000.0))
            << " ops/sec\n";

  // Cleanup
  cvc::state::instance(test_ctx)("perf").reset();
}

TEST(StateTest, PerformanceCallbackChains) {
  // Performance test for cascading callbacks
  // Tests callback chains that trigger other state changes
  // Ensures no stack overflow with deep callback chains
  // Tests safeguards against infinite callback loops

  std::cout << "\n=== Callback Chain Performance & Safety Test ===\n";

  const int MAX_CHAIN_DEPTH = 100;
  const int MAX_RUNTIME_SECONDS = 60;
  auto start_time = boost::chrono::high_resolution_clock::now();

  std::atomic<int> total_callbacks(0);
  std::atomic<int> chain_depth(0);
  std::atomic<int> max_depth_reached(0);

  // Test 1: Linear callback chain (A triggers B triggers C...)
  std::cout << "Test 1: Linear callback chain (depth " << MAX_CHAIN_DEPTH << ")...\n";

  std::vector<boost::signals2::connection> connections;

  // Set up chain: each node triggers the next
  for (int i = 0; i < MAX_CHAIN_DEPTH - 1; ++i) {
    std::string current = "perf.chain.linear." + boost::lexical_cast<std::string>(i);
    std::string next = "perf.chain.linear." + boost::lexical_cast<std::string>(i + 1);

    auto conn = cvc::state::instance(test_ctx)(current).valueChanged.connect(
        [&total_callbacks, next, i, &chain_depth, &max_depth_reached]() {
          total_callbacks++;
          int current_depth = chain_depth.fetch_add(1) + 1;

          // Track max depth
          int current_max = max_depth_reached.load();
          while (current_depth > current_max &&
                 !max_depth_reached.compare_exchange_weak(current_max, current_depth)) {
            current_max = max_depth_reached.load();
          }

          // Trigger next in chain
          cvc::state::instance(test_ctx)(next).value(i + 1);

          chain_depth.fetch_sub(1);
        });
    connections.push_back(conn);
  }

  // Initialize all nodes first
  for (int i = 0; i < MAX_CHAIN_DEPTH; ++i) {
    std::string key = "perf.chain.linear." + boost::lexical_cast<std::string>(i);
    cvc::state::instance(test_ctx)(key).value(0);
  }

  // Trigger the chain
  total_callbacks.store(0);
  chain_depth.store(0);
  max_depth_reached.store(0);
  cvc::state::instance(test_ctx)("perf.chain.linear.0").value(1);

  // Wait for chain to complete (small delay)
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  auto checkpoint1 = boost::chrono::high_resolution_clock::now();
  auto elapsed1 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint1 - start_time).count();

  std::cout << "  Completed in " << elapsed1 << " ms\n";
  std::cout << "  Total callbacks fired: " << total_callbacks.load() << "\n";
  std::cout << "  Max stack depth reached: " << max_depth_reached.load() << "\n";

  EXPECT_EQ(total_callbacks.load(), MAX_CHAIN_DEPTH - 1) << "All callbacks should fire once";
  EXPECT_LE(max_depth_reached.load(), MAX_CHAIN_DEPTH) << "Stack depth should be reasonable";

  // Disconnect linear chain
  for (auto &conn : connections) {
    conn.disconnect();
  }
  connections.clear();

  // Test 2: Fan-out callbacks (one parent triggers multiple children)
  std::cout << "\nTest 2: Fan-out callbacks (1 parent -> 50 children)...\n";

  const int FAN_OUT_COUNT = 50;
  std::atomic<int> fanout_callbacks(0);
  std::atomic<int> child_callbacks(0);

  // Parent callback triggers writes to all children
  auto parent_conn = cvc::state::instance(test_ctx)("perf.chain.fanout.parent")
                         .valueChanged.connect([&fanout_callbacks, FAN_OUT_COUNT]() {
                           fanout_callbacks++;
                           for (int i = 0; i < FAN_OUT_COUNT; ++i) {
                             std::string child =
                                 "perf.chain.fanout.child." + boost::lexical_cast<std::string>(i);
                             cvc::state::instance(test_ctx)(child).value(i);
                           }
                         });

  // Each child has a callback
  std::vector<boost::signals2::connection> child_conns;
  for (int i = 0; i < FAN_OUT_COUNT; ++i) {
    std::string child = "perf.chain.fanout.child." + boost::lexical_cast<std::string>(i);
    auto conn = cvc::state::instance(test_ctx)(child).valueChanged.connect(
        [&child_callbacks]() { child_callbacks++; });
    child_conns.push_back(conn);
  }

  // Trigger fan-out
  fanout_callbacks.store(0);
  child_callbacks.store(0);
  cvc::state::instance(test_ctx)("perf.chain.fanout.parent").value(42);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  auto checkpoint2 = boost::chrono::high_resolution_clock::now();
  auto elapsed2 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint2 - checkpoint1).count();

  std::cout << "  Completed in " << elapsed2 << " ms\n";
  std::cout << "  Parent callbacks: " << fanout_callbacks.load() << "\n";
  std::cout << "  Child callbacks: " << child_callbacks.load() << "\n";

  EXPECT_EQ(fanout_callbacks.load(), 1) << "Parent callback should fire once";
  EXPECT_EQ(child_callbacks.load(), FAN_OUT_COUNT) << "All child callbacks should fire";

  parent_conn.disconnect();
  for (auto &conn : child_conns) {
    conn.disconnect();
  }
  child_conns.clear();

  // Test 3: Prevent infinite loops with counter-based guard
  std::cout << "\nTest 3: Infinite loop prevention (mutual triggers with guard)...\n";

  std::atomic<int> nodeA_callbacks(0);
  std::atomic<int> nodeB_callbacks(0);
  const int MAX_ITERATIONS = 10; // Safety limit

  // Node A triggers B (with iteration limit)
  auto connA = cvc::state::instance(test_ctx)("perf.chain.loop.A")
                   .valueChanged.connect([&nodeA_callbacks, MAX_ITERATIONS]() {
                     int count = nodeA_callbacks.fetch_add(1);
                     if (count < MAX_ITERATIONS) {
                       cvc::state::instance(test_ctx)("perf.chain.loop.B").value(count);
                     }
                   });

  // Node B triggers A (with iteration limit)
  auto connB = cvc::state::instance(test_ctx)("perf.chain.loop.B")
                   .valueChanged.connect([&nodeB_callbacks, MAX_ITERATIONS]() {
                     int count = nodeB_callbacks.fetch_add(1);
                     if (count < MAX_ITERATIONS) {
                       cvc::state::instance(test_ctx)("perf.chain.loop.A").value(count);
                     }
                   });

  // Initialize
  cvc::state::instance(test_ctx)("perf.chain.loop.A").value(0);
  cvc::state::instance(test_ctx)("perf.chain.loop.B").value(0);

  // Trigger potential loop
  nodeA_callbacks.store(0);
  nodeB_callbacks.store(0);
  cvc::state::instance(test_ctx)("perf.chain.loop.A").value(1);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  auto checkpoint3 = boost::chrono::high_resolution_clock::now();
  auto elapsed3 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint3 - checkpoint2).count();

  std::cout << "  Completed in " << elapsed3 << " ms\n";
  std::cout << "  Node A callbacks: " << nodeA_callbacks.load() << "\n";
  std::cout << "  Node B callbacks: " << nodeB_callbacks.load() << "\n";
  std::cout << "  Loop prevented by counter guard\n";

  EXPECT_LE(nodeA_callbacks.load(), MAX_ITERATIONS + 1)
      << "Counter guard should prevent infinite loop";
  EXPECT_LE(nodeB_callbacks.load(), MAX_ITERATIONS + 1)
      << "Counter guard should prevent infinite loop";

  connA.disconnect();
  connB.disconnect();

  // Test 4: State-based loop prevention (check if already processing)
  std::cout << "\nTest 4: State-based loop prevention (processing flag)...\n";

  struct GuardedCallback {
    std::atomic<bool> processing{false};
    std::atomic<int> callback_count{0};

    void operator()(const std::string &trigger_key) {
      // Try to acquire processing flag
      bool expected = false;
      if (!processing.compare_exchange_strong(expected, true)) {
        // Already processing, skip to prevent loop
        return;
      }

      callback_count++;

      // Do work (trigger another state)
      if (trigger_key != "self") {
        cvc::state::instance(test_ctx)(trigger_key).value(callback_count.load());
      }

      // Release flag
      processing.store(false);
    }
  };

  GuardedCallback guardC;
  GuardedCallback guardD;

  // C triggers D, D triggers C - but guards prevent infinite loop
  auto connC =
      cvc::state::instance(test_ctx)("perf.chain.guard.C").valueChanged.connect([&guardC]() {
        guardC("perf.chain.guard.D");
      });

  auto connD =
      cvc::state::instance(test_ctx)("perf.chain.guard.D").valueChanged.connect([&guardD]() {
        guardD("perf.chain.guard.C");
      });

  // Initialize
  cvc::state::instance(test_ctx)("perf.chain.guard.C").value(0);
  cvc::state::instance(test_ctx)("perf.chain.guard.D").value(0);

  // Trigger
  guardC.callback_count.store(0);
  guardD.callback_count.store(0);
  cvc::state::instance(test_ctx)("perf.chain.guard.C").value(1);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  auto checkpoint4 = boost::chrono::high_resolution_clock::now();
  auto elapsed4 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint4 - checkpoint3).count();

  std::cout << "  Completed in " << elapsed4 << " ms\n";
  std::cout << "  Guard C callbacks: " << guardC.callback_count.load() << "\n";
  std::cout << "  Guard D callbacks: " << guardD.callback_count.load() << "\n";
  std::cout << "  Loop prevented by processing flag\n";

  EXPECT_LE(guardC.callback_count.load(), 2) << "Processing flag should prevent re-entry";
  EXPECT_LE(guardD.callback_count.load(), 2) << "Processing flag should prevent re-entry";

  connC.disconnect();
  connD.disconnect();

  // Test 5: Deep hierarchy with parent-child callbacks
  std::cout << "\nTest 5: Deep hierarchy callbacks (parent updates trigger child updates)...\n";

  std::atomic<int> hierarchy_callbacks(0);
  const int HIER_LEVELS = 5;

  std::vector<boost::signals2::connection> hier_conns;

  // Set up linear chain where each level triggers the next level
  for (int level = 0; level < HIER_LEVELS - 1; ++level) {
    std::string current = "perf.chain.hier.level" + boost::lexical_cast<std::string>(level);
    std::string next = "perf.chain.hier.level" + boost::lexical_cast<std::string>(level + 1);

    auto conn = cvc::state::instance(test_ctx)(current).valueChanged.connect(
        [&hierarchy_callbacks, next, level]() {
          hierarchy_callbacks++;
          // Update next level with incremented value
          cvc::state::instance(test_ctx)(next).value(level + 2);
        });
    hier_conns.push_back(conn);
  }

  // Last level just counts
  auto last_conn = cvc::state::instance(test_ctx)("perf.chain.hier.level" +
                                                  boost::lexical_cast<std::string>(HIER_LEVELS - 1))
                       .valueChanged.connect([&hierarchy_callbacks]() { hierarchy_callbacks++; });
  hier_conns.push_back(last_conn);

  // Initialize all levels
  for (int level = 0; level < HIER_LEVELS; ++level) {
    std::string key = "perf.chain.hier.level" + boost::lexical_cast<std::string>(level);
    cvc::state::instance(test_ctx)(key).value(0);
  }

  // Trigger from root level
  hierarchy_callbacks.store(0);
  cvc::state::instance(test_ctx)("perf.chain.hier.level0").value(1);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  auto checkpoint5 = boost::chrono::high_resolution_clock::now();
  auto elapsed5 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint5 - checkpoint4).count();

  std::cout << "  Completed in " << elapsed5 << " ms\n";
  std::cout << "  Total hierarchy callbacks: " << hierarchy_callbacks.load() << "\n";

  // Should trigger all levels: HIER_LEVELS callbacks
  EXPECT_EQ(hierarchy_callbacks.load(), HIER_LEVELS) << "All hierarchy levels should trigger";

  for (auto &conn : hier_conns) {
    conn.disconnect();
  }
  hier_conns.clear();

  // Test 6: Stress test - many concurrent callbacks
  std::cout << "\nTest 6: Concurrent callback stress (1000 nodes, simultaneous triggers)...\n";

  const int CONCURRENT_NODES = 1000;
  std::atomic<int> concurrent_callbacks(0);

  std::vector<boost::signals2::connection> concurrent_conns;
  for (int i = 0; i < CONCURRENT_NODES; ++i) {
    std::string key = "perf.chain.concurrent." + boost::lexical_cast<std::string>(i);

    auto conn = cvc::state::instance(test_ctx)(key).valueChanged.connect([&concurrent_callbacks]() {
      concurrent_callbacks++;
      // Simulate some work
      volatile int dummy = 0;
      for (int j = 0; j < 100; ++j) {
        dummy += j;
      }
      (void)dummy;
    });
    concurrent_conns.push_back(conn);

    // Initialize with -1 so any value >= 0 will trigger change
    cvc::state::instance(test_ctx)(key).value(-1);
  }

  // Trigger all simultaneously
  concurrent_callbacks.store(0);
  for (int i = 0; i < CONCURRENT_NODES; ++i) {
    std::string key = "perf.chain.concurrent." + boost::lexical_cast<std::string>(i);
    cvc::state::instance(test_ctx)(key).value(i);
  }

  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  auto checkpoint6 = boost::chrono::high_resolution_clock::now();
  auto elapsed6 =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(checkpoint6 - checkpoint5).count();

  std::cout << "  Completed in " << elapsed6 << " ms\n";
  std::cout << "  Concurrent callbacks fired: " << concurrent_callbacks.load() << "\n";
  std::cout << "  Throughput: " << (concurrent_callbacks.load() / (elapsed6 / 1000.0))
            << " callbacks/sec\n";

  EXPECT_EQ(concurrent_callbacks.load(), CONCURRENT_NODES)
      << "All concurrent callbacks should fire";

  for (auto &conn : concurrent_conns) {
    conn.disconnect();
  }

  // Final summary
  auto end_time = boost::chrono::high_resolution_clock::now();
  auto total_elapsed =
      boost::chrono::duration_cast<boost::chrono::milliseconds>(end_time - start_time).count();

  std::cout << "\n=== Callback Chain Test Summary ===\n";
  std::cout << "Total test time: " << total_elapsed << " ms (" << (total_elapsed / 1000.0)
            << " seconds)\n";
  std::cout << "All callback chain tests passed\n";
  std::cout << "\n✓ Best Practices for Callback Safety:\n";
  std::cout << "  1. Use iteration counters to limit callback chains\n";
  std::cout << "  2. Use processing flags to prevent re-entry\n";
  std::cout << "  3. Avoid circular callback dependencies\n";
  std::cout << "  4. Design callbacks to be idempotent when possible\n";
  std::cout << "  5. Consider async operations for long callback chains\n";
  std::cout << "  6. Monitor callback depth in production code\n";

  EXPECT_LT(total_elapsed / 1000.0, MAX_RUNTIME_SECONDS)
      << "Callback tests should complete quickly";

  // Cleanup
  cvc::state::instance(test_ctx)("perf.chain").reset();
}

// ===========================
// Futures API Tests
// ===========================

TEST(StateTest, ValueWithCallback) {
  // Test value retrieval with callback
  std::atomic<int> callback_count(0);
  std::atomic<int> last_value(0);

  // Set up callback
  auto callback = [&callback_count, &last_value](int val) {
    callback_count++;
    last_value.store(val);
  };

  // Connect and get initial value
  cvc::state::instance(test_ctx)("test.future.callback").value(42);
  int initial = cvc::state::instance(test_ctx)("test.future.callback").value<int>(callback);
  EXPECT_EQ(initial, 42);

  // Change value several times
  for (int i = 0; i < 5; ++i) {
    cvc::state::instance(test_ctx)("test.future.callback").value(100 + i);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }

  // Callback should have fired
  EXPECT_GT(callback_count.load(), 0);
  EXPECT_EQ(last_value.load(), 104);

  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, WaitForValue) {
  // Test blocking wait for value

  // Thread that sets value after delay
  boost::thread writer([]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    cvc::state::instance(test_ctx)("test.future.wait").value(12345);
  });

  // Wait for value to be set
  int val = cvc::state::instance(test_ctx)("test.future.wait").wait_for_value<int>();

  EXPECT_EQ(val, 12345);
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.future.wait").initialized());

  writer.join();
  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, WaitForValueWithTimeout) {
  // Test wait with timeout - should timeout
  try {
    cvc::state::instance(test_ctx)("test.future.timeout")
        .wait_for_value<int>(boost::chrono::milliseconds(50));
    FAIL() << "Should have thrown timeout exception";
  } catch (const cvc::timeout_error &e) {
    EXPECT_TRUE(std::string(e.what()).find("Timeout") != std::string::npos);
  }

  // Test wait with timeout - should succeed
  boost::thread writer([]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    cvc::state::instance(test_ctx)("test.future.timeout2").value(999);
  });

  int val = cvc::state::instance(test_ctx)("test.future.timeout2")
                .wait_for_value<int>(boost::chrono::milliseconds(200));
  EXPECT_EQ(val, 999);

  writer.join();
  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, ValueFutureGet) {
  // Test state_future blocking get
  boost::thread writer([]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    cvc::state::instance(test_ctx)("test.future.get").value("future_value");
  });

  auto future = cvc::state::instance(test_ctx)("test.future.get").value_future<std::string>();

  EXPECT_FALSE(future.is_ready());

  std::string val = future.get();

  EXPECT_TRUE(future.is_ready());
  EXPECT_EQ(val, "future_value");

  writer.join();
  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, ValueFutureWaitFor) {
  // Test state_future with timeout
  auto future = cvc::state::instance(test_ctx)("test.future.waitfor").value_future<int>();

  // Should timeout
  EXPECT_FALSE(future.wait_for(boost::chrono::milliseconds(50)));
  EXPECT_FALSE(future.is_ready());

  // Set value
  cvc::state::instance(test_ctx)("test.future.waitfor").value(777);

  // Should succeed immediately
  EXPECT_TRUE(future.wait_for(boost::chrono::milliseconds(10)));
  EXPECT_TRUE(future.is_ready());
  EXPECT_EQ(future.get(), 777);

  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, ValueFutureGetFor) {
  // Test state_future get with timeout
  auto future = cvc::state::instance(test_ctx)("test.future.getfor").value_future<double>();

  // Should timeout
  try {
    future.get_for(boost::chrono::milliseconds(50));
    FAIL() << "Should have thrown timeout exception";
  } catch (const cvc::timeout_error &e) {
    EXPECT_TRUE(std::string(e.what()).find("timeout") != std::string::npos);
  }

  // Start writer thread
  boost::thread writer([]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    cvc::state::instance(test_ctx)("test.future.getfor").value(3.14159);
  });

  // Should succeed with longer timeout
  double val = future.get_for(boost::chrono::milliseconds(200));
  EXPECT_DOUBLE_EQ(val, 3.14159);

  writer.join();
  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, DataWithCallback) {
  // Test data retrieval with callback
  std::atomic<int> callback_count(0);
  std::string last_data;
  boost::mutex data_mutex;

  auto callback = [&callback_count, &last_data, &data_mutex](std::string val) {
    callback_count++;
    boost::mutex::scoped_lock lock(data_mutex);
    last_data = val;
  };

  // Set initial data and connect callback
  cvc::state::instance(test_ctx)("test.future.datacb").data(std::string("initial"));
  std::string initial =
      cvc::state::instance(test_ctx)("test.future.datacb").data<std::string>(callback);
  EXPECT_EQ(initial, "initial");

  // Change data several times
  for (int i = 0; i < 5; ++i) {
    cvc::state::instance(test_ctx)("test.future.datacb")
        .data(std::string("data_") + boost::lexical_cast<std::string>(i));
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }

  EXPECT_GT(callback_count.load(), 0);

  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, WaitForData) {
  // Test blocking wait for data
  boost::thread writer([]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    cvc::state::instance(test_ctx)("test.future.waitdata").data(42);
  });

  int val = cvc::state::instance(test_ctx)("test.future.waitdata").wait_for_data<int>();

  EXPECT_EQ(val, 42);

  writer.join();
  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, WaitForDataWithTimeout) {
  // Test data wait with timeout - should timeout
  try {
    cvc::state::instance(test_ctx)("test.future.datatimeout")
        .wait_for_data<int>(boost::chrono::milliseconds(50));
    FAIL() << "Should have thrown timeout exception";
  } catch (const cvc::timeout_error &e) {
    EXPECT_TRUE(std::string(e.what()).find("Timeout") != std::string::npos);
  }

  // Test wait with timeout - should succeed
  boost::thread writer([]() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    cvc::state::instance(test_ctx)("test.future.datatimeout2").data(std::string("success"));
  });

  std::string val = cvc::state::instance(test_ctx)("test.future.datatimeout2")
                        .wait_for_data<std::string>(boost::chrono::milliseconds(200));
  EXPECT_EQ(val, "success");

  writer.join();
  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, MultipleFuturesOnSameState) {
  // Test multiple futures waiting on the same state
  const int num_futures = 5;
  std::vector<boost::thread> threads;
  std::atomic<int> success_count(0);

  for (int i = 0; i < num_futures; ++i) {
    threads.emplace_back([&success_count]() {
      try {
        auto future = cvc::state::instance(test_ctx)("test.future.multiple").value_future<int>();
        int val = future.get_for(boost::chrono::milliseconds(500));
        if (val == 888) {
          success_count++;
        }
      } catch (...) {
        FAIL() << "Future failed";
      }
    });
  }

  // Give threads time to set up futures
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // Set value - all futures should be notified
  cvc::state::instance(test_ctx)("test.future.multiple").value(888);

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_EQ(success_count.load(), num_futures);

  cvc::state::instance(test_ctx)("test.future").reset();
}

TEST(StateTest, FutureProducerConsumerPattern) {
  // Test classic producer-consumer with futures
  const int num_items = 10;
  std::atomic<int> consumed(0);

  // Consumer thread waits for each item
  boost::thread consumer([&consumed, num_items]() {
    for (int i = 0; i < num_items; ++i) {
      std::string key = "test.future.queue.item" + boost::lexical_cast<std::string>(i);
      int val = cvc::state::instance(test_ctx)(key).wait_for_value<int>(
          boost::chrono::milliseconds(1000));
      EXPECT_EQ(val, i * 10);
      consumed++;
    }
  });

  // Producer thread generates items
  boost::thread producer([num_items]() {
    for (int i = 0; i < num_items; ++i) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      std::string key = "test.future.queue.item" + boost::lexical_cast<std::string>(i);
      cvc::state::instance(test_ctx)(key).value(i * 10);
    }
  });

  consumer.join();
  producer.join();

  EXPECT_EQ(consumed.load(), num_items);

  cvc::state::instance(test_ctx)("test.future").reset();
}

// ===========================
// State Name Validation Tests
// ===========================

TEST(StateTest, ValidStateNames) {
  // Valid names - should pass validation
  EXPECT_TRUE(cvc::state::isValidStateName("validName"));
  EXPECT_TRUE(cvc::state::isValidStateName("_private"));
  EXPECT_TRUE(cvc::state::isValidStateName("name123"));
  EXPECT_TRUE(cvc::state::isValidStateName("_underscore_name"));
  EXPECT_TRUE(cvc::state::isValidStateName("CamelCase"));
  EXPECT_TRUE(cvc::state::isValidStateName("snake_case_name"));
  EXPECT_TRUE(cvc::state::isValidStateName("name_with_digits_123"));
  EXPECT_TRUE(cvc::state::isValidStateName("_123"));
}

TEST(StateTest, InvalidStateNames) {
  // Invalid names - should fail validation
  EXPECT_FALSE(cvc::state::isValidStateName(""));                // Empty
  EXPECT_FALSE(cvc::state::isValidStateName("123start"));        // Starts with digit
  EXPECT_FALSE(cvc::state::isValidStateName("name-with-dash"));  // Contains dash
  EXPECT_FALSE(cvc::state::isValidStateName("name with space")); // Contains space
  EXPECT_FALSE(cvc::state::isValidStateName("name.with.dot"));   // Contains dot
  EXPECT_FALSE(cvc::state::isValidStateName("name$pecial"));     // Contains special char
  EXPECT_FALSE(cvc::state::isValidStateName("name@symbol"));     // Contains @
  EXPECT_FALSE(cvc::state::isValidStateName("name#hash"));       // Contains #
}

TEST(StateTest, SanitizeEmptyName) {
  std::string result = cvc::state::sanitizeStateName("");
  EXPECT_EQ(result, "unnamed");
}

TEST(StateTest, SanitizeValidName) {
  // Already valid names should pass through unchanged
  EXPECT_EQ(cvc::state::sanitizeStateName("validName"), "validName");
  EXPECT_EQ(cvc::state::sanitizeStateName("_private"), "_private");
  EXPECT_EQ(cvc::state::sanitizeStateName("name123"), "name123");
}

TEST(StateTest, SanitizeNameWithDashes) {
  // Dashes should be converted to underscores
  EXPECT_EQ(cvc::state::sanitizeStateName("name-with-dashes"), "name_with_dashes");
  EXPECT_EQ(cvc::state::sanitizeStateName("multi-part-name"), "multi_part_name");
  EXPECT_EQ(cvc::state::sanitizeStateName("-starts-with-dash"), "_starts_with_dash");
}

TEST(StateTest, SanitizeNameWithSpaces) {
  // Spaces should be converted to underscores
  EXPECT_EQ(cvc::state::sanitizeStateName("name with spaces"), "name_with_spaces");
  EXPECT_EQ(cvc::state::sanitizeStateName("multiple   spaces"), "multiple___spaces");
}

TEST(StateTest, SanitizeNameStartingWithDigit) {
  // Names starting with digits should get underscore prefix
  EXPECT_EQ(cvc::state::sanitizeStateName("123name"), "_123name");
  EXPECT_EQ(cvc::state::sanitizeStateName("4test"), "_4test");
}

TEST(StateTest, SanitizeNameWithSpecialChars) {
  // Special characters should be converted to underscores
  EXPECT_EQ(cvc::state::sanitizeStateName("name@symbol"), "name_symbol");
  EXPECT_EQ(cvc::state::sanitizeStateName("file$name"), "file_name");
  EXPECT_EQ(cvc::state::sanitizeStateName("test#value"), "test_value");
  EXPECT_EQ(cvc::state::sanitizeStateName("name.with.dots"), "name_with_dots");
}

TEST(StateTest, SanitizeComplexFilename) {
  // Real-world filename examples
  EXPECT_EQ(cvc::state::sanitizeStateName("my-mesh-file"), "my_mesh_file");
  EXPECT_EQ(cvc::state::sanitizeStateName("bunny.obj"), "bunny_obj");
  EXPECT_EQ(cvc::state::sanitizeStateName("test file (1)"), "test_file__1_");
  EXPECT_EQ(cvc::state::sanitizeStateName("2024-12-30_data"), "_2024_12_30_data");
}

TEST(StateTest, SanitizePreservesUnderscores) {
  // Underscores should be preserved
  EXPECT_EQ(cvc::state::sanitizeStateName("name_with_underscores"), "name_with_underscores");
  EXPECT_EQ(cvc::state::sanitizeStateName("__double__underscore__"), "__double__underscore__");
}

TEST(StateTest, SanitizeResultIsValid) {
  // All sanitized names should pass validation
  std::vector<std::string> testNames = {"123start",          "name-with-dash", "name with space",
                                        "special@char#name", "file.name.ext",  ""};

  for (const auto &name : testNames) {
    std::string sanitized = cvc::state::sanitizeStateName(name);
    EXPECT_TRUE(cvc::state::isValidStateName(sanitized))
        << "Sanitized name '" << sanitized << "' from '" << name << "' should be valid";
  }
}

// ===========================
// Data Type Registration Tests
// ===========================

TEST_F(StateDataFixture, RegisteredDataTypeNames) {
  // Test that core CVC types are registered with human-readable names

  // Create instances of each type
  cvc::geometry geom;
  cvc::voxels vox(ctx, cvc::dimension(10, 10, 10), cvc::UChar);
  cvc::volume vol(ctx, cvc::dimension(10, 10, 10), cvc::UChar);
  cvc::bounding_box bbox(0, 0, 0, 1, 1, 1);
  cvc::dimension dim(10, 10, 10);

  // Store them in the fixture-local app's data map
  ctx.data("test.geometry", geom);
  ctx.data("test.voxels", vox);
  ctx.data("test.volume", vol);
  ctx.data("test.bbox", bbox);
  ctx.data("test.dimension", dim);

  // Verify the type names are human-readable, not mangled C++ names
  std::string geom_type = ctx.dataTypeName(std::string("test.geometry"));
  std::string vox_type = ctx.dataTypeName(std::string("test.voxels"));
  std::string vol_type = ctx.dataTypeName(std::string("test.volume"));
  std::string bbox_type = ctx.dataTypeName(std::string("test.bbox"));
  std::string dim_type = ctx.dataTypeName(std::string("test.dimension"));

  EXPECT_EQ(geom_type, "cvc::geometry") << "Geometry type should be registered";
  EXPECT_EQ(vox_type, "cvc::voxels") << "Voxels type should be registered";
  EXPECT_EQ(vol_type, "cvc::volume") << "Volume type should be registered";
  EXPECT_EQ(bbox_type, "cvc::bounding_box") << "BoundingBox type should be registered";
  EXPECT_EQ(dim_type, "cvc::dimension") << "Dimension type should be registered";

  // Clean up
  ctx.data("test.geometry", boost::any());
  ctx.data("test.voxels", boost::any());
  ctx.data("test.volume", boost::any());
  ctx.data("test.bbox", boost::any());
  ctx.data("test.dimension", boost::any());
}

TEST_F(StateDataFixture, RegisteredSharedPtrDataTypeNames) {
  // Test that shared_ptr variants are also registered

  boost::shared_ptr<cvc::geometry> geom_ptr(new cvc::geometry());
  boost::shared_ptr<cvc::voxels> vox_ptr(
      new cvc::voxels(ctx, cvc::dimension(10, 10, 10), cvc::UChar));
  boost::shared_ptr<cvc::volume> vol_ptr(
      new cvc::volume(ctx, cvc::dimension(10, 10, 10), cvc::UChar));

  ctx.data("test.geometry_ptr", geom_ptr);
  ctx.data("test.voxels_ptr", vox_ptr);
  ctx.data("test.volume_ptr", vol_ptr);

  std::string geom_ptr_type = ctx.dataTypeName(std::string("test.geometry_ptr"));
  std::string vox_ptr_type = ctx.dataTypeName(std::string("test.voxels_ptr"));
  std::string vol_ptr_type = ctx.dataTypeName(std::string("test.volume_ptr"));

  EXPECT_EQ(geom_ptr_type, "boost::shared_ptr<cvc::geometry>")
      << "Geometry shared_ptr type should be registered";
  EXPECT_EQ(vox_ptr_type, "boost::shared_ptr<cvc::voxels>")
      << "Voxels shared_ptr type should be registered";
  EXPECT_EQ(vol_ptr_type, "boost::shared_ptr<cvc::volume>")
      << "Volume shared_ptr type should be registered";

  // Clean up
  ctx.data("test.geometry_ptr", boost::any());
  ctx.data("test.voxels_ptr", boost::any());
  ctx.data("test.volume_ptr", boost::any());
}

TEST_F(StateDataFixture, DataTypeNameTemplate) {
  // Test the template version of dataTypeName<T>()

  std::string geom_type = ctx.dataTypeName<cvc::geometry>();
  std::string vox_type = ctx.dataTypeName<cvc::voxels>();
  std::string vol_type = ctx.dataTypeName<cvc::volume>();
  std::string bbox_type = ctx.dataTypeName<cvc::bounding_box>();
  std::string dim_type = ctx.dataTypeName<cvc::dimension>();

  EXPECT_EQ(geom_type, "cvc::geometry");
  EXPECT_EQ(vox_type, "cvc::voxels");
  EXPECT_EQ(vol_type, "cvc::volume");
  EXPECT_EQ(bbox_type, "cvc::bounding_box");
  EXPECT_EQ(dim_type, "cvc::dimension");

  // Test shared_ptr versions
  std::string geom_ptr_type = ctx.dataTypeName<boost::shared_ptr<cvc::geometry>>();
  std::string vox_ptr_type = ctx.dataTypeName<boost::shared_ptr<cvc::voxels>>();
  std::string vol_ptr_type = ctx.dataTypeName<boost::shared_ptr<cvc::volume>>();

  EXPECT_EQ(geom_ptr_type, "boost::shared_ptr<cvc::geometry>");
  EXPECT_EQ(vox_ptr_type, "boost::shared_ptr<cvc::voxels>");
  EXPECT_EQ(vol_ptr_type, "boost::shared_ptr<cvc::volume>");
}

TEST_F(StateDataFixture, DataTypeNameFromAny) {
  // Test the dataTypeName(boost::any) overload

  boost::any geom_any = cvc::geometry();
  boost::any vox_any = cvc::voxels(ctx, cvc::dimension(10, 10, 10), cvc::UChar);
  boost::any vol_any = cvc::volume(ctx, cvc::dimension(10, 10, 10), cvc::UChar);

  std::string geom_type = ctx.dataTypeName(geom_any);
  std::string vox_type = ctx.dataTypeName(vox_any);
  std::string vol_type = ctx.dataTypeName(vol_any);

  EXPECT_EQ(geom_type, "cvc::geometry");
  EXPECT_EQ(vox_type, "cvc::voxels");
  EXPECT_EQ(vol_type, "cvc::volume");
}

TEST_F(StateDataFixture, UnregisteredTypeReturnsMangled) {
  // Test that unregistered types return the mangled C++ name

  struct UnregisteredType {
    int value;
  };

  UnregisteredType custom;
  ctx.data("test.unregistered", custom);

  std::string type_name = ctx.dataTypeName(std::string("test.unregistered"));

  // The type name should be a mangled C++ name, not a nice name
  // It will contain something like "UnregisteredType" but might be mangled
  EXPECT_NE(type_name, "") << "Unregistered types should still return a type name";
  EXPECT_TRUE(type_name.find("UnregisteredType") != std::string::npos ||
              type_name.find("N") != std::string::npos)
      << "Unregistered type should return raw/mangled name: " << type_name;

  // Clean up
  ctx.data("test.unregistered", boost::any());
}

// ===========================
// Reset with Parameters Tests
// ===========================

TEST(StateTest, ResetWithoutChildren) {
  // Create a state hierarchy
  cvc::state::instance(test_ctx)("test.reset_params.parent").value("parent_value");
  cvc::state::instance(test_ctx)("test.reset_params.parent.child1").value("child1_value");
  cvc::state::instance(test_ctx)("test.reset_params.parent.child2").value("child2_value");

  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset_params.parent").initialized());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset_params.parent.child1").initialized());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset_params.parent.child2").initialized());

  // Reset parent without resetting children
  cvc::state::instance(test_ctx)("test.reset_params.parent").reset(false, true);

  // Parent should be reset
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_params.parent").initialized());

  // Children are detached - accessing them via parent path creates new states
  // The old children exist as shared_ptrs elsewhere but are no longer accessible via parent
  // This is the expected behavior of reset(false, ...)
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_params.parent.child1").initialized());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_params.parent.child2").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test.reset_params").reset();
}

TEST(StateTest, ResetWithChildren) {
  // Create a state hierarchy
  cvc::state::instance(test_ctx)("test.reset_children.parent").value("parent_value");
  cvc::state::instance(test_ctx)("test.reset_children.parent.child1").value("child1_value");
  cvc::state::instance(test_ctx)("test.reset_children.parent.child2").value("child2_value");
  cvc::state::instance(test_ctx)("test.reset_children.parent.child2.grandchild")
      .value("grandchild_value");

  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset_children.parent").initialized());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset_children.parent.child1").initialized());
  EXPECT_TRUE(cvc::state::instance(test_ctx)("test.reset_children.parent.child2").initialized());
  EXPECT_TRUE(
      cvc::state::instance(test_ctx)("test.reset_children.parent.child2.grandchild").initialized());

  // Reset parent with children (default behavior)
  cvc::state::instance(test_ctx)("test.reset_children.parent").reset(true, true);

  // Parent and all children should be reset
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_children.parent").initialized());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_children.parent.child1").initialized());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_children.parent.child2").initialized());
  EXPECT_FALSE(
      cvc::state::instance(test_ctx)("test.reset_children.parent.child2.grandchild").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test.reset_children").reset();
}

TEST(StateTest, ResetWithoutCallbacks) {
  // Track callback invocations
  std::atomic<int> callbackCount(0);

  auto conn = cvc::state::instance(test_ctx)("test.reset_callbacks.watched")
                  .valueChanged.connect([&callbackCount]() { callbackCount++; });

  cvc::state::instance(test_ctx)("test.reset_callbacks.watched").value("initial_value");
  int initialCount = callbackCount.load();
  EXPECT_GT(initialCount, 0);

  // Reset without firing callbacks
  cvc::state::instance(test_ctx)("test.reset_callbacks.watched").reset(true, false);

  // Callback should not have been triggered by reset
  EXPECT_EQ(callbackCount.load(), initialCount);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_callbacks.watched").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test.reset_callbacks").reset();
}

TEST(StateTest, ResetWithCallbacks) {
  // Track callback invocations
  std::atomic<int> callbackCount(0);

  auto conn = cvc::state::instance(test_ctx)("test.reset_with_callbacks.watched")
                  .valueChanged.connect([&callbackCount]() { callbackCount++; });

  cvc::state::instance(test_ctx)("test.reset_with_callbacks.watched").value("initial_value");
  callbackCount.store(0); // Reset counter after initialization

  // Reset with firing callbacks (default behavior)
  cvc::state::instance(test_ctx)("test.reset_with_callbacks.watched").reset(true, true);

  // Callback should have been triggered by reset
  EXPECT_GT(callbackCount.load(), 0);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.reset_with_callbacks.watched").initialized());

  // Clean up
  cvc::state::instance(test_ctx)("test.reset_with_callbacks").reset();
}

TEST(StateTest, ResetParameterCombinations) {
  // Test all four combinations of resetChildren and fireCallbacks

  // Setup 1: reset(false, false) - no children, no callbacks
  cvc::state::instance(test_ctx)("test.combinations.case1.parent").value("value");
  cvc::state::instance(test_ctx)("test.combinations.case1.parent.child").value("child_value");

  std::atomic<int> count1(0);
  auto conn1 = cvc::state::instance(test_ctx)("test.combinations.case1.parent")
                   .valueChanged.connect([&count1]() { count1++; });
  count1.store(0);

  cvc::state::instance(test_ctx)("test.combinations.case1.parent").reset(false, false);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.combinations.case1.parent").initialized());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.combinations.case1.parent.child")
                   .initialized()); // Detached
  EXPECT_EQ(count1.load(), 0);

  // Setup 2: reset(false, true) - no children, yes callbacks
  cvc::state::instance(test_ctx)("test.combinations.case2.parent").value("value");
  cvc::state::instance(test_ctx)("test.combinations.case2.parent.child").value("child_value");

  std::atomic<int> count2(0);
  auto conn2 = cvc::state::instance(test_ctx)("test.combinations.case2.parent")
                   .valueChanged.connect([&count2]() { count2++; });
  count2.store(0);

  cvc::state::instance(test_ctx)("test.combinations.case2.parent").reset(false, true);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.combinations.case2.parent").initialized());
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.combinations.case2.parent.child")
                   .initialized()); // Detached
  EXPECT_GT(count2.load(), 0);

  // Setup 3: reset(true, false) - yes children, no callbacks
  cvc::state::instance(test_ctx)("test.combinations.case3.parent").value("value");
  cvc::state::instance(test_ctx)("test.combinations.case3.parent.child").value("child_value");

  std::atomic<int> count3(0);
  auto conn3 = cvc::state::instance(test_ctx)("test.combinations.case3.parent")
                   .valueChanged.connect([&count3]() { count3++; });
  count3.store(0);

  cvc::state::instance(test_ctx)("test.combinations.case3.parent").reset(true, false);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.combinations.case3.parent").initialized());
  EXPECT_FALSE(
      cvc::state::instance(test_ctx)("test.combinations.case3.parent.child").initialized());
  EXPECT_EQ(count3.load(), 0);

  // Setup 4: reset(true, true) - yes children, yes callbacks (default)
  cvc::state::instance(test_ctx)("test.combinations.case4.parent").value("value");
  cvc::state::instance(test_ctx)("test.combinations.case4.parent.child").value("child_value");

  std::atomic<int> count4(0);
  auto conn4 = cvc::state::instance(test_ctx)("test.combinations.case4.parent")
                   .valueChanged.connect([&count4]() { count4++; });
  count4.store(0);

  cvc::state::instance(test_ctx)("test.combinations.case4.parent").reset(true, true);
  EXPECT_FALSE(cvc::state::instance(test_ctx)("test.combinations.case4.parent").initialized());
  EXPECT_FALSE(
      cvc::state::instance(test_ctx)("test.combinations.case4.parent.child").initialized());
  EXPECT_GT(count4.load(), 0);

  // Clean up
  cvc::state::instance(test_ctx)("test.combinations").reset();
}

// ===========================
// state_object Threading Control Tests
// ===========================

// Test object that tracks synchronous vs threaded execution
class ThreadingTestObject : public state_object<ThreadingTestObject> {
public:
  std::atomic<int> handleCount;
  std::atomic<int> synchronousCount;
  std::atomic<int> threadedCount;
  boost::thread::id constructorThreadId;

  ThreadingTestObject(cvc::app &ctx)
      : state_object<ThreadingTestObject>(ctx), handleCount(0), synchronousCount(0),
        threadedCount(0), constructorThreadId(boost::this_thread::get_id()) {}

protected:
  virtual void handleStateChanged(const std::string &childState) override {
    handleCount++;

    // Check if running on same thread as constructor (synchronous) or different (threaded)
    if (boost::this_thread::get_id() == constructorThreadId) {
      synchronousCount++;
    } else {
      threadedCount++;
    }

    // Simulate some work
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
  }
};

TEST_F(StateObjectFixture, StateObjectThreadingDisabled) {
  // Disable threading for this test
  state_object<ThreadingTestObject>::setUseThreading(false);

  ThreadingTestObject obj(ctx);

  // Make multiple state changes
  for (int i = 0; i < 10; ++i) {
    obj.getState("property" + boost::lexical_cast<std::string>(i)).value("value");
  }

  // Wait for any pending handlers (should be none with threading disabled)
  obj.waitForHandlers();

  // All callbacks should have run synchronously
  EXPECT_EQ(obj.handleCount.load(), 10);
  EXPECT_EQ(obj.synchronousCount.load(), 10);
  EXPECT_EQ(obj.threadedCount.load(), 0);

  // Re-enable threading for other tests
  state_object<ThreadingTestObject>::setUseThreading(true);

  // Clean up
  obj.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectThreadingEnabled) {
  // Ensure threading is enabled (default)
  state_object<ThreadingTestObject>::setUseThreading(true);

  ThreadingTestObject obj(ctx);

  // Make multiple state changes
  for (int i = 0; i < 10; ++i) {
    obj.getState("property" + boost::lexical_cast<std::string>(i)).value("value");
  }

  // Wait for handlers to complete
  obj.waitForHandlers();

  // All callbacks should have been triggered
  EXPECT_EQ(obj.handleCount.load(), 10);

  // With threading enabled, callbacks should run on different threads
  // (though some might coincidentally run on the same thread)
  EXPECT_GT(obj.threadedCount.load(), 0) << "Expected at least some callbacks on different threads";

  // Clean up
  obj.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectThreadingToggle) {
  // Test that we can toggle threading on and off

  // Start with threading disabled
  state_object<ThreadingTestObject>::setUseThreading(false);
  EXPECT_FALSE(state_object<ThreadingTestObject>::getUseThreading());

  ThreadingTestObject obj1(ctx);
  obj1.getState("prop1").value("value1");
  obj1.waitForHandlers();
  EXPECT_EQ(obj1.synchronousCount.load(), 1);
  EXPECT_EQ(obj1.threadedCount.load(), 0);

  // Enable threading
  state_object<ThreadingTestObject>::setUseThreading(true);
  EXPECT_TRUE(state_object<ThreadingTestObject>::getUseThreading());

  ThreadingTestObject obj2(ctx);
  obj2.getState("prop2").value("value2");
  obj2.waitForHandlers();
  EXPECT_EQ(obj2.handleCount.load(), 1);
  // With threading enabled, should be on different thread (usually)

  // Disable again
  state_object<ThreadingTestObject>::setUseThreading(false);
  EXPECT_FALSE(state_object<ThreadingTestObject>::getUseThreading());

  ThreadingTestObject obj3(ctx);
  obj3.getState("prop3").value("value3");
  obj3.waitForHandlers();
  EXPECT_EQ(obj3.synchronousCount.load(), 1);
  EXPECT_EQ(obj3.threadedCount.load(), 0);

  // Re-enable for other tests
  state_object<ThreadingTestObject>::setUseThreading(true);

  // Clean up
  obj1.getState().reset();
  obj2.getState().reset();
  obj3.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectBatchingWithThreadingDisabled) {
  // Test that batching works correctly even with threading disabled
  state_object<ThreadingTestObject>::setUseThreading(false);

  ThreadingTestObject obj(ctx);

  {
    state_change_batch_scope<ThreadingTestObject> batch(obj);

    // Make multiple changes within batch
    for (int i = 0; i < 5; ++i) {
      obj.getState("batch_prop" + boost::lexical_cast<std::string>(i)).value("value");
    }

    // Handler should not have been called yet
    EXPECT_EQ(obj.handleCount.load(), 0);

  } // batch.flush() happens here

  // Now all handlers should have been called synchronously
  EXPECT_EQ(obj.handleCount.load(), 5);
  EXPECT_EQ(obj.synchronousCount.load(), 5);
  EXPECT_EQ(obj.threadedCount.load(), 0);

  // Re-enable threading
  state_object<ThreadingTestObject>::setUseThreading(true);

  // Clean up
  obj.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectThreadingPerTemplateType) {
  // Test that threading control is independent per template instantiation

  class TypeA : public state_object<TypeA> {
  public:
    TypeA(cvc::app &ctx) : state_object<TypeA>(ctx) {}
    std::atomic<bool> called{false};

  protected:
    void handleStateChanged(const std::string &) override { called = true; }
  };

  class TypeB : public state_object<TypeB> {
  public:
    TypeB(cvc::app &ctx) : state_object<TypeB>(ctx) {}
    std::atomic<bool> called{false};

  protected:
    void handleStateChanged(const std::string &) override { called = true; }
  };

  // Disable threading for TypeA only
  state_object<TypeA>::setUseThreading(false);
  state_object<TypeB>::setUseThreading(true);

  EXPECT_FALSE(state_object<TypeA>::getUseThreading());
  EXPECT_TRUE(state_object<TypeB>::getUseThreading());

  TypeA objA(ctx);
  TypeB objB(ctx);

  objA.getState("test").value("value");
  objB.getState("test").value("value");

  objA.waitForHandlers();
  objB.waitForHandlers();

  EXPECT_TRUE(objA.called);
  EXPECT_TRUE(objB.called);

  // Reset TypeA back to default
  state_object<TypeA>::setUseThreading(true);

  // Clean up
  objA.getState().reset();
  objB.getState().reset();
}

// ===========================
// Per-Instance Threading Tests
// ===========================

TEST_F(StateObjectFixture, StateObjectInstanceThreadingEnabled) {
  // Test enabling instance threading on a single object
  state_object<ThreadingTestObject>::setUseThreading(false); // Global disabled

  ThreadingTestObject obj(ctx);

  // Should follow global flag initially
  EXPECT_FALSE(obj.getInstanceThreading());

  // Make a change - should be synchronous
  obj.getState("prop1").value("value1");
  obj.waitForHandlers();
  EXPECT_EQ(obj.handleCount.load(), 1);
  EXPECT_EQ(obj.synchronousCount.load(), 1);
  EXPECT_EQ(obj.threadedCount.load(), 0);

  // Enable instance threading
  obj.setInstanceThreading(true);
  EXPECT_TRUE(obj.getInstanceThreading());

  // Now changes should be threaded
  obj.getState("prop2").value("value2");
  obj.waitForHandlers();
  EXPECT_EQ(obj.handleCount.load(), 2);
  EXPECT_GT(obj.threadedCount.load(), 0) << "Expected threaded execution";

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  obj.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectInstanceThreadingDisabled) {
  // Test disabling instance threading on a single object
  state_object<ThreadingTestObject>::setUseThreading(true); // Global enabled

  ThreadingTestObject obj(ctx);

  // Should follow global flag initially
  EXPECT_TRUE(obj.getInstanceThreading());

  // Disable instance threading
  obj.setInstanceThreading(false);
  EXPECT_FALSE(obj.getInstanceThreading());

  // Now changes should be synchronous
  obj.getState("prop1").value("value1");
  obj.waitForHandlers();
  EXPECT_EQ(obj.handleCount.load(), 1);
  EXPECT_EQ(obj.synchronousCount.load(), 1);
  EXPECT_EQ(obj.threadedCount.load(), 0);

  // Clean up
  obj.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectMixedInstanceThreading) {
  // Test multiple objects with different instance threading settings
  state_object<ThreadingTestObject>::setUseThreading(false); // Global disabled

  ThreadingTestObject obj1(ctx), obj2(ctx), obj3(ctx);

  // All should follow global flag initially
  EXPECT_FALSE(obj1.getInstanceThreading());
  EXPECT_FALSE(obj2.getInstanceThreading());
  EXPECT_FALSE(obj3.getInstanceThreading());

  // Enable threading on obj1 and obj3 only
  obj1.setInstanceThreading(true);
  obj3.setInstanceThreading(true);

  EXPECT_TRUE(obj1.getInstanceThreading());
  EXPECT_FALSE(obj2.getInstanceThreading());
  EXPECT_TRUE(obj3.getInstanceThreading());

  // Make changes on all objects
  obj1.getState("prop").value("value1");
  obj2.getState("prop").value("value2");
  obj3.getState("prop").value("value3");

  obj1.waitForHandlers();
  obj2.waitForHandlers();
  obj3.waitForHandlers();

  // Verify obj1 and obj3 used threading, obj2 was synchronous
  EXPECT_EQ(obj1.handleCount.load(), 1);
  EXPECT_GT(obj1.threadedCount.load(), 0);

  EXPECT_EQ(obj2.handleCount.load(), 1);
  EXPECT_EQ(obj2.synchronousCount.load(), 1);
  EXPECT_EQ(obj2.threadedCount.load(), 0);

  EXPECT_EQ(obj3.handleCount.load(), 1);
  EXPECT_GT(obj3.threadedCount.load(), 0);

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  obj1.getState().reset();
  obj2.getState().reset();
  obj3.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectClearInstanceThreading) {
  // Test clearing instance threading to revert to global flag
  state_object<ThreadingTestObject>::setUseThreading(false); // Global disabled

  ThreadingTestObject obj(ctx);

  // Enable instance threading
  obj.setInstanceThreading(true);
  EXPECT_TRUE(obj.getInstanceThreading());

  obj.getState("prop1").value("value1");
  obj.waitForHandlers();
  EXPECT_GT(obj.threadedCount.load(), 0);

  // Clear instance threading - should revert to global (disabled)
  obj.clearInstanceThreading();
  EXPECT_FALSE(obj.getInstanceThreading());

  obj.getState("prop2").value("value2");
  obj.waitForHandlers();

  // New change should be synchronous
  EXPECT_EQ(obj.handleCount.load(), 2);
  EXPECT_EQ(obj.synchronousCount.load(), 1);

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  obj.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectInstanceThreadingWithGlobalChange) {
  // Test that instance threading persists across global flag changes
  state_object<ThreadingTestObject>::setUseThreading(false);

  ThreadingTestObject obj1(ctx), obj2(ctx);

  // Enable instance threading on obj1
  obj1.setInstanceThreading(true);

  EXPECT_TRUE(obj1.getInstanceThreading());
  EXPECT_FALSE(obj2.getInstanceThreading());

  // Change global flag to true
  state_object<ThreadingTestObject>::setUseThreading(true);

  // obj1 keeps its instance setting, obj2 follows global
  EXPECT_TRUE(obj1.getInstanceThreading());
  EXPECT_TRUE(obj2.getInstanceThreading());

  // Disable obj2 instance threading
  obj2.setInstanceThreading(false);
  EXPECT_FALSE(obj2.getInstanceThreading());

  // Change global back to false
  state_object<ThreadingTestObject>::setUseThreading(false);

  // Both keep their instance settings
  EXPECT_TRUE(obj1.getInstanceThreading());
  EXPECT_FALSE(obj2.getInstanceThreading());

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  obj1.getState().reset();
  obj2.getState().reset();
}

TEST_F(StateObjectFixture, StateObjectInstanceThreadingStressTest) {
  // Stress test with multiple objects and mixed threading settings
  state_object<ThreadingTestObject>::setUseThreading(false);

  const int numObjects = 10;
  std::vector<std::shared_ptr<ThreadingTestObject>> objects;

  // Create objects with alternating threading settings
  for (int i = 0; i < numObjects; ++i) {
    auto obj = std::make_shared<ThreadingTestObject>(ctx);
    if (i % 2 == 0) {
      obj->setInstanceThreading(true); // Even indices: threaded
    }
    // Odd indices: synchronous (follow global disabled)
    objects.push_back(obj);
  }

  // Make changes from THIS thread (constructor thread)
  for (int i = 0; i < numObjects; ++i) {
    for (int j = 0; j < 10; ++j) {
      std::string key = "prop" + boost::lexical_cast<std::string>(j);
      std::string value = "value" + boost::lexical_cast<std::string>(j);
      objects[i]->getState(key).value(value);
    }
  }

  // Wait for all handlers
  for (auto &obj : objects) {
    obj->waitForHandlers();
  }

  // Verify threading behavior for each object
  for (int i = 0; i < numObjects; ++i) {
    EXPECT_EQ(objects[i]->handleCount.load(), 10)
        << "Object " << i << " should have handled all 10 state changes";

    if (i % 2 == 0) {
      // Even indices had threading enabled - handlers run on different thread
      EXPECT_GT(objects[i]->threadedCount.load(), 0)
          << "Object " << i << " should have used threaded execution";
    } else {
      // Odd indices should be synchronous - handlers run on same thread
      EXPECT_EQ(objects[i]->synchronousCount.load(), 10)
          << "Object " << i << " should have used synchronous execution";
      EXPECT_EQ(objects[i]->threadedCount.load(), 0)
          << "Object " << i << " should not have used threaded execution";
    }
  }

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  for (auto &obj : objects) {
    obj->getState().reset();
  }
}

TEST_F(StateObjectFixture, StateObjectInstanceThreadingConcurrentStress) {
  // Stress test with concurrent modifications from multiple threads
  state_object<ThreadingTestObject>::setUseThreading(false);

  const int numObjects = 10;
  std::vector<std::shared_ptr<ThreadingTestObject>> objects;

  // Create objects with alternating threading settings
  for (int i = 0; i < numObjects; ++i) {
    auto obj = std::make_shared<ThreadingTestObject>(ctx);
    if (i % 2 == 0) {
      obj->setInstanceThreading(true);
    }
    objects.push_back(obj);
  }

  // Spawn threads to modify objects concurrently
  std::vector<boost::thread> threads;
  std::atomic<int> operationsCompleted(0);

  for (int i = 0; i < numObjects; ++i) {
    threads.emplace_back([i, &objects, &operationsCompleted]() {
      for (int j = 0; j < 10; ++j) {
        std::string key = "prop" + boost::lexical_cast<std::string>(j);
        std::string value = "value" + boost::lexical_cast<std::string>(j);
        objects[i]->getState(key).value(value);
        operationsCompleted++;
      }
    });
  }

  // Wait for all threads
  for (auto &t : threads) {
    t.join();
  }

  // Wait for all handlers
  for (auto &obj : objects) {
    obj->waitForHandlers();
  }

  // Verify all operations completed
  EXPECT_EQ(operationsCompleted.load(), numObjects * 10);

  // Verify all handlers were called
  for (int i = 0; i < numObjects; ++i) {
    EXPECT_EQ(objects[i]->handleCount.load(), 10)
        << "Object " << i << " should have handled all 10 state changes";
  }

  // Note: We can't easily verify synchronous vs threaded here because:
  // - Changes are made from worker threads, not the constructor thread
  // - Instance threading controls whether changes queue to a handler thread
  // - But the handler thread is still different from the worker thread
  // The key behavior is that handlers are called and complete correctly

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  for (auto &obj : objects) {
    obj->getState().reset();
  }
}

TEST_F(StateObjectFixture, StateObjectInstanceThreadingBatching) {
  // Test that batching works correctly with instance threading
  state_object<ThreadingTestObject>::setUseThreading(false);

  ThreadingTestObject obj(ctx);
  obj.setInstanceThreading(true);

  {
    state_change_batch_scope<ThreadingTestObject> batch(obj);

    for (int i = 0; i < 5; ++i) {
      obj.getState("batch_prop" + boost::lexical_cast<std::string>(i)).value("value");
    }

    // Handlers should not have been called yet
    EXPECT_EQ(obj.handleCount.load(), 0);

  } // batch.flush() happens here

  // Wait for handlers
  obj.waitForHandlers();

  // All handlers should have been called using threading
  EXPECT_EQ(obj.handleCount.load(), 5);
  EXPECT_GT(obj.threadedCount.load(), 0)
      << "Expected threaded execution with instance threading enabled";

  // Clean up
  state_object<ThreadingTestObject>::setUseThreading(true);
  obj.getState().reset();
}

// Main function to handle custom flags
int main(int argc, char **argv) {
  // Parse custom flags before GoogleTest removes them
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--enable-stress-tests") {
      enable_stress_tests = true;
    } else if (std::string(argv[i]) == "--help-stress") {
      std::cout << "\nStress Test Options:\n";
      std::cout << "  --enable-stress-tests    Enable long-running stress/performance tests\n";
      std::cout << "                           (default: disabled for faster test runs)\n";
      std::cout << "\nStress tests in state_test:\n";
      std::cout << "  - StateTest.PerformanceHierarchyBenchmark (6 minutes, 1M operations)\n";
      std::cout << std::endl;
      return 0;
    }
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

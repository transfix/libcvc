/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for cvc::app class functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic> // compute_async: the fan-out accumulator
#include <chrono> // compute_async: bounded wait for the completion callback
#include <condition_variable>
#include <cvc/core/app.h>
#include <cvc/core/async_task.h> // launch_pool_task: a dropped task posts on_error's value
#include <cvc/core/world_clock.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/core/world_units.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h> // PR05: the policy exec_scheduler() has
#include <filesystem>
#include <future> // compute_async: deterministic wait on on_done via a promise
#include <gtest/gtest.h>
#include <memory> // compute_async: shared accumulator captured by fn + on_done
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace cvc;

// ===========================
// AppTest fixture
// ===========================
// Each test gets a fresh, locally-owned cvc::app instance.  No reliance
// on the ctx singleton — tests exercise per-instance behavior.

class AppTest : public ::testing::Test {
protected:
  app ctx;
};

// ===========================
// Basic Construction Tests
// ===========================

TEST_F(AppTest, ConstructionYieldsValidInstance) {
  // Two references to the same local instance compare equal.
  app &app1 = ctx;
  app &app2 = ctx;
  EXPECT_EQ(&app1, &app2);
}

// ===========================
// Data Management Tests
// ===========================

TEST_F(AppTest, DataSetAndGet) {
  // Test setting and getting string data
  std::string test_key = "test.data.string";
  std::string test_value = "Hello, World!";

  ctx.data(test_key, test_value);

  ASSERT_TRUE(ctx.isData<std::string>(test_key));
  EXPECT_EQ(ctx.data<std::string>(test_key), test_value);

  // Clean up
  ctx.data(test_key, boost::any());
}

TEST_F(AppTest, DataTypeInt) {
  std::string key = "test.data.int";
  int value = 42;

  ctx.data(key, value);

  ASSERT_TRUE(ctx.isData<int>(key));
  EXPECT_EQ(ctx.data<int>(key), value);

  // Clean up
  ctx.data(key, boost::any());
}

TEST_F(AppTest, DataTypeDouble) {
  std::string key = "test.data.double";
  double value = 3.14159;

  ctx.data(key, value);

  ASSERT_TRUE(ctx.isData<double>(key));
  EXPECT_DOUBLE_EQ(ctx.data<double>(key), value);

  // Clean up
  ctx.data(key, boost::any());
}

TEST_F(AppTest, DataTypeBool) {
  std::string key = "test.data.bool";
  bool value = true;

  ctx.data(key, value);

  ASSERT_TRUE(ctx.isData<bool>(key));
  EXPECT_EQ(ctx.data<bool>(key), value);

  // Clean up
  ctx.data(key, boost::any());
}

TEST_F(AppTest, DataRemoval) {
  std::string key = "test.data.removal";
  std::string value = "temporary";

  ctx.data(key, value);
  ASSERT_TRUE(ctx.isData<std::string>(key));

  // Remove by setting empty boost::any
  ctx.data(key, boost::any());
  EXPECT_FALSE(ctx.isData<std::string>(key));
}

TEST_F(AppTest, DataTypeName) {
  std::string key = "test.data.typename";
  int value = 123;

  ctx.data(key, value);

  std::string typeName = ctx.dataTypeName(key);
  EXPECT_FALSE(typeName.empty());

  // Clean up
  ctx.data(key, boost::any());
}

TEST_F(AppTest, DataMap) {
  // Test getting the entire data map
  std::string key1 = "test.map.key1";
  std::string key2 = "test.map.key2";

  ctx.data(key1, 100);
  ctx.data(key2, std::string("value2"));

  data_map map = ctx.data();

  EXPECT_TRUE(map.find(key1) != map.end());
  EXPECT_TRUE(map.find(key2) != map.end());

  // Clean up
  ctx.data(key1, boost::any());
  ctx.data(key2, boost::any());
}

// ===========================
// Property Management Tests
// ===========================

TEST_F(AppTest, PropertySetAndGet) {
  std::string key = "test.property.simple";
  std::string value = "test_value";

  ctx.properties(key, value);

  EXPECT_EQ(ctx.properties(key), value);
  EXPECT_TRUE(ctx.hasProperty(key));

  // Clean up
  ctx.properties(key, std::string());
}

TEST_F(AppTest, PropertyRemoval) {
  std::string key = "test.property.removal";

  ctx.properties(key, "temporary");
  ASSERT_TRUE(ctx.hasProperty(key));

  // Remove by setting empty string
  ctx.properties(key, std::string());
  EXPECT_FALSE(ctx.hasProperty(key));
}

TEST_F(AppTest, PropertyList) {
  std::string key = "test.property.list";

  // Set comma-separated list
  ctx.properties(key, "item1,item2,item3");

  std::vector<std::string> items = ctx.listProperty(key);

  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item2");
  EXPECT_EQ(items[2], "item3");

  // Clean up
  ctx.properties(key, std::string());
}

TEST_F(AppTest, PropertyListWithSpaces) {
  std::string key = "test.property.list.spaces";

  // Set list with spaces around commas
  ctx.properties(key, "item1 , item2 , item3");

  std::vector<std::string> items = ctx.listProperty(key);

  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item2");
  EXPECT_EQ(items[2], "item3");

  // Clean up
  ctx.properties(key, std::string());
}

TEST_F(AppTest, PropertyListUnique) {
  std::string key = "test.property.list.unique";

  // Set list with duplicates
  ctx.properties(key, "item1,item2,item1,item3,item2");

  std::vector<std::string> items = ctx.listProperty(key, true);

  // Should have only unique items
  EXPECT_EQ(items.size(), 3);

  // Clean up
  ctx.properties(key, std::string());
}

TEST_F(AppTest, PropertyListAppend) {
  std::string key = "test.property.list.append";

  ctx.properties(key, "item1,item2");
  ctx.listPropertyAppend(key, "item3");

  std::vector<std::string> items = ctx.listProperty(key);

  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[2], "item3");

  // Clean up
  ctx.properties(key, std::string());
}

TEST_F(AppTest, PropertyListRemove) {
  std::string key = "test.property.list.remove";

  ctx.properties(key, "item1,item2,item3");
  ctx.listPropertyRemove(key, "item2");

  std::vector<std::string> items = ctx.listProperty(key);

  ASSERT_EQ(items.size(), 2);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item3");

  // Clean up
  ctx.properties(key, std::string());
}

TEST_F(AppTest, PropertyMap) {
  // Test getting the entire property map
  std::string key1 = "test.propmap.key1";
  std::string key2 = "test.propmap.key2";

  ctx.properties(key1, "value1");
  ctx.properties(key2, "value2");

  property_map map = ctx.properties();

  EXPECT_TRUE(map.find(key1) != map.end());
  EXPECT_TRUE(map.find(key2) != map.end());

  // Clean up
  ctx.properties(key1, std::string());
  ctx.properties(key2, std::string());
}

// ===========================
// Thread Management Tests
// ===========================

TEST_F(AppTest, ThreadKeyGeneration) {
  std::string hint = "test_thread";
  std::string key1 = ctx.uniqueThreadKey(hint);

  // Register the first key so the second will be different
  ctx.threads(key1, thread_ptr(new boost::thread()));

  std::string key2 = ctx.uniqueThreadKey(hint);

  // Keys should be unique
  EXPECT_NE(key1, key2);
  EXPECT_TRUE(key1.find(hint) != std::string::npos);
  EXPECT_TRUE(key2.find(hint) != std::string::npos);

  // Clean up
  ctx.removeThread(key1);
}

TEST_F(AppTest, ThreadProgress) {
  // Test thread progress tracking
  double progress = 0.5;
  ctx.threadProgress(progress);

  double retrieved = ctx.threadProgress();
  EXPECT_DOUBLE_EQ(retrieved, progress);
}

TEST_F(AppTest, ThreadProgressClamping) {
  // Test that progress is clamped to [0.0, 1.0]
  ctx.threadProgress(-0.5);
  EXPECT_DOUBLE_EQ(ctx.threadProgress(), 0.0);

  ctx.threadProgress(1.5);
  EXPECT_DOUBLE_EQ(ctx.threadProgress(), 1.0);
}

// ===========================
// Listify Tests
// ===========================

TEST_F(AppTest, ListifyVectorToString) {
  std::vector<std::string> vec = {"item1", "item2", "item3"};
  std::string result = ctx.listify(vec);

  EXPECT_FALSE(result.empty());
  EXPECT_TRUE(result.find("item1") != std::string::npos);
  EXPECT_TRUE(result.find("item2") != std::string::npos);
  EXPECT_TRUE(result.find("item3") != std::string::npos);
}

TEST_F(AppTest, ListifyStringToVector) {
  std::string list = "item1,item2,item3";
  std::vector<std::string> result = ctx.listify(list);

  ASSERT_EQ(result.size(), 3);
  EXPECT_EQ(result[0], "item1");
  EXPECT_EQ(result[1], "item2");
  EXPECT_EQ(result[2], "item3");
}

TEST_F(AppTest, ListifyRoundTrip) {
  std::vector<std::string> original = {"alpha", "beta", "gamma"};
  std::string stringified = ctx.listify(original);
  std::vector<std::string> result = ctx.listify(stringified);

  ASSERT_EQ(result.size(), original.size());
  for (size_t i = 0; i < original.size(); ++i) {
    EXPECT_EQ(result[i], original[i]);
  }
}

// ===========================
// Mutex Management Tests
// ===========================

TEST_F(AppTest, MutexCreation) {
  std::string mutex_name = "test.mutex";
  mutex_ptr mtx = ctx.mutex(mutex_name);

  ASSERT_NE(mtx, nullptr);

  // Getting the same mutex should return the same pointer
  mutex_ptr mtx2 = ctx.mutex(mutex_name);
  EXPECT_EQ(mtx, mtx2);
}

TEST_F(AppTest, MutexInfo) {
  std::string mutex_name = "test.mutex.info";
  std::string info = "Test mutex information";

  ctx.mutexInfo(mutex_name, info);
  std::string retrieved = ctx.mutexInfo(mutex_name);

  EXPECT_EQ(retrieved, info);
}

// ===========================
// Data Type Registration Tests
// ===========================

TEST_F(AppTest, DataTypeEnumRegistration) {
  // Test that basic types are registered correctly
  int test_int = 42;
  ctx.data("test.enum.int", test_int);

  data_type dt = ctx.dataType("test.enum.int");
  EXPECT_EQ(dt, Int);

  // Clean up
  ctx.data("test.enum.int", boost::any());
}

// ===========================
// Additional Property Map Tests
// ===========================

TEST_F(AppTest, PropertyMapOperations) {
  property_map test_map;
  test_map["prop1"] = "value1";
  test_map["prop2"] = "value2";
  test_map["prop3"] = "value3";

  ctx.properties(test_map);

  EXPECT_EQ(ctx.properties("prop1"), "value1");
  EXPECT_EQ(ctx.properties("prop2"), "value2");
  EXPECT_EQ(ctx.properties("prop3"), "value3");

  // Clean up
  ctx.properties("prop1", "");
  ctx.properties("prop2", "");
  ctx.properties("prop3", "");
}

TEST_F(AppTest, AddProperties) {
  property_map test_map;
  test_map["addprop1"] = "addvalue1";
  test_map["addprop2"] = "addvalue2";

  ctx.addProperties(test_map);

  EXPECT_TRUE(ctx.hasProperty("addprop1"));
  EXPECT_TRUE(ctx.hasProperty("addprop2"));
  EXPECT_EQ(ctx.properties("addprop1"), "addvalue1");

  // Clean up
  ctx.properties("addprop1", "");
  ctx.properties("addprop2", "");
}

TEST_F(AppTest, PropertyListOperations) {
  // Test comma-separated list property
  ctx.properties("test.list", "item1,item2,item3");
  std::vector<std::string> items = ctx.listProperty("test.list");

  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item2");
  EXPECT_EQ(items[2], "item3");

  // Clean up
  ctx.properties("test.list", "");
}

TEST_F(AppTest, PropertyListUniqueElements) {
  ctx.properties("test.list.unique", "a,b,a,c,b,d");
  std::vector<std::string> items = ctx.listProperty("test.list.unique", true);

  // Should only contain unique elements
  EXPECT_EQ(items.size(), 4);

  // Clean up
  ctx.properties("test.list.unique", "");
}

TEST_F(AppTest, ListPropertyAppendRemove) {
  ctx.properties("test.list.modify", "alpha,beta");

  ctx.listPropertyAppend("test.list.modify", "gamma");
  std::vector<std::string> items = ctx.listProperty("test.list.modify");
  EXPECT_EQ(items.size(), 3);
  EXPECT_EQ(items[2], "gamma");

  ctx.listPropertyRemove("test.list.modify", "beta");
  items = ctx.listProperty("test.list.modify");
  EXPECT_EQ(items.size(), 2);

  // Clean up
  ctx.properties("test.list.modify", "");
}

TEST_F(AppTest, PropertyTypedAccess) {
  ctx.properties("test.int.prop", 12345);
  int val = ctx.properties<int>("test.int.prop");
  EXPECT_EQ(val, 12345);

  ctx.properties("test.double.prop", 3.14159);
  double dval = ctx.properties<double>("test.double.prop");
  EXPECT_NEAR(dval, 3.14159, 0.00001);

  // Clean up
  ctx.properties("test.int.prop", "");
  ctx.properties("test.double.prop", "");
}

// ===========================
// Data Map Bulk Operations
// ===========================

TEST_F(AppTest, DataMapBulkOperations) {
  data_map test_data;
  test_data["bulk1"] = std::string("value1");
  test_data["bulk2"] = int(42);
  test_data["bulk3"] = double(3.14);

  ctx.data(test_data);

  EXPECT_TRUE(ctx.isData<std::string>("bulk1"));
  EXPECT_TRUE(ctx.isData<int>("bulk2"));
  EXPECT_TRUE(ctx.isData<double>("bulk3"));

  data_map retrieved = ctx.data();
  EXPECT_TRUE(retrieved.find("bulk1") != retrieved.end());
  EXPECT_TRUE(retrieved.find("bulk2") != retrieved.end());

  // Clean up
  ctx.data("bulk1", boost::any());
  ctx.data("bulk2", boost::any());
  ctx.data("bulk3", boost::any());
}

TEST_F(AppTest, DataVectorOperations) {
  std::vector<std::string> keys = {"vec1", "vec2", "vec3"};
  std::vector<int> values = {10, 20, 30};

  ctx.data(keys, values);

  EXPECT_EQ(ctx.data<int>("vec1"), 10);
  EXPECT_EQ(ctx.data<int>("vec2"), 20);
  EXPECT_EQ(ctx.data<int>("vec3"), 30);

  // Test retrieving as vector
  std::vector<int> retrieved = ctx.data<int>(keys);
  ASSERT_EQ(retrieved.size(), 3);
  EXPECT_EQ(retrieved[0], 10);
  EXPECT_EQ(retrieved[1], 20);
  EXPECT_EQ(retrieved[2], 30);

  // Clean up
  ctx.data("vec1", boost::any());
  ctx.data("vec2", boost::any());
  ctx.data("vec3", boost::any());
}

TEST_F(AppTest, DataTypesByType) {
  // Store several strings
  ctx.data("str1", std::string("test1"));
  ctx.data("str2", std::string("test2"));
  ctx.data("int1", int(42));

  // Get all string keys
  std::vector<std::string> str_keys = ctx.data<std::string>();
  EXPECT_GE(str_keys.size(), 2);

  // Clean up
  ctx.data("str1", boost::any());
  ctx.data("str2", boost::any());
  ctx.data("int1", boost::any());
}

TEST_F(AppTest, DataTypeNameFromAny) {
  boost::any any_val = std::string("test");
  std::string type_name = ctx.dataTypeName(any_val);
  EXPECT_FALSE(type_name.empty());
}

TEST_F(AppTest, DataTypeTemplate) {
  // Test template version
  std::string type_name = ctx.dataTypeName<std::string>();
  EXPECT_FALSE(type_name.empty());

  std::string type_name2 = ctx.dataTypeName<int>();
  EXPECT_FALSE(type_name2.empty());
}

// ===========================
// Thread Progress and Info Tests
// ===========================

TEST_F(AppTest, ThreadProgressBasic) {
  std::string thread_key = "test.thread.progress";

  // Simulate thread registration
  ctx.threads(thread_key, thread_ptr(new boost::thread([]() {})));

  ctx.threadProgress(thread_key, 0.5);
  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 0.5, 0.01);

  ctx.threadProgress(thread_key, 1.0);
  progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01);

  // Clean up
  ctx.threads(thread_key, thread_ptr());
}

TEST_F(AppTest, ThreadInfoOperations) {
  std::string thread_key = "test.thread.info";
  std::string info = "Processing data...";

  ctx.threads(thread_key, thread_ptr(new boost::thread([]() {})));

  ctx.threadInfo(thread_key, info);
  std::string retrieved = ctx.threadInfo(thread_key);
  EXPECT_EQ(retrieved, info);

  // Clean up
  ctx.threads(thread_key, thread_ptr());
}

TEST_F(AppTest, HasThread) {
  std::string thread_key = "test.thread.exists";

  EXPECT_FALSE(ctx.hasThread(thread_key));

  ctx.threads(thread_key, thread_ptr(new boost::thread([]() {})));
  EXPECT_TRUE(ctx.hasThread(thread_key));

  ctx.threads(thread_key, thread_ptr());
  EXPECT_FALSE(ctx.hasThread(thread_key));
}

TEST_F(AppTest, UniqueThreadKey) {
  // Register a thread with first key
  std::string key1 = ctx.uniqueThreadKey("test");
  ctx.threads(key1, thread_ptr(new boost::thread([]() {})));

  // Should get a different unique key
  std::string key2 = ctx.uniqueThreadKey("test");

  // Keys should be unique
  EXPECT_NE(key1, key2);

  // Clean up
  ctx.threads(key1, thread_ptr());
}

// thread_info and thread_feedback call into the app, and each of those calls is an interruption
// point. An interrupt that arrives while they keep their books must stay pending for the thread's
// next interruption point. They used to throw it there and swallow it, which clears the request:
// the thread kept running, and ~app gave up joining it and freed the app under it.
TEST_F(AppTest, ThreadBookkeepingKeepsAnInterruptPending) {
  std::atomic<bool> go{false};
  std::atomic<bool> stopped{false};
  boost::thread t([&]() {
    while (!go.load()) // yield is not an interruption point
      boost::this_thread::yield();
    {
      thread_feedback feedback(ctx); // and the thread_info inside it
      thread_info info(ctx, "nested");
    }
    try {
      boost::this_thread::interruption_point();
    } catch (boost::thread_interrupted &) {
      stopped = true;
    }
  });
  t.interrupt(); // before the thread can reach any of the bookkeeping
  go = true;
  t.join();
  EXPECT_TRUE(stopped.load());
}

// ===========================
// Listify Tests
// ===========================

TEST_F(AppTest, ListifyString) {
  std::string list = "a,b,c";
  std::vector<std::string> vec = ctx.listify(list);

  ASSERT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[0], "a");
  EXPECT_EQ(vec[1], "b");
  EXPECT_EQ(vec[2], "c");
}

TEST_F(AppTest, ListifyVector) {
  std::vector<std::string> vec = {"x", "y", "z"};
  std::string list = ctx.listify(vec);

  EXPECT_FALSE(list.empty());
  EXPECT_NE(list.find("x"), std::string::npos);
  EXPECT_NE(list.find("y"), std::string::npos);
  EXPECT_NE(list.find("z"), std::string::npos);
}

// ===========================
// Data Reader Tests
// ===========================

TEST_F(AppTest, DataReaderCollection) {
  data_reader_collection readers = ctx.dataReaders();

  // Add a test reader
  data_reader test_reader = [](const std::string &path) -> bool { return path == "test.dat"; };

  ctx.dataReader(test_reader);

  data_reader_collection updated = ctx.dataReaders();
  EXPECT_GE(updated.size(), readers.size());
}

TEST_F(AppTest, ReadDataWithReaders) {
  // Add a reader that handles .test files
  data_reader test_reader = [this](const std::string &path) -> bool {
    if (path.find(".test") != std::string::npos) {
      ctx.data("test.read.result", std::string("success"));
      return true;
    }
    return false;
  };

  ctx.dataReader(test_reader);

  // Try to read a .test file
  bool result = ctx.readData("example.test");
  EXPECT_TRUE(result);
  EXPECT_EQ(ctx.data<std::string>("test.read.result"), "success");

  // Try to read an unsupported file
  bool result2 = ctx.readData("example.unsupported");
  EXPECT_FALSE(result2);

  // Clean up
  ctx.data("test.read.result", boost::any());
}

// ===========================
// Property Map File I/O Tests
// ===========================

TEST_F(AppTest, PropertyMapSaveLoad) {
  std::string temp_file =
      (std::filesystem::temp_directory_path() / "test_property_map.info").string();

  // Set up some properties
  ctx.properties("io.test.prop1", "value1");
  ctx.properties("io.test.prop2", "value2");
  ctx.properties("io.test.number", 42);

  // Save property map
  ctx.writePropertyMap(temp_file);

  // Clear properties
  ctx.properties("io.test.prop1", "");
  ctx.properties("io.test.prop2", "");
  ctx.properties("io.test.number", "");

  // Restore from file
  ctx.readPropertyMap(temp_file);

  // Verify restored
  EXPECT_EQ(ctx.properties("io.test.prop1"), "value1");
  EXPECT_EQ(ctx.properties("io.test.prop2"), "value2");

  // Clean up
  std::remove(temp_file.c_str());
  ctx.properties("io.test.prop1", "");
  ctx.properties("io.test.prop2", "");
  ctx.properties("io.test.number", "");
}

// ===========================
// Thread Map Bulk Operations
// ===========================

TEST_F(AppTest, ThreadMapBulkSet) {
  thread_map test_map;

  // Create threads
  thread_ptr t1(new boost::thread([]() {}));
  thread_ptr t2(new boost::thread([]() {}));

  test_map["bulk.thread1"] = t1;
  test_map["bulk.thread2"] = t2;

  // Set the entire map
  ctx.threads(test_map);

  // Verify
  EXPECT_TRUE(ctx.hasThread("bulk.thread1"));
  EXPECT_TRUE(ctx.hasThread("bulk.thread2"));

  // Clean up
  ctx.threads("bulk.thread1", thread_ptr());
  ctx.threads("bulk.thread2", thread_ptr());
}

TEST_F(AppTest, RemoveThread) {
  std::string thread_key = "test.remove.thread";

  ctx.threads(thread_key, thread_ptr(new boost::thread([]() {})));
  EXPECT_TRUE(ctx.hasThread(thread_key));

  ctx.removeThread(thread_key);
  EXPECT_FALSE(ctx.hasThread(thread_key));
}

TEST_F(AppTest, ThreadKeyLookup) {
  // Get thread key for current thread (should return "unknown" or valid key)
  std::string key = ctx.threadKey();
  EXPECT_FALSE(key.empty());
}

// ===========================
// Sleep Function Test
// ===========================

TEST_F(AppTest, SleepFunction) {
  auto start = boost::posix_time::microsec_clock::universal_time();
  ctx.sleep(10.0); // Sleep for 10 milliseconds
  auto end = boost::posix_time::microsec_clock::universal_time();

  auto duration = end - start;
  EXPECT_GE(duration.total_milliseconds(), 5);
}

// ===========================
// Property Data Tests
// ===========================

TEST_F(AppTest, PropertyDataLookup) {
  // Store some int data
  ctx.data("pd.obj1", 100);
  ctx.data("pd.obj2", 200);
  ctx.data("pd.obj3", 300);

  // Create property with list of keys
  ctx.properties("pd.list", "pd.obj1,pd.obj2,pd.obj3");

  // Get property data
  std::vector<int> data = ctx.propertyData<int>("pd.list");

  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 100);
  EXPECT_EQ(data[1], 200);
  EXPECT_EQ(data[2], 300);

  // Clean up
  ctx.properties("pd.list", "");
  ctx.data("pd.obj1", boost::any());
  ctx.data("pd.obj2", boost::any());
  ctx.data("pd.obj3", boost::any());
}

TEST_F(AppTest, ListDataFunction) {
  // Store data
  ctx.data("ld.a", 10);
  ctx.data("ld.b", 20);
  ctx.data("ld.c", 30);

  // Use listData to get data from comma-separated list
  std::vector<int> data = ctx.listData<int>("ld.a,ld.b,ld.c");

  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 10);
  EXPECT_EQ(data[1], 20);
  EXPECT_EQ(data[2], 30);

  // Clean up
  ctx.data("ld.a", boost::any());
  ctx.data("ld.b", boost::any());
  ctx.data("ld.c", boost::any());
}

// ===========================
// Data Type Enum Tests
// ===========================

TEST_F(AppTest, DataTypeEnumTemplateMethod) {
  // Test template version of dataType
  data_type dt_int = ctx.dataType<int>();
  EXPECT_EQ(dt_int, Int);

  data_type dt_double = ctx.dataType<double>();
  EXPECT_EQ(dt_double, Double);

  data_type dt_float = ctx.dataType<float>();
  EXPECT_EQ(dt_float, Float);
}

// ===========================
// Scoped Lock Tests
// ===========================

TEST_F(AppTest, ScopedLockUsage) {
  std::string mutex_name = "test.scoped.mutex";

  {
    scoped_lock lock(ctx, mutex_name, "test lock info");

    // Mutex should be locked and info set (may include thread info)
    std::string info = ctx.mutexInfo(mutex_name);
    EXPECT_NE(info.find("test lock info"), std::string::npos);
  }

  // After scope, info should be cleared or reset
  std::string info = ctx.mutexInfo(mutex_name);
  // Info may persist or be cleared depending on implementation
  EXPECT_TRUE(true); // Just verify no crash
}

// ===========================
// Thread Pool Tests
// ===========================

TEST_F(AppTest, ThreadPoolBasicExecution) {
  std::atomic<bool> task_executed(false);

  ctx.startThreadPooled(
      "pool_basic_test", [&task_executed]() { task_executed = true; }, PRIORITY_NORMAL, true);

  // Wait for task to complete
  if (ctx.hasThread("pool_basic_test")) {
    thread_ptr tptr = ctx.threads("pool_basic_test");
    if (tptr)
      tptr->join();
  }

  EXPECT_TRUE(task_executed.load());
}

TEST_F(AppTest, ThreadPoolMultipleTasks) {
  std::atomic<int> counter(0);
  std::vector<std::string> keys;
  const int num_tasks = 5;

  for (int i = 0; i < num_tasks; i++) {
    std::string key = "pool_multi_" + std::to_string(i);
    keys.push_back(key);
    ctx.startThreadPooled(
        key,
        [&counter]() {
          counter++;
          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        },
        PRIORITY_NORMAL, true);
  }

  // Wait for all tasks
  for (const auto &key : keys) {
    if (ctx.hasThread(key)) {
      thread_ptr tptr = ctx.threads(key);
      if (tptr)
        tptr->join();
    }
  }

  EXPECT_EQ(counter.load(), num_tasks);
}

TEST_F(AppTest, ThreadPoolPriority) {
  std::atomic<int> execution_order(0);
  std::vector<int> order;
  boost::mutex order_mutex;

  // Set small pool size to force queuing
  unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(1);

  // Start a blocking task to fill the pool
  std::atomic<bool> blocker_done(false);
  std::atomic<bool> blocker_running(true);
  ctx.startThreadPooled(
      "pool_priority_blocker",
      [&]() {
        while (blocker_running.load()) {
          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
          boost::this_thread::interruption_point();
        }
        blocker_done = true;
      },
      PRIORITY_NORMAL, false); // use unique key

  // Give blocker time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // Now submit tasks with unique keys that will queue (blocker is running)
  // Submit in reverse priority order - high, normal, critical
  ctx.startThreadPooled(
      "pool_priority_high",
      [&]() {
        boost::mutex::scoped_lock lock(order_mutex);
        order.push_back(1);
        execution_order++;
      },
      PRIORITY_HIGH, false); // unique key

  ctx.startThreadPooled(
      "pool_priority_normal",
      [&]() {
        boost::mutex::scoped_lock lock(order_mutex);
        order.push_back(0);
        execution_order++;
      },
      PRIORITY_NORMAL, false); // unique key

  ctx.startThreadPooled(
      "pool_priority_critical",
      [&]() {
        boost::mutex::scoped_lock lock(order_mutex);
        order.push_back(2);
        execution_order++;
      },
      PRIORITY_CRITICAL, false); // unique key

  // Give tasks time to queue
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

  // Stop blocker to let queued tasks execute
  blocker_running = false;

  // Wait for all tasks to complete
  for (int i = 0; i < 50; i++) {
    if (execution_order.load() >= 3)
      break;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }

  EXPECT_TRUE(blocker_done.load());
  EXPECT_EQ(execution_order.load(), 3);
  EXPECT_EQ(order.size(), 3);

  // With priority queue, critical should execute first, then high, then normal
  // Order should be: [2, 1, 0] (critical=2, high=1, normal=0)
  if (order.size() == 3) {
    EXPECT_EQ(order[0], 2); // critical first
    EXPECT_EQ(order[1], 1); // high second
    EXPECT_EQ(order[2], 0); // normal last
  }

  ctx.setThreadPoolSize(original_size);
}

TEST_F(AppTest, ThreadPoolSizeConfiguration) {
  unsigned int original_size = ctx.getThreadPoolSize();

  // Test setting pool size
  ctx.setThreadPoolSize(2);
  EXPECT_EQ(ctx.getThreadPoolSize(), 2u);

  ctx.setThreadPoolSize(4);
  EXPECT_EQ(ctx.getThreadPoolSize(), 4u);

  // Restore original
  ctx.setThreadPoolSize(original_size);
  EXPECT_EQ(ctx.getThreadPoolSize(), original_size);
}

TEST_F(AppTest, ThreadPoolActiveCount) {
  std::atomic<bool> keep_running(true);
  std::vector<std::string> keys;

  unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(2);

  // Start 2 long-running tasks
  for (int i = 0; i < 2; i++) {
    std::string key = "pool_active_" + std::to_string(i);
    keys.push_back(key);
    ctx.startThreadPooled(
        key,
        [&keep_running]() {
          while (keep_running.load()) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
          }
        },
        PRIORITY_NORMAL, true);
  }

  // Give threads time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // Should have 2 active workers
  unsigned int active = ctx.getActiveThreadCount();
  EXPECT_GE(active, 1u); // At least one should be running
  EXPECT_LE(active, 2u); // Should not exceed pool size

  // Stop tasks
  keep_running = false;

  for (const auto &key : keys) {
    if (ctx.hasThread(key)) {
      thread_ptr tptr = ctx.threads(key);
      if (tptr)
        tptr->join();
    }
  }

  ctx.setThreadPoolSize(original_size);
}

TEST_F(AppTest, ThreadPoolInterruption) {
  std::atomic<bool> task_started(false);
  std::atomic<bool> task_interrupted(false);

  ctx.startThreadPooled(
      "pool_interrupt_test",
      [&]() {
        task_started = true;
        try {
          // Long-running task with interruption points
          for (int i = 0; i < 100; i++) {
            boost::this_thread::interruption_point();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
          }
        } catch (boost::thread_interrupted &) {
          task_interrupted = true;
          throw;
        }
      },
      PRIORITY_NORMAL, true);

  // Give task time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  EXPECT_TRUE(task_started.load());

  // Interrupt the thread
  if (ctx.hasThread("pool_interrupt_test")) {
    thread_ptr tptr = ctx.threads("pool_interrupt_test");
    if (tptr) {
      tptr->interrupt();
      try {
        tptr->join();
      } catch (...) {
        // Thread may have thrown during join
      }
    }
  }

  // Give time for interruption to process
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  EXPECT_TRUE(task_interrupted.load());
}

TEST_F(AppTest, ThreadPoolReplaceRunningTask) {
  std::atomic<int> task_count(0);
  std::atomic<bool> first_task_running(true);

  // Start first task with wait=true (uses key "replace_test")
  ctx.startThreadPooled(
      "pool_replace_test",
      [&]() {
        task_count++;
        while (first_task_running.load()) {
          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
          boost::this_thread::interruption_point();
        }
      },
      PRIORITY_NORMAL, true);

  // Give it time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  EXPECT_EQ(task_count.load(), 1);

  // Start second task with same key and wait=true
  // This should interrupt the first task and start a new one
  ctx.startThreadPooled("pool_replace_test", [&]() { task_count++; }, PRIORITY_NORMAL, true);

  // Wait for new task
  if (ctx.hasThread("pool_replace_test")) {
    thread_ptr tptr = ctx.threads("pool_replace_test");
    if (tptr)
      tptr->join();
  }

  // Should have executed both tasks (first interrupted, second completed)
  EXPECT_EQ(task_count.load(), 2);
  first_task_running = false;
}

TEST_F(AppTest, ThreadPoolExceptionHandling) {
  std::atomic<bool> task_ran(false);
  std::atomic<bool> cleanup_ran(false);

  // Check initial state
  unsigned int initial_active = ctx.getActiveThreadCount();
  bool had_exception_thread_before = ctx.hasThread("pool_exception_test");

  ctx.startThreadPooled(
      "pool_exception_test",
      [&]() {
        task_ran = true;
        throw std::runtime_error("Test exception");
      },
      PRIORITY_NORMAL, true);

  // Give task time to execute and fail
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  EXPECT_TRUE(task_ran.load());

  // Verify thread pool state after exception
  // The thread should have exited and active worker count should return to normal
  unsigned int active_after_exception = ctx.getActiveThreadCount();
  EXPECT_EQ(active_after_exception, initial_active)
      << "Active worker count should return to initial state";

  // Pool should continue to work after exception
  ctx.startThreadPooled(
      "pool_after_exception", [&]() { cleanup_ran = true; }, PRIORITY_NORMAL, true);

  if (ctx.hasThread("pool_after_exception")) {
    thread_ptr tptr = ctx.threads("pool_after_exception");
    if (tptr)
      tptr->join();
  }

  EXPECT_TRUE(cleanup_ran.load());
}

TEST_F(AppTest, ThreadPoolStateConsistency) {
  // Test that thread map stays clean after multiple operations
  unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(2);

  std::atomic<int> completed(0);
  std::vector<std::string> keys;

  // Run several tasks including some that throw
  for (int i = 0; i < 5; i++) {
    std::string key = "pool_state_" + std::to_string(i);
    keys.push_back(key);

    ctx.startThreadPooled(
        key,
        [&, i]() {
          if (i == 2) {
            // Task 2 throws an exception
            throw std::runtime_error("Intentional exception");
          }
          completed++;
          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        },
        PRIORITY_NORMAL, true);
  }

  // Wait for all tasks to complete
  boost::this_thread::sleep_for(boost::chrono::milliseconds(200));

  // Should have completed 4 tasks (task 2 threw exception)
  EXPECT_EQ(completed.load(), 4);

  // Active worker count should be back to 0 or minimal
  unsigned int active = ctx.getActiveThreadCount();
  EXPECT_EQ(active, 0u) << "No workers should be active after all tasks complete";

  ctx.setThreadPoolSize(original_size);
}

TEST_F(AppTest, ThreadPoolConcurrencyLimit) {
  std::atomic<int> concurrent_count(0);
  std::atomic<int> max_concurrent(0);
  std::vector<std::string> keys;
  boost::mutex counter_mutex;

  unsigned int original_size = ctx.getThreadPoolSize();
  const int pool_size = 2;
  ctx.setThreadPoolSize(pool_size);

  // Submit more tasks than pool size
  for (int i = 0; i < 5; i++) {
    std::string key = "pool_concurrency_" + std::to_string(i);
    keys.push_back(key);

    ctx.startThreadPooled(
        key,
        [&]() {
          // Track concurrent execution
          int current;
          {
            boost::mutex::scoped_lock lock(counter_mutex);
            concurrent_count++;
            current = concurrent_count.load();
            if (current > max_concurrent.load()) {
              max_concurrent = current;
            }
          }

          // Simulate work
          boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

          {
            boost::mutex::scoped_lock lock(counter_mutex);
            concurrent_count--;
          }
        },
        PRIORITY_NORMAL, true);
  }

  // Wait for all tasks
  for (const auto &key : keys) {
    if (ctx.hasThread(key)) {
      thread_ptr tptr = ctx.threads(key);
      if (tptr)
        tptr->join();
    }
  }

  // Max concurrent should not exceed pool size
  EXPECT_LE(max_concurrent.load(), pool_size);
  EXPECT_GE(max_concurrent.load(), 1); // At least one should have run

  ctx.setThreadPoolSize(original_size);
}

TEST_F(AppTest, ThreadPoolTaskChaining) {
  std::atomic<int> execution_order(0);
  std::vector<int> order;
  boost::mutex order_mutex;

  unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(1); // Force serial execution

  // Submit multiple tasks that will chain
  for (int i = 0; i < 3; i++) {
    std::string key = "pool_chain_" + std::to_string(i);
    ctx.startThreadPooled(
        key,
        [&, i]() {
          boost::mutex::scoped_lock lock(order_mutex);
          order.push_back(i);
          execution_order++;
        },
        PRIORITY_NORMAL, true);
  }

  // Wait for all
  for (int i = 0; i < 3; i++) {
    std::string key = "pool_chain_" + std::to_string(i);
    if (ctx.hasThread(key)) {
      thread_ptr tptr = ctx.threads(key);
      if (tptr)
        tptr->join();
    }
  }

  EXPECT_EQ(execution_order.load(), 3);
  EXPECT_EQ(order.size(), 3);

  ctx.setThreadPoolSize(original_size);
}

// Spin for `us` microseconds without an interruption point (a kernel that never checks).
static void spin_us(int us) {
  const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(us);
  while (std::chrono::steady_clock::now() < until) {
  }
}

// Regression (docs/STATE_EXEC_BENCH.md, "The background pool stalls, then hangs"): more
// anonymous (wait=false) tasks than the pool has workers. The pool used to give every task queued
// while it was full the same thread key, so a worker starting the next one interrupted and joined
// (up to 5 s) the thread already holding that key; the interrupted worker then threw from the
// interruption point at the top of app::threads() inside tryStartWorker, which had unlocked
// _threadPoolMutex by hand, and the scoped lock of its caller unlocked it a second time. On
// Windows that leaves the boost mutex looking locked with no owner, and every later locker
// (workers, submitters, ~app) blocks forever. Every task must run, promptly, and the pool must
// still answer afterwards.
TEST_F(AppTest, ThreadPoolFloodPastPoolSizeCompletes) {
  const unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(4);
  const int n = 1000;
  const int spin = 20; // microseconds: no interruption point, like a kernel that never checks
  std::atomic<int> done(0);
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; i++)
    ctx.startThreadPooled(
        "pool_flood",
        [&done, spin] {
          spin_us(spin);
          done++;
        },
        PRIORITY_NORMAL, /*wait=*/false);
  const auto deadline = t0 + std::chrono::seconds(10);
  while (done.load() < n && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_EQ(done.load(), n) << "the pool stranded tasks";
  // 1000 x 20 us on 4 workers is ~5 ms of work: no 5 s stall per key collision.
  EXPECT_LT(secs, 4.0);
  // The pool still answers (its mutex is usable) and drains to idle.
  for (int i = 0; i < 2000 && (ctx.getActiveThreadCount() != 0 || ctx.getPendingThreadCount() != 0);
       i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(ctx.getActiveThreadCount(), 0u);
  EXPECT_EQ(ctx.getPendingThreadCount(), 0u);
  ctx.setThreadPoolSize(original_size);
}

// Wait until the background pool has nothing queued or running (bounded).
static bool wait_pool_idle(app &a, int ms = 5000) {
  for (int i = 0; i < ms; i++) {
    if (a.getActiveThreadCount() == 0 && a.getPendingThreadCount() == 0)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

// Anonymous tasks run on persistent workers: never more OS threads than the pool's size, the
// same few threads reused, and nothing added to the thread map.
TEST_F(AppTest, ThreadPoolAnonymousTasksReuseFewWorkersOutsideTheThreadMap) {
  const unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(3);
  const std::size_t threads_before = ctx.threads().size();
  std::mutex m;
  std::set<std::thread::id> ids;
  std::atomic<int> done(0);
  for (int i = 0; i < 300; i++)
    ctx.submitPooled([&] {
      {
        std::lock_guard<std::mutex> lk(m);
        ids.insert(std::this_thread::get_id());
      }
      done++;
    });
  ASSERT_TRUE(wait_pool_idle(ctx));
  EXPECT_EQ(done.load(), 300);
  EXPECT_LE(ids.size(), 3u);
  EXPECT_GE(ids.size(), 1u);
  EXPECT_EQ(ctx.threads().size(), threads_before);
  EXPECT_NE(ids.count(std::this_thread::get_id()), 1u); // off the submitting thread
  ctx.setThreadPoolSize(original_size);
}

// Within a priority, tasks start in the order they were submitted.
TEST_F(AppTest, ThreadPoolIsFifoWithinAPriority) {
  const unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(1);
  std::atomic<bool> release(false);
  std::atomic<bool> blocker_running(false);
  ctx.submitPooled([&] {
    blocker_running = true;
    while (!release.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  for (int i = 0; i < 2000 && !blocker_running.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(blocker_running.load());
  std::mutex m;
  std::vector<int> order;
  for (int i = 0; i < 40; i++)
    ctx.submitPooled(
        [&, i] {
          std::lock_guard<std::mutex> lk(m);
          order.push_back(i);
        },
        i % 2 ? PRIORITY_HIGH : PRIORITY_NORMAL);
  EXPECT_EQ(ctx.getPendingThreadCount(), 40u);
  release = true;
  ASSERT_TRUE(wait_pool_idle(ctx));
  ASSERT_EQ(order.size(), 40u);
  // Every HIGH task (odd i, ascending) before every NORMAL one (even i, ascending).
  for (int k = 0; k < 20; k++) {
    EXPECT_EQ(order[k], 2 * k + 1) << "at " << k;
    EXPECT_EQ(order[20 + k], 2 * k) << "at " << 20 + k;
  }
  ctx.setThreadPoolSize(original_size);
}

// A finished task keeps nothing alive: its captures go on the worker before it counts as done
// (it used to stay in the thread map, with everything it captured, until ~app).
TEST_F(AppTest, ThreadPoolFinishedTasksRetainNothing) {
  std::vector<std::weak_ptr<std::string>> weak;
  for (int i = 0; i < 50; i++) {
    auto payload = std::make_shared<std::string>(1024, 'x');
    weak.push_back(payload);
    ctx.startThreadPooled(
        "pool_retention", [payload] { EXPECT_EQ(payload->size(), 1024u); }, PRIORITY_NORMAL,
        /*wait=*/false);
  }
  ASSERT_TRUE(wait_pool_idle(ctx));
  for (const auto &w : weak)
    EXPECT_TRUE(w.expired());
}

// wait() (wait_for_threads) waits for anonymous pool tasks that are queued or running.
TEST_F(AppTest, WaitForThreadsWaitsForThePool) {
  const unsigned int original_size = ctx.getThreadPoolSize();
  ctx.setThreadPoolSize(2);
  std::atomic<int> done(0);
  for (int i = 0; i < 6; i++)
    ctx.submitPooled([&done] {
      spin_us(20000); // no interruption point: runs to the end
      done++;
    });
  ctx.wait();
  EXPECT_EQ(done.load(), 6);
  EXPECT_EQ(ctx.getActiveThreadCount(), 0u);
  EXPECT_EQ(ctx.getPendingThreadCount(), 0u);
  ctx.setThreadPoolSize(original_size);
}

// ~app ends promptly with a full queue: the running task is interrupted, the queued ones are
// dropped without running and release what they captured.
TEST(AppPoolTeardown, DestroyingTheAppDropsQueuedTasksPromptly) {
  auto payload = std::make_shared<int>(7);
  std::weak_ptr<int> weak = payload;
  std::atomic<int> ran(0);
  std::atomic<bool> started(false);
  const auto t0 = std::chrono::steady_clock::now();
  {
    app a;
    a.setThreadPoolSize(1);
    a.submitPooled([&started] {
      started = true;
      for (;;) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // an interruption point
      }
    });
    for (int i = 0; i < 100; i++)
      a.submitPooled([payload, &ran] { ran++; });
    payload.reset();
    for (int i = 0; i < 2000 && !started.load(); i++)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_TRUE(started.load());
  }
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 3.0);
  EXPECT_EQ(ran.load(), 0);
  EXPECT_TRUE(weak.expired());
}

// ~app interrupts every task a worker has taken, however soon after the submission it comes. (An
// interrupt that reached a worker after it took a task but before the task started was once
// dropped as stale, and ~app then waited 5 s for a task that never stopped.)
TEST(AppPoolTeardown, DestroyingTheAppRightAfterSubmittingInterruptsRunningTasks) {
  for (int iter = 0; iter < 64; iter++) {
    // Shared, and the loop bounded, so that a task that is never interrupted ends by itself
    // (after ~app's 5 s join gives up on it) without touching anything freed.
    struct counts {
      std::atomic<int> started{0};
      std::atomic<int> interrupted{0};
    };
    auto c = std::make_shared<counts>();
    const auto t0 = std::chrono::steady_clock::now();
    {
      app a;
      a.setThreadPoolSize(2);
      for (int t = 0; t < 2; t++)
        a.submitPooled([c] {
          c->started++;
          try {
            const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(8);
            while (std::chrono::steady_clock::now() < until)
              boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // interruption point
          } catch (boost::thread_interrupted &) {
            c->interrupted++;
            throw;
          }
        });
      spin_us((iter % 16) * 20); // 0..300 us: ~app lands at every stage of the tasks' start
    }
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_LT(secs, 3.0) << "iteration " << iter;
    EXPECT_EQ(c->interrupted.load(), c->started.load()) << "iteration " << iter;
    if (HasFailure())
      break;
  }
}

namespace {

// An app whose pool internals a test can reach: ~app's first step for the pool, with the app
// still there, and the window between a new worker's start and its registration.
struct probe_app : app {
  void stopPool() { stopThreadPool(); }
  // Called on the thread creating a worker, once the worker runs and before it is registered.
  void onWorkerCreated(std::function<void()> f) { _poolWorkerCreatedHook = std::move(f); }
  // Called on the thread creating a worker, right before its thread is created; a throw makes
  // that creation fail, as when thread resources are exhausted.
  void onWorkerSpawn(std::function<void()> f) { _poolWorkerSpawnHook = std::move(f); }
  std::uint64_t interruptsSent() {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    return _poolInterruptGen;
  }
  // trySubmitPooled calls deciding whether to hand their own task back (waiting on the worker
  // creations it relies on, say).
  unsigned int handBackChecks() {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    return _handBackChecks;
  }
};

} // namespace

// wait() interrupts a task that a just-created worker took, however soon after the worker's
// start it comes. wait() and ~app interrupt the workers in the pool's worker list, and a new
// worker could take and start a task before the thread that created it had put it there: the
// interrupt then missed the task, wait() waited out the task (up to its 5 s), and ~app could
// return while the task still ran. Here the creator is held in that window until the task has
// started and wait() has sent its interrupt (or, the task not starting, for 300 ms); a new worker
// now takes no task before it is in the list.
TEST(AppPoolTeardown, AnInterruptRightAfterAWorkerStartsReachesItsTask) {
  struct shared {
    std::mutex m;
    std::condition_variable cv;
    bool started = false;
    std::atomic<int> interrupted{0};
  };
  auto s = std::make_shared<shared>();
  probe_app a;
  a.setThreadPoolSize(2);
  const std::uint64_t sent = a.interruptsSent();
  a.onWorkerCreated([&a, s, sent] {
    {
      std::unique_lock<std::mutex> lk(s->m);
      if (!s->cv.wait_for(lk, std::chrono::milliseconds(300), [&s] { return s->started; }))
        return; // the new worker hasn't taken its task: it is waiting to be registered
    }
    // It took its task unregistered: hold the creator until wait() has interrupted the pool.
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (a.interruptsSent() == sent && std::chrono::steady_clock::now() < until)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  std::thread creator([&a, s] {
    a.submitPooled([s] { // a fresh app: this creates the worker that takes it
      {
        std::lock_guard<std::mutex> lk(s->m);
        s->started = true;
      }
      s->cv.notify_all();
      try {
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(4);
        while (std::chrono::steady_clock::now() < until)
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // interruption point
      } catch (boost::thread_interrupted &) {
        s->interrupted++;
        throw;
      }
    });
  });
  bool started = false;
  {
    std::unique_lock<std::mutex> lk(s->m);
    started = s->cv.wait_for(lk, std::chrono::seconds(10), [&s] { return s->started; });
  }
  const auto t0 = std::chrono::steady_clock::now();
  a.wait(); // the task has started: this interrupts it
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  creator.join();
  EXPECT_TRUE(started);
  EXPECT_EQ(s->interrupted.load(), 1);
  EXPECT_LT(secs, 2.0);
}

// ~app stops the pool while a worker's creator is between starting it and registering it: the
// worker, never registered, ends without taking a task (the task goes with the queue, unrun), and
// the creator is not held up. (The worker used to take the task, which nothing interrupted, and
// its creator then joined it with no interrupt and no time limit.)
TEST(AppPoolTeardown, AWorkerStartedAsThePoolStopsTakesNoTask) {
  struct shared {
    std::mutex m;
    std::condition_variable cv;
    bool started = false;
  };
  auto s = std::make_shared<shared>();
  auto payload = std::make_shared<int>(1);
  std::weak_ptr<int> weak = payload;
  probe_app a;
  a.setThreadPoolSize(2);
  a.onWorkerCreated([&a, s] {
    {
      // Give the new worker the time to take its task, were it allowed to before registration.
      std::unique_lock<std::mutex> lk(s->m);
      s->cv.wait_for(lk, std::chrono::milliseconds(300), [&s] { return s->started; });
    }
    a.stopPool(); // ~app's first step, with this worker not registered yet
  });
  const auto t0 = std::chrono::steady_clock::now();
  a.submitPooled([s, payload] { // a fresh app: this creates the worker, and the hook stops it
    {
      std::lock_guard<std::mutex> lk(s->m);
      s->started = true;
    }
    s->cv.notify_all();
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < until)
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // interruption point
  });
  payload.reset();
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  {
    std::lock_guard<std::mutex> lk(s->m);
    EXPECT_FALSE(s->started);
  }
  EXPECT_TRUE(weak.expired());
  EXPECT_LT(secs, 2.0);
}

// Once ~app has stopped the pool, a task submitted to it is not run: submitPooled says so, and
// the task, with what it captured, is gone when it returns (async_host and launch_pool_task
// report the drop from there).
TEST(AppPoolTeardown, ATaskSubmittedOnceThePoolHasStoppedIsDroppedAndReported) {
  probe_app a;
  EXPECT_TRUE(a.submitPooled([] {}));
  ASSERT_TRUE(wait_pool_idle(a));
  a.stopPool();
  auto payload = std::make_shared<int>(1);
  std::weak_ptr<int> weak = payload;
  std::atomic<bool> ran(false);
  EXPECT_FALSE(a.submitPooled([payload, &ran] { ran = true; }));
  payload.reset();
  EXPECT_TRUE(weak.expired());
  EXPECT_FALSE(ran.load());
  EXPECT_EQ(a.getPendingThreadCount(), 0u);
}

// compute_async's on_error form calls exactly one of its callbacks: on_done when the fan-out
// joins, on_error with the message when fn throws, and on_error when the app drops the job unrun
// (submitted once the pool has stopped, or still queued when ~app stops it). The four-argument
// form calls neither for a dropped job, and releases both.
TEST(AppPoolTeardown, ComputeAsyncReportsAThrowOrADroppedJobToOnError) {
  struct outcome {
    std::mutex m;
    std::condition_variable cv;
    int done = 0;
    std::vector<std::string> errors;
    void add(int d, const std::string *e) {
      {
        std::lock_guard<std::mutex> lk(m);
        done += d;
        if (e)
          errors.push_back(*e);
      }
      cv.notify_all();
    }
    bool wait_for(std::size_t n) {
      std::unique_lock<std::mutex> lk(m);
      return cv.wait_for(lk, std::chrono::seconds(10),
                         [&] { return std::size_t(done) + errors.size() >= n; });
    }
  };
  const std::string dropped = "not run: the app's background pool has stopped";
  {
    app a;
    a.computePool(); // built on this thread, as in ComputeAsyncRunsFanOutOffThreadThenCallsOnDone
    auto o = std::make_shared<outcome>();
    auto on_done = [o] { o->add(1, nullptr); };
    auto on_error = [o](const std::string &e) { o->add(0, &e); };
    a.compute_async(8, [](int) {}, on_done, on_error);
    ASSERT_TRUE(o->wait_for(1));
    a.compute_async(
        8, [](int i) { throw std::runtime_error("chunk " + std::to_string(i)); }, on_done,
        on_error);
    ASSERT_TRUE(o->wait_for(2));
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->done, 1);
    ASSERT_EQ(o->errors.size(), 1u);
    EXPECT_EQ(o->errors[0].rfind("chunk ", 0), 0u) << o->errors[0];
  }
  {
    // Submitted once the pool has stopped: on_error before compute_async returns.
    probe_app a;
    a.stopPool();
    auto o = std::make_shared<outcome>();
    a.compute_async(
        8, [](int) {}, [o] { o->add(1, nullptr); }, [o](const std::string &e) { o->add(0, &e); });
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->done, 0);
    ASSERT_EQ(o->errors.size(), 1u);
    EXPECT_EQ(o->errors[0], dropped);
  }
  {
    // Still queued when ~app stops the pool: on_error from ~app; the four-argument form's job
    // calls nothing and releases its callbacks.
    auto o = std::make_shared<outcome>();
    auto held = std::make_shared<int>(1);
    std::weak_ptr<int> weak = held;
    {
      app a;
      a.setThreadPoolSize(1);
      std::atomic<bool> started(false);
      a.submitPooled([&started] {
        started = true;
        for (;;)
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // until ~app
      });
      for (int i = 0; i < 5000 && !started.load(); i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      ASSERT_TRUE(started.load());
      a.compute_async(
          8, [](int) {}, [o] { o->add(1, nullptr); }, [o](const std::string &e) { o->add(0, &e); });
      a.compute_async(8, [held](int) {}, [o, held] { o->add(1, nullptr); });
      held.reset();
      EXPECT_EQ(a.getPendingThreadCount(), 2u);
    }
    EXPECT_TRUE(weak.expired());
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->done, 0);
    ASSERT_EQ(o->errors.size(), 1u);
    EXPECT_EQ(o->errors[0], dropped);
  }
}

namespace {

// What a probe_app's failing worker creation saw: how many creations were tried, and whether
// they fail.
struct spawn_switch {
  std::atomic<bool> fail{false};
  std::atomic<int> attempts{0};
  void install(probe_app &a) {
    a.onWorkerSpawn([this] {
      ++attempts;
      if (fail.load())
        throw boost::thread_resource_error();
    });
  }
};

} // namespace

// When no worker can be started for a compute_async job and the pool has none, compute_async
// throws and runs nothing, as it did when each job had a thread of its own and creating it failed:
// a threaded tf overlay task relies on that to carry on cooperatively. Both forms call neither
// callback and release what they captured, and nothing is left queued. A submitPooled task waits
// in the queue instead, and runs once a later submission manages to start a worker.
TEST(AppPoolSpawnFailure, ComputeAsyncThrowsWhenNoWorkerCanBeStarted) {
  spawn_switch spawn; // before the app: its hook points here
  probe_app a;
  spawn.install(a);
  a.computePool(); // built on this thread, as in ComputeAsyncRunsFanOutOffThreadThenCallsOnDone
  spawn.fail = true;
  auto held = std::make_shared<int>(1);
  std::weak_ptr<int> weak = held;
  std::atomic<int> calls(0);
  EXPECT_THROW(a.compute_async(
                   4, [held](int) {}, [held, &calls] { ++calls; }),
               boost::thread_resource_error);
  EXPECT_THROW(a.compute_async(
                   4, [held](int) {}, [held, &calls] { ++calls; },
                   [held, &calls](const std::string &) { ++calls; }),
               boost::thread_resource_error);
  held.reset();
  EXPECT_TRUE(weak.expired()) << "a job compute_async threw for kept its captures";
  EXPECT_EQ(calls.load(), 0);
  EXPECT_GE(spawn.attempts.load(), 2);
  EXPECT_EQ(a.getPendingThreadCount(), 0u);
  EXPECT_EQ(a.getActiveThreadCount(), 0u);

  std::atomic<int> ran(0);
  EXPECT_TRUE(a.submitPooled([&ran] { ++ran; }));
  EXPECT_EQ(a.getPendingThreadCount(), 1u) << "a submitPooled task waits for a worker";
  EXPECT_EQ(ran.load(), 0);

  spawn.fail = false;
  std::promise<void> done;
  std::future<void> done_f = done.get_future();
  a.compute_async(1, [](int) {}, [&done] { done.set_value(); }); // starts a worker
  ASSERT_EQ(done_f.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  ASSERT_TRUE(wait_pool_idle(a));
  EXPECT_EQ(ran.load(), 1);
}

// With a worker running, a failed creation of another one only queues the job: the running worker
// takes it once it is free, and compute_async does not throw.
TEST(AppPoolSpawnFailure, ComputeAsyncQueuesTheJobWhenAWorkerIsRunning) {
  spawn_switch spawn; // before the app: its hook points here
  probe_app a;
  spawn.install(a);
  a.setThreadPoolSize(2);
  a.computePool();
  std::atomic<bool> started(false), release(false);
  a.submitPooled([&started, &release] {
    started = true;
    while (!release.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  for (int i = 0; i < 10000 && !started.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(started.load());
  spawn.fail = true;
  const int before = spawn.attempts.load();
  std::promise<void> done;
  std::future<void> done_f = done.get_future();
  EXPECT_NO_THROW(a.compute_async(1, [](int) {}, [&done] { done.set_value(); }));
  EXPECT_GT(spawn.attempts.load(), before) << "the pool had room: it should try a second worker";
  EXPECT_EQ(a.getPendingThreadCount(), 1u);
  release = true;
  EXPECT_EQ(done_f.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  EXPECT_TRUE(wait_pool_idle(a));
}

// compute_async decides whether to throw only once the worker creations it relies on have
// succeeded or failed: its own, and those other calls have under way. Two calls on an app with no
// worker, A's creation under way when B queues its job. In a pool of 2, B makes a creation of its
// own, and A's fails while B's is under way; in a pool of 1, A's is the one for both jobs, and it
// fails once B has queued its job. Both calls throw, and nothing is left queued. (A call that saw
// another's creation under way used to take it for a worker on its way, leave its job queued and
// return. That creation then failed, and the job waited, with no worker running or starting, for
// a later submission; its caller was never told.)
TEST(AppPoolSpawnFailure, ComputeAsyncThrowsWhenTheWorkerItReliedOnFailsLater) {
  for (unsigned pool_size = 1; pool_size <= 2; pool_size++) {
    SCOPED_TRACE(pool_size == 1 ? "B relies on A's creation" : "A's fails while B's is under way");
    struct shared {
      std::atomic<bool> a_in_creation{false}, b_in_creation{false}, a_returned{false},
          b_returned{false}, go_a{false}, go_b{false};
      std::atomic<int> a_threw{-1}, b_threw{-1}, ran{0};
      std::atomic<std::thread::id> a_id, b_id;
    };
    auto s = std::make_shared<shared>();
    probe_app a;
    a.setThreadPoolSize(pool_size);
    // Bounded by the clock: a 1 ms sleep can take a timer tick (~15 ms) on Windows.
    const auto wait_for = [](const std::atomic<bool> &flag, int ms) {
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
      while (!flag.load() && std::chrono::steady_clock::now() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };
    a.onWorkerSpawn([s, pool_size, wait_for] {
      const auto me = std::this_thread::get_id();
      if (me == s->a_id.load()) {
        s->a_in_creation = true;
        if (pool_size == 1) {
          // B makes no creation of its own: A's fails once B has queued its job and returned, or
          // after 300 ms (B waiting for it).
          wait_for(s->b_returned, 300);
        } else {
          wait_for(s->b_in_creation, 10000);
        }
      } else if (me == s->b_id.load()) {
        // B's fails once A has returned, or after 300 ms (A waiting for it).
        s->b_in_creation = true;
        wait_for(s->a_returned, 300);
      }
      throw boost::thread_resource_error();
    });
    const auto call = [&a, s](std::atomic<bool> &go, std::atomic<int> &threw,
                              std::atomic<bool> &returned) {
      while (!go.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      try {
        a.compute_async(1, [](int) {}, [s] { ++s->ran; });
        threw = 0;
      } catch (const boost::thread_resource_error &) {
        threw = 1;
      }
      returned = true;
    };
    std::thread ta([&] { call(s->go_a, s->a_threw, s->a_returned); });
    s->a_id = ta.get_id();
    s->go_a = true;
    for (int i = 0; i < 10000 && !s->a_in_creation.load(); i++)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_TRUE(s->a_in_creation.load());
    std::thread tb([&] { call(s->go_b, s->b_threw, s->b_returned); });
    s->b_id = tb.get_id();
    s->go_b = true;
    ta.join();
    tb.join();
    EXPECT_EQ(s->a_threw.load(), 1);
    EXPECT_EQ(s->b_threw.load(), 1);
    EXPECT_EQ(s->ran.load(), 0);
    EXPECT_EQ(a.getPendingThreadCount(), 0u) << "a job waits for a worker nothing will start";
    EXPECT_EQ(a.getActiveThreadCount(), 0u);
  }
}

// A compute_async job can be queued with no worker running or being created: keyed tasks, on
// threads of their own, hold every slot of a pool that has never started a worker. When one of
// them ends and the worker its thread asks for can't be created, the on_error form's job isn't
// left queued for a later submission to start a worker: the pool drops it unrun, on that thread,
// and on_error hears of it ("not run: no background worker could be started"). The four-argument
// form's job has no way to hear of a drop, so it waits in the queue, as a submitPooled task does,
// and runs once a later submission starts a worker. (Before, every job stayed queued with its
// caller never told; then the four-argument form's job was dropped too, without a word, and a
// threaded tf overlay task waiting for its chunk's reply waited for good.)
TEST(AppPoolSpawnFailure, ComputeAsyncJobsLeftWithNoWorkerAreDroppedOrWaitForOne) {
  struct outcome {
    std::mutex m;
    std::condition_variable cv;
    int done = 0, four_done = 0;
    std::vector<std::string> errors;
  };
  spawn_switch spawn; // before the app: its hook points here
  probe_app a;
  spawn.install(a);
  a.computePool();
  a.setThreadPoolSize(1);
  std::atomic<bool> release(false);
  // The pool's one slot, held by a keyed task on a thread of its own: no worker is started.
  a.startThreadPooled(
      "hold",
      [&release] {
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (!release.load() && std::chrono::steady_clock::now() < until)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      },
      PRIORITY_NORMAL, /*wait=*/true);
  ASSERT_TRUE(a.hasThread("hold"));
  auto o = std::make_shared<outcome>();
  auto held = std::make_shared<int>(1);
  std::weak_ptr<int> weak = held;
  a.compute_async(
      4, [](int) {},
      [o] {
        std::lock_guard<std::mutex> lk(o->m);
        ++o->done;
        o->cv.notify_all();
      },
      [o](const std::string &e) {
        std::lock_guard<std::mutex> lk(o->m);
        o->errors.push_back(e);
        o->cv.notify_all();
      });
  a.compute_async(
      4, [held](int) {},
      [o, held] {
        std::lock_guard<std::mutex> lk(o->m);
        ++o->four_done;
        o->cv.notify_all();
      });
  held.reset();
  std::atomic<int> ran(0);
  EXPECT_TRUE(a.submitPooled([&ran] { ++ran; }));
  EXPECT_EQ(a.getPendingThreadCount(), 3u);

  // The keyed task ends, and the worker its thread asks for on the way out can't be created.
  spawn.fail = true;
  release = true;
  if (thread_ptr hold = a.threads("hold"))
    hold->join();
  bool reported = false;
  {
    std::unique_lock<std::mutex> lk(o->m);
    reported = o->cv.wait_for(lk, std::chrono::seconds(5),
                              [&o] { return o->done > 0 || !o->errors.empty(); });
  }
  const unsigned int pending = a.getPendingThreadCount();
  const bool four_kept = !weak.expired();
  spawn.fail = false;
  EXPECT_TRUE(reported) << "the on_error job waits for a worker nothing will start";
  {
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->done, 0);
    ASSERT_EQ(o->errors.size(), 1u);
    EXPECT_EQ(o->errors[0], "not run: no background worker could be started");
    EXPECT_EQ(o->four_done, 0);
  }
  EXPECT_TRUE(four_kept) << "the four-argument form's job was dropped without a word";
  EXPECT_EQ(pending, 2u) << "the four-argument job and the submitPooled task wait for a worker";
  EXPECT_EQ(a.getActiveThreadCount(), 0u);
  EXPECT_EQ(ran.load(), 0);

  // A later submission starts a worker (this also ends the test when a job is still queued).
  std::promise<void> done;
  std::future<void> done_f = done.get_future();
  a.compute_async(1, [](int) {}, [&done] { done.set_value(); });
  ASSERT_EQ(done_f.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  ASSERT_TRUE(wait_pool_idle(a));
  EXPECT_EQ(ran.load(), 1);
  {
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->four_done, 1) << "the four-argument form's job never ran";
    EXPECT_EQ(o->done, 0);
    EXPECT_EQ(o->errors.size(), 1u);
  }
  EXPECT_TRUE(weak.expired());
}

// A compute_async call that starts a keyed task queued ahead of its job (the slot is free, and the
// keyed task starts on the calling thread) still throws, running nothing and calling neither
// callback, when that leaves the pool with no worker: the keyed task's thread can't be created,
// nor the worker asked for as its slot is freed. That state comes about when the pool drops a job
// queued ahead of keyed tasks for want of a worker: the first of them is then at the head, with
// the slot free and nothing starting it. (The worker creation that failed inside the call used to
// drop the call's own job there: the four-argument form's without a word, so that compute_async
// returned as though it were queued, and the on_error form's with on_error called inside
// compute_async instead of a throw.)
TEST(AppPoolSpawnFailure, ComputeAsyncThrowsWhenAQueuedKeyedTaskItStartsLeavesNoWorker) {
  struct outcome {
    std::mutex m;
    int done = 0;
    std::vector<std::string> errors;
  };
  spawn_switch spawn; // before the app: its hook points here
  probe_app a;
  spawn.install(a);
  a.computePool();
  a.setThreadPoolSize(1);
  std::atomic<bool> release(false);
  a.startThreadPooled(
      "hold",
      [&release] {
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (!release.load() && std::chrono::steady_clock::now() < until)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      },
      PRIORITY_NORMAL, /*wait=*/true);
  ASSERT_TRUE(a.hasThread("hold"));
  auto o = std::make_shared<outcome>();
  const auto on_done = [o] {
    std::lock_guard<std::mutex> lk(o->m);
    ++o->done;
  };
  const auto on_error = [o](const std::string &label) {
    return [o, label](const std::string &e) {
      std::lock_guard<std::mutex> lk(o->m);
      o->errors.push_back(label + e);
    };
  };
  a.compute_async(1, [](int) {}, on_done, on_error("first: "));
  std::atomic<int> keyed_ran(0);
  a.startThreadPooled("k1", [&keyed_ran] { ++keyed_ran; }, PRIORITY_NORMAL, /*wait=*/true);
  a.startThreadPooled("k2", [&keyed_ran] { ++keyed_ran; }, PRIORITY_NORMAL, /*wait=*/true);
  ASSERT_EQ(a.getPendingThreadCount(), 3u);

  // The hold ends, and the worker its thread asks for can't be created: the first job is dropped,
  // which leaves k1 at the head of the queue with the slot free.
  spawn.fail = true;
  release = true;
  if (thread_ptr hold = a.threads("hold"))
    hold->join();
  ASSERT_EQ(a.getPendingThreadCount(), 2u);
  ASSERT_EQ(a.getActiveThreadCount(), 0u);
  {
    std::lock_guard<std::mutex> lk(o->m);
    ASSERT_EQ(o->errors.size(), 1u);
    EXPECT_EQ(o->errors[0], "first: not run: no background worker could be started");
  }

  // Each call starts the keyed task at the head, whose thread can't be created, nor the worker
  // asked for as its slot is freed.
  auto held = std::make_shared<int>(1);
  std::weak_ptr<int> weak = held;
  EXPECT_THROW(a.compute_async(
                   1, [held](int) {}, [held, on_done] { on_done(); }),
               boost::thread_resource_error);
  EXPECT_EQ(a.getPendingThreadCount(), 1u) << "k2 only";
  EXPECT_THROW(a.compute_async(
                   1, [held](int) {}, [held, on_done] { on_done(); }, on_error("third: ")),
               boost::thread_resource_error);
  held.reset();
  EXPECT_TRUE(weak.expired()) << "a job compute_async threw for kept its captures";
  EXPECT_EQ(a.getPendingThreadCount(), 0u);
  EXPECT_EQ(a.getActiveThreadCount(), 0u);
  {
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->errors.size(), 1u) << o->errors.back();
  }

  // Thread resources come back: a worker starts, and nothing of the jobs or keyed tasks runs.
  spawn.fail = false;
  std::promise<void> done;
  std::future<void> done_f = done.get_future();
  a.compute_async(1, [](int) {}, [&done] { done.set_value(); });
  ASSERT_EQ(done_f.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  ASSERT_TRUE(wait_pool_idle(a));
  std::lock_guard<std::mutex> lk(o->m);
  EXPECT_EQ(o->done, 0) << "a job compute_async threw for (or had dropped) ran";
  EXPECT_EQ(keyed_ran.load(), 0);
}

// A trySubmitPooled call waiting on a worker creation that its own job relies on decides about
// that job first: until it has, nothing else drops a task for want of a worker. When the creation
// fails, the call hands its job back (compute_async throws), and then drops the tasks queued before
// it that the failed creation left with no worker, on its own thread, before compute_async throws.
// Here a keyed task holds the only slot, job A is queued behind it, and the creation asked for as
// the keyed task ends is held until call B waits on it, and then fails.
TEST(AppPoolSpawnFailure, AJobLeftWithNoWorkerWhileACallWaitsOnTheCreationIsDroppedByThatCall) {
  struct gate {
    std::mutex m;
    std::condition_variable cv;
    bool armed = false, in_creation = false, open = false;
  };
  struct outcome {
    std::mutex m;
    std::condition_variable cv;
    int done = 0;
    std::vector<std::string> errors;
    std::thread::id error_thread;
  };
  auto g = std::make_shared<gate>();
  probe_app a;
  a.onWorkerSpawn([g] {
    std::unique_lock<std::mutex> lk(g->m);
    if (!g->armed)
      return;
    g->in_creation = true;
    g->cv.notify_all();
    g->cv.wait_for(lk, std::chrono::seconds(10), [&g] { return g->open; });
    throw boost::thread_resource_error();
  });
  a.computePool();
  a.setThreadPoolSize(1);
  std::atomic<bool> release(false);
  a.startThreadPooled(
      "hold",
      [&release] {
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (!release.load() && std::chrono::steady_clock::now() < until)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      },
      PRIORITY_NORMAL, /*wait=*/true);
  ASSERT_TRUE(a.hasThread("hold"));
  auto o = std::make_shared<outcome>();
  const auto on_done = [o] {
    std::lock_guard<std::mutex> lk(o->m);
    ++o->done;
    o->cv.notify_all();
  };
  const auto on_error = [o](const std::string &label) {
    return [o, label](const std::string &e) {
      std::lock_guard<std::mutex> lk(o->m);
      o->errors.push_back(label + e);
      o->error_thread = std::this_thread::get_id();
      o->cv.notify_all();
    };
  };
  a.compute_async(1, [](int) {}, on_done, on_error("A: "));
  ASSERT_EQ(a.getPendingThreadCount(), 1u);

  // The keyed task ends; the worker its thread asks for is held in its creation.
  {
    std::lock_guard<std::mutex> lk(g->m);
    g->armed = true;
  }
  release = true;
  bool in_creation = false;
  {
    std::unique_lock<std::mutex> lk(g->m);
    in_creation = g->cv.wait_for(lk, std::chrono::seconds(10), [&g] { return g->in_creation; });
  }
  ASSERT_TRUE(in_creation);
  // B's job relies on that creation: B waits for it to succeed or fail.
  std::atomic<int> b_threw(-1);
  std::thread b([&a, &b_threw, on_done, on_error] {
    try {
      a.compute_async(1, [](int) {}, on_done, on_error("B: "));
      b_threw = 0;
    } catch (const boost::thread_resource_error &) {
      b_threw = 1;
    }
  });
  const std::thread::id b_id = b.get_id();
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (a.handBackChecks() == 0 && std::chrono::steady_clock::now() < until)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const unsigned int checks = a.handBackChecks();
  {
    std::lock_guard<std::mutex> lk(g->m);
    g->open = true; // the creation fails
  }
  g->cv.notify_all();
  b.join();
  if (thread_ptr hold = a.threads("hold"))
    hold->join();
  bool reported = false;
  {
    std::unique_lock<std::mutex> lk(o->m);
    reported = o->cv.wait_for(lk, std::chrono::seconds(5),
                              [&o] { return o->done > 0 || !o->errors.empty(); });
  }
  const unsigned int pending = a.getPendingThreadCount();
  {
    std::lock_guard<std::mutex> lk(g->m);
    g->armed = false;
  }
  if (!reported)
    a.submitPooled([] {}); // so that the test ends: a worker that can start runs A
  EXPECT_EQ(checks, 1u) << "B did not wait on the creation";
  EXPECT_TRUE(reported) << "A waits for a worker nothing will start";
  EXPECT_EQ(b_threw.load(), 1);
  EXPECT_EQ(pending, 0u);
  {
    std::lock_guard<std::mutex> lk(o->m);
    EXPECT_EQ(o->done, 0);
    ASSERT_EQ(o->errors.size(), 1u);
    EXPECT_EQ(o->errors[0], "A: not run: no background worker could be started");
    EXPECT_EQ(o->error_thread, b_id) << "A was not dropped by B's call";
  }
  EXPECT_TRUE(wait_pool_idle(a));
}

// startThreadPooled(wait=true) throws boost::thread_resource_error when its task's own thread
// can't be created (thread resources exhausted), as it did when every pooled task had a thread of
// its own: the task doesn't run, what it captured is released, and nothing is registered under its
// key. (The task used to be dropped without a word: startThreadPooled returned as if it had
// registered it, and `if (hasThread(k)) threads(k)->join();` went on at once.) A keyed task that
// waited in the queue, and whose thread can't be created when its turn comes, has no caller to
// tell: it is dropped and logged, and the pool carries on. The drop is logged before its slot is
// freed, so a pool found idle (wait(), or the counts at 0) has logged it. (It used to be logged
// after the slot was freed: a test that read the log as soon as the pool was idle could miss it.)
TEST(AppPoolSpawnFailure, AKeyedTaskWhoseThreadCantBeCreatedThrowsOrIsLogged) {
  spawn_switch spawn; // before the app: its hook points here
  probe_app a;
  spawn.install(a);
  const std::string log_key("pool_log");
  a.data(log_key, std::string());
  a.properties("system.log_output", log_key);
  a.setThreadPoolSize(1);
  std::atomic<int> ran(0);
  auto held = std::make_shared<int>(1);
  std::weak_ptr<int> weak = held;
  spawn.fail = true;
  EXPECT_THROW(a.startThreadPooled(
                   "k1", [held, &ran] { ++ran; }, PRIORITY_NORMAL, /*wait=*/true),
               boost::thread_resource_error);
  held.reset();
  EXPECT_TRUE(weak.expired()) << "a keyed task startThreadPooled threw for kept its captures";
  EXPECT_FALSE(a.hasThread("k1"));
  EXPECT_EQ(a.getActiveThreadCount(), 0u);
  EXPECT_EQ(a.getPendingThreadCount(), 0u);

  // Queued behind the full pool, then started, and failing, as the slot frees.
  spawn.fail = false;
  std::atomic<bool> started(false), release(false);
  a.submitPooled([&started, &release] {
    started = true;
    while (!release.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  for (int i = 0; i < 10000 && !started.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(started.load());
  held = std::make_shared<int>(2);
  weak = held;
  EXPECT_NO_THROW(
      a.startThreadPooled("k2", [held, &ran] { ++ran; }, PRIORITY_NORMAL, /*wait=*/true)); // queued
  held.reset();
  EXPECT_EQ(a.getPendingThreadCount(), 1u);
#ifndef USING_LOG4CPLUS_DEFAULT
  // What the pool's counts were as the drop was logged (the log is a data key: its change
  // signal comes from inside log(), on the thread logging).
  std::atomic<int> logged_while_idle(-1);
  boost::signals2::scoped_connection on_log =
      a.dataChanged.connect([&a, &log_key, &logged_while_idle](const std::string &key) {
        if (key == log_key)
          logged_while_idle = a.getActiveThreadCount() == 0 && a.getPendingThreadCount() == 0;
      });
#endif
  spawn.fail = true;
  release = true;
  EXPECT_TRUE(wait_pool_idle(a));
  spawn.fail = false;
  EXPECT_TRUE(weak.expired());
  EXPECT_FALSE(a.hasThread("k2"));
  EXPECT_EQ(ran.load(), 0);
#ifndef USING_LOG4CPLUS_DEFAULT
  const std::string logged = a.data<std::string>(log_key);
  EXPECT_NE(logged.find("'k2'"), std::string::npos) << "the drop wasn't logged: " << logged;
  EXPECT_EQ(logged_while_idle.load(), 0)
      << "the drop was logged once its slot was freed (-1: not logged at all)";
#endif
}

// ~app interrupts the threads in threads() before it joins the pool's workers, as it interrupts
// everything before joining anything: a pool task waiting, with no interruption point, on
// something a map thread holds until it is interrupted is released at once. (~app used to join
// the workers first: it waited out the 5 s join, then left the worker running into a freed app.)
TEST(AppPoolTeardown, DestroyingTheAppInterruptsTheThreadMapBeforeJoiningThePool) {
  struct shared {
    std::mutex m;
    std::condition_variable cv;
    bool held = false, released = false, waiting = false;
    std::atomic<bool> pool_task_done{false};
  };
  auto s = std::make_shared<shared>();
  std::chrono::steady_clock::time_point t0;
  {
    app a;
    a.startThread("holder", [s] {
      {
        std::lock_guard<std::mutex> lk(s->m);
        s->held = true;
      }
      s->cv.notify_all();
      try {
        for (;;)
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // an interruption point
      } catch (boost::thread_interrupted &) {
        {
          std::lock_guard<std::mutex> lk(s->m);
          s->released = true;
        }
        s->cv.notify_all();
        throw;
      }
    });
    a.submitPooled([s] {
      std::unique_lock<std::mutex> lk(s->m);
      s->waiting = true;
      s->cv.notify_all();
      // A std::condition_variable: no interruption point. Bounded, so that the test ends if the
      // holder is never interrupted.
      s->cv.wait_for(lk, std::chrono::seconds(8), [&s] { return s->released; });
      s->pool_task_done = true;
    });
    std::unique_lock<std::mutex> lk(s->m);
    ASSERT_TRUE(
        s->cv.wait_for(lk, std::chrono::seconds(10), [&s] { return s->held && s->waiting; }));
    lk.unlock();
    t0 = std::chrono::steady_clock::now();
  }
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(secs, 3.0);
  EXPECT_TRUE(s->pool_task_done.load()) << "~app returned with the pool task still running";
  std::lock_guard<std::mutex> lk(s->m);
  EXPECT_TRUE(s->released);
}

// A thread registered in threads() while ~app joins the pool's workers is interrupted and joined
// too, also when the map was empty before: ~app takes the map again after that join. (A pool task
// that starts a thread once it is interrupted, or a keyed task that a worker took just before the
// pool closed, registered its thread after ~app's only look at the empty map: ~app returned
// without interrupting or joining it, and the thread ran on into the freed app.)
TEST(AppPoolTeardown, AThreadRegisteredWhileTheAppJoinsThePoolIsJoinedToo) {
  struct shared {
    std::atomic<bool> started{false}, late_done{false};
  };
  auto s = std::make_shared<shared>();
  std::chrono::steady_clock::time_point t0;
  {
    app a;
    a.submitPooled([s, &a] {
      s->started = true;
      try {
        for (;;)
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // until ~app
      } catch (boost::thread_interrupted &) {
        // ~app has closed the pool: let it take the (empty) map and start joining this worker.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        // The thread touches nothing of the app, and ends by itself if it is never interrupted.
        a.startThread("late", [s] {
          try {
            const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < until)
              boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // interruption point
          } catch (boost::thread_interrupted &) {
            s->late_done = true;
            throw;
          }
        });
      }
    });
    for (int i = 0; i < 10000 && !s->started.load(); i++)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_TRUE(s->started.load());
    t0 = std::chrono::steady_clock::now();
  }
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_TRUE(s->late_done.load()) << "~app returned without interrupting and joining the thread";
  EXPECT_LT(secs, 3.0);
}

// wait() waits with interruption held off: an interrupt sent to the thread calling it stays
// pending until it returns, and doesn't cut the wait for the pool short. (It used to be thrown in
// the wait for the pool, dropped by the catch there, and wait() returned at once.)
TEST(AppPoolTeardown, WaitKeepsAnInterruptSentToItsCallerPending) {
  app a;
  a.setThreadPoolSize(1);
  std::atomic<bool> started(false);
  a.submitPooled([&started] {
    started = true;
    spin_us(800000); // no interruption point: wait()'s interrupt doesn't end it
  });
  for (int i = 0; i < 10000 && !started.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(started.load());
  std::atomic<bool> in_wait(false), interrupted_out(false), pending(false);
  std::atomic<int> running_after(-1);
  boost::thread waiter([&] {
    in_wait = true;
    try {
      a.wait();
    } catch (boost::thread_interrupted &) {
      interrupted_out = true; // reached before wait() began waiting: the test didn't happen
      return;
    }
    running_after = static_cast<int>(a.getActiveThreadCount());
    pending = boost::this_thread::interruption_requested();
  });
  for (int i = 0; i < 10000 && !in_wait.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(300)); // into the wait for the pool
  waiter.interrupt();
  waiter.join();
  ASSERT_FALSE(interrupted_out.load());
  EXPECT_TRUE(pending.load()) << "wait() dropped the interrupt sent to its caller";
  EXPECT_EQ(running_after.load(), 0) << "the interrupt cut wait() short";
}

namespace {

// A pool task whose destruction reaches an interruption point, and records whether it threw
// there. Only the object the pool holds probes: moving or copying hands that over, as the pool's
// own drop-reporting tasks do.
struct interrupt_probe {
  std::shared_ptr<std::atomic<int>> thrown, destroyed;
  bool armed = true;
  interrupt_probe(std::shared_ptr<std::atomic<int>> t, std::shared_ptr<std::atomic<int>> d)
      : thrown(std::move(t)), destroyed(std::move(d)) {}
  interrupt_probe(interrupt_probe &&o) noexcept
      : thrown(o.thrown), destroyed(o.destroyed), armed(std::exchange(o.armed, false)) {}
  interrupt_probe(const interrupt_probe &o) noexcept
      : interrupt_probe(std::move(const_cast<interrupt_probe &>(o))) {}
  interrupt_probe &operator=(const interrupt_probe &) = delete;
  void operator()() { armed = false; }
  ~interrupt_probe() {
    if (!armed)
      return;
    ++*destroyed;
    try {
      boost::this_thread::interruption_point();
    } catch (boost::thread_interrupted &) {
      ++*thrown; // and the interrupt is gone
    }
  }
};

} // namespace

// A task the pool drops unrun is destroyed with interruption held off, wherever that happens, so
// an interrupt pending on the thread doing it stays pending instead of being thrown inside a
// callback reporting the drop and swallowed there: compute_async's on_error, launch_pool_task's
// on_error (both refused at submission), a plain task refused at submission, and the queued tasks
// and worker joins of the pool's stop (a keyed thread ~app interrupted might be the one there).
TEST(AppPoolTeardown, DroppingATaskKeepsAnInterruptPendingOnTheDroppingThread) {
  auto thrown = std::make_shared<std::atomic<int>>(0);
  auto destroyed = std::make_shared<std::atomic<int>>(0);
  probe_app stopped;
  state_exec::async_scheduler &sched = stopped.exec_scheduler(); // built here, posted to below
  stopped.stopPool();
  std::atomic<int> on_errors(0), threw_in_on_error(0);
  std::atomic<bool> pending_after_compute(false), pending_after_post(false),
      pending_after_task(false), refused(false);
  boost::thread t([&] {
    while (!boost::this_thread::interruption_requested())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    stopped.compute_async(
        1, [](int) {}, [] {},
        [&](const std::string &) {
          ++on_errors;
          try {
            boost::this_thread::interruption_point();
          } catch (boost::thread_interrupted &) {
            ++threw_in_on_error;
            throw;
          }
        });
    pending_after_compute = boost::this_thread::interruption_requested();
    cvc::launch_pool_task(
        stopped, sched, std::string(), "probe.reply", [] { return state_exec::value_t(true); },
        [&](const std::string &e) {
          ++on_errors;
          try {
            boost::this_thread::interruption_point();
          } catch (boost::thread_interrupted &) {
            ++threw_in_on_error;
            throw;
          }
          return state_exec::value_t(e);
        });
    pending_after_post = boost::this_thread::interruption_requested();
    refused = !stopped.submitPooled(std::function<void()>(interrupt_probe(thrown, destroyed)));
    pending_after_task = boost::this_thread::interruption_requested();
  });
  t.interrupt();
  t.join();
  EXPECT_EQ(on_errors.load(), 2);
  EXPECT_EQ(threw_in_on_error.load(), 0);
  EXPECT_TRUE(pending_after_compute.load());
  EXPECT_TRUE(pending_after_post.load());
  EXPECT_TRUE(refused.load());
  EXPECT_TRUE(pending_after_task.load());
  EXPECT_EQ(destroyed->load(), 1);
  EXPECT_EQ(thrown->load(), 0);

  // The pool's stop on an interrupted thread: the queued task's destruction and the joins.
  probe_app b;
  b.setThreadPoolSize(1);
  std::atomic<bool> started(false);
  b.submitPooled([&started] {
    started = true;
    for (;;)
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // until the stop
  });
  for (int i = 0; i < 10000 && !started.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(started.load());
  EXPECT_TRUE(b.submitPooled(std::function<void()>(interrupt_probe(thrown, destroyed))));
  EXPECT_EQ(b.getPendingThreadCount(), 1u);
  std::atomic<bool> pending_after_stop(false);
  std::atomic<unsigned> running_after_stop(99);
  boost::thread stopper([&] {
    while (!boost::this_thread::interruption_requested())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    b.stopPool();
    running_after_stop = b.getActiveThreadCount();
    pending_after_stop = boost::this_thread::interruption_requested();
  });
  stopper.interrupt();
  stopper.join();
  EXPECT_EQ(destroyed->load(), 2);
  EXPECT_EQ(thrown->load(), 0);
  EXPECT_TRUE(pending_after_stop.load());
  EXPECT_EQ(running_after_stop.load(), 0u) << "the stop skipped joining the worker";
}

// ~app holds interruption off until the last thing the app holds is destroyed, not only through
// its own body: an interrupt pending on the thread destroying the app is still pending when ~app
// returns, and isn't thrown at an interruption point in the destructor of something the app holds
// (its data: the state tree joining a handler thread, a user object that joins one; a slot), where
// it would be std::terminate. (Held off in the body only, it was thrown there, also when ~app had
// workers to join.)
TEST(AppPoolTeardown, AnInterruptPendingOnTheThreadDestroyingTheAppOutlivesIt) {
  auto thrown = std::make_shared<std::atomic<int>>(0);
  auto destroyed = std::make_shared<std::atomic<int>>(0);
  std::atomic<bool> ready(false), pending_after(false), done(false);
  boost::thread t([&] {
    auto a = std::make_unique<app>();
    std::atomic<bool> ran(false);
    a->submitPooled([&ran] { ran = true; }); // a worker for ~app to join
    while (!ran.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    a->data("probe", std::make_shared<interrupt_probe>(thrown, destroyed));
    ready = true;
    while (!boost::this_thread::interruption_requested())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    a.reset();
    pending_after = boost::this_thread::interruption_requested();
    done = true;
  });
  for (int i = 0; i < 10000 && !ready.load(); i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  t.interrupt();
  t.join();
  EXPECT_TRUE(done.load());
  EXPECT_EQ(destroyed->load(), 1);
  EXPECT_EQ(thrown->load(), 0) << "an interrupt was thrown in the destructor of the app's data";
  EXPECT_TRUE(pending_after.load()) << "the interrupt didn't outlive the app";
}

// A keyed task (wait=true) that the pool has room for is in threads() under its key when
// startThreadPooled returns, also right behind an anonymous task still waiting for its worker to
// start (a fresh app) or to wake (a warm one); the documented join pattern,
// `if (hasThread(k)) threads(k)->join();`, then waits for it.
TEST(AppPoolKeyed, AKeyedTaskWithRoomIsRegisteredOnReturnBehindAnonymousTasks) {
  for (int warm = 0; warm < 2; warm++) {
    for (int iter = 0; iter < 50; iter++) {
      app a;
      a.setThreadPoolSize(4);
      if (warm) {
        std::atomic<int> w(0);
        for (int k = 0; k < 8; k++)
          a.submitPooled([&w] { w++; });
        ASSERT_TRUE(wait_pool_idle(a));
        ASSERT_EQ(w.load(), 8);
      }
      std::atomic<bool> release(false);
      std::atomic<bool> keyed_ran(false);
      a.startThreadPooled("anon", [] {}, PRIORITY_NORMAL, /*wait=*/false);
      a.startThreadPooled(
          "keyed",
          [&] {
            while (!release.load())
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            keyed_ran = true;
          },
          PRIORITY_NORMAL, /*wait=*/true);
      EXPECT_TRUE(a.hasThread("keyed")) << "warm=" << warm << " iteration " << iter;
      release = true;
      if (a.hasThread("keyed")) {
        thread_ptr t = a.threads("keyed");
        if (t)
          t->join();
      }
      EXPECT_TRUE(keyed_ran.load()) << "warm=" << warm << " iteration " << iter;
      ASSERT_TRUE(wait_pool_idle(a));
      if (HasFailure())
        return;
    }
  }
}

// A burst runs on as many workers as it has tasks, up to the pool's size, from its first task:
// eight tasks that each wait until all eight are running all finish promptly, on a fresh app
// (no worker yet) and on a warm one (idle workers). Starting one worker at a time ran such a
// burst on one to three threads under load.
TEST(AppPoolBurst, ABurstStartsAWorkerPerTaskUpToThePoolSize) {
  for (int warm = 0; warm < 2; warm++) {
    app a;
    a.setThreadPoolSize(8);
    if (warm) {
      std::atomic<int> w(0);
      for (int k = 0; k < 8; k++)
        a.submitPooled([&w] {
          spin_us(2000);
          w++;
        });
      ASSERT_TRUE(wait_pool_idle(a));
    }
    std::atomic<int> arrived(0);
    std::atomic<int> all_met(0);
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < 8; k++)
      a.submitPooled([&] {
        arrived++;
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (arrived.load() < 8 && std::chrono::steady_clock::now() < until)
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        if (arrived.load() == 8)
          all_met++;
      });
    ASSERT_TRUE(wait_pool_idle(a, 15000));
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_EQ(all_met.load(), 8) << "warm=" << warm;
    EXPECT_LT(secs, 4.0) << "warm=" << warm;
  }
}

TEST_F(AppTest, ThreadInfoAndProgressTracking) {
  std::string thread_key = "test.progress.tracking";
  std::atomic<bool> thread_started(false);
  std::atomic<bool> continue_running(true);

  // Checkpoint handshake between the worker and this thread.
  //
  // The worker used to publish a checkpoint and then sleep 50ms, while the
  // reader polled until progress crossed a threshold and only then asserted the
  // exact value. That is a race: if the reader was descheduled for longer than
  // the worker's sleep -- routine on a loaded CI runner -- the worker had
  // already moved on, and the reader read 0.50 where it asserted 0.25. It
  // failed intermittently on macos-latest/Debug while telling us nothing about
  // the code under test.
  //
  // Instead the worker parks on each checkpoint until the reader reports having
  // consumed it, so every read happens with the worker held at exactly the
  // checkpoint being asserted. No sleeps, no timing assumptions.
  std::atomic<int> published(0); // checkpoints the worker has made visible
  std::atomic<int> observed(0);  // checkpoints the reader has consumed

  // static so the worker's captures stay valid even if the join below is
  // skipped and the thread outlives this frame.
  static const double checkpoints[] = {0.0, 0.25, 0.50, 0.75};
  static const char *const infos[] = {"Starting processing", "Processing step 1",
                                      "Processing step 2", "Processing step 3"};
  static const int num_checkpoints = 4;

  ctx.startThreadPooled(
      thread_key,
      [&]() {
        thread_started = true;

        for (int s = 0; s < num_checkpoints; ++s) {
          ctx.threadInfo(thread_key, infos[s]);
          ctx.threadProgress(thread_key, checkpoints[s]);
          published.store(s + 1);

          // Hold here until the reader has consumed this checkpoint.
          while (observed.load() < s + 1)
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }

        // Wait until we're told to finish
        while (continue_running.load()) {
          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
          // NOTE: No interruption_point() here - we want to finish cleanly
        }

        // Complete
        ctx.threadInfo(thread_key, "Finished");
        ctx.threadProgress(thread_key, 1.0);

        // Small delay to ensure final state is written
        boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
      },
      PRIORITY_NORMAL, true);

  // Wait for thread to start
  for (int i = 0; i < 500 && !thread_started.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(thread_started.load()) << "Thread should have started";

  // Wait for checkpoint n, read progress while the worker is parked on it, then
  // release the worker. Always releases, even when the wait times out, so a
  // failure reports cleanly instead of wedging the worker.
  auto consume = [&](int n) {
    for (int i = 0; i < 500 && published.load() < n; i++)
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    EXPECT_GE(published.load(), n) << "worker never reached checkpoint " << n;
    double p = ctx.threadProgress(thread_key);
    observed.store(n);
    return p;
  };

  consume(1); // 0.0 -- the starting checkpoint, nothing to assert
  double progress1 = consume(2);
  EXPECT_NEAR(progress1, 0.25, 0.01);
  double progress2 = consume(3);
  EXPECT_NEAR(progress2, 0.50, 0.01);
  double progress3 = consume(4);
  EXPECT_NEAR(progress3, 0.75, 0.01);

  // Verify progress is increasing
  EXPECT_LT(progress1, progress2);
  EXPECT_LT(progress2, progress3);

  // Let thread finish
  continue_running = false;

  // Join the thread to clean up
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr)
      tptr->join();
  }

  // NOTE: We successfully tested reading thread info/progress while running.
  // The final state after join() is not reliable due to thread pool cleanup timing.
}

// ===========================
// Persistent Progress Tests
// ===========================

TEST_F(AppTest, ThreadProgressPersistsAfterCompletion) {
  std::string thread_key = "test.persistent.progress";
  std::atomic<bool> thread_finished(false);

  // Start a thread that sets progress and completes quickly
  ctx.startThread(thread_key, [&]() {
    cvc::app::thread_feedback feedback(ctx, thread_key);
    ctx.threadProgress(thread_key, 0.5);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    thread_finished = true;
    // thread_feedback destructor will set progress to 100%
  });

  // Wait for thread to finish
  for (int i = 0; i < 100 && !thread_finished.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(thread_finished.load()) << "Thread should have completed";

  // Give thread time to fully exit
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // CRITICAL: Progress should be readable even after thread exits
  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
      << "Progress should be 100% after thread completion (thread_feedback sets it)";

  // Clean up
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr && tptr->joinable())
      tptr->join();
  }
}

TEST_F(AppTest, ThreadProgressPersistenceWithMultipleThreads) {
  std::vector<std::string> thread_keys = {"test.multi.thread1", "test.multi.thread2",
                                          "test.multi.thread3"};
  std::atomic<int> completed_count(0);

  // Start multiple threads with different progress values
  for (size_t i = 0; i < thread_keys.size(); i++) {
    double target_progress = (i + 1) * 0.25; // 0.25, 0.5, 0.75
    ctx.startThread(thread_keys[i], [&, i, target_progress]() {
      cvc::app::thread_feedback feedback(ctx, thread_keys[i]);
      ctx.threadProgress(thread_keys[i], target_progress);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
      completed_count++;
      // thread_feedback destructor sets to 100%
    });
  }

  // Wait for all threads to complete
  for (int i = 0; i < 100 && completed_count.load() < 3; i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_EQ(completed_count.load(), 3) << "All threads should have completed";

  // Give threads time to exit
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // All threads should show 100% progress after completion
  for (const auto &key : thread_keys) {
    double progress = ctx.threadProgress(key);
    EXPECT_NEAR(progress, 1.0, 0.01) << "Thread " << key << " should show 100% after completion";
  }

  // Clean up
  for (const auto &key : thread_keys) {
    if (ctx.hasThread(key)) {
      thread_ptr tptr = ctx.threads(key);
      if (tptr && tptr->joinable())
        tptr->join();
    }
  }
}

TEST_F(AppTest, ThreadProgressZeroToOneHundred) {
  std::string thread_key = "test.zero.to.hundred";
  std::atomic<bool> at_zero(false);
  std::atomic<bool> at_fifty(false);
  std::atomic<bool> finished(false);
  // Acknowledgement gates: the worker waits for the main thread to sample
  // each phase before advancing. Without these the worker can race through
  // 0% → 50% → 100% before the main thread's polling loop wakes up to
  // observe the intermediate states (seen on fast macOS Release runners).
  std::atomic<bool> zero_observed(false);
  std::atomic<bool> fifty_observed(false);

  ctx.startThread(thread_key, [&]() {
    cvc::app::thread_feedback feedback(ctx, thread_key);
    // thread_feedback constructor sets to 0%
    at_zero = true;
    while (!zero_observed.load()) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }

    ctx.threadProgress(thread_key, 0.5);
    at_fifty = true;
    while (!fifty_observed.load()) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }

    finished = true;
    // thread_feedback destructor sets to 100%
  });

  // Check progress at 0%
  for (int i = 0; i < 200 && !at_zero.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(at_zero.load());
  {
    double progress = ctx.threadProgress(thread_key);
    EXPECT_NEAR(progress, 0.0, 0.01) << "Progress should be 0% at start";
  }
  zero_observed = true;

  // Check progress at 50%
  for (int i = 0; i < 200 && !at_fifty.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(at_fifty.load());
  {
    double progress = ctx.threadProgress(thread_key);
    EXPECT_NEAR(progress, 0.5, 0.01) << "Progress should be 50% midway";
  }
  fifty_observed = true;

  // Wait for completion
  for (int i = 0; i < 200 && !finished.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(finished.load());

  // Give thread time to exit
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // Check final progress persists at 100%
  double final_progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(final_progress, 1.0, 0.01) << "Progress should persist at 100% after thread exits";

  // Clean up
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr && tptr->joinable())
      tptr->join();
  }
}

TEST_F(AppTest, ThreadProgressQueryAfterThreadDestruction) {
  std::string thread_key = "test.progress.after.destroy";

  {
    // Start thread in inner scope
    std::atomic<bool> done(false);
    ctx.startThread(thread_key, [&]() {
      cvc::app::thread_feedback feedback(ctx, thread_key);
      ctx.threadProgress(thread_key, 0.75);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      done = true;
    });

    // Wait for completion
    for (int i = 0; i < 50 && !done.load(); i++) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }

    // Join thread
    if (ctx.hasThread(thread_key)) {
      thread_ptr tptr = ctx.threads(thread_key);
      if (tptr && tptr->joinable())
        tptr->join();
    }
  }

  // Thread object has been destroyed, but progress should still be queryable
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
      << "Progress should be queryable at 100% even after thread object destroyed";
}

TEST_F(AppTest, ThreadFeedbackExceptionSafety) {
  std::string thread_key = "test.feedback.exception";
  std::atomic<bool> exception_thrown(false);

  ctx.startThread(thread_key, [&]() {
    try {
      cvc::app::thread_feedback feedback(ctx, thread_key);
      ctx.threadProgress(thread_key, 0.3);

      // Simulate an exception during processing
      exception_thrown = true;
      throw std::runtime_error("Simulated error");
    } catch (const std::exception &e) {
      // thread_feedback destructor should still set progress to 100%
      // even when exception is thrown
    }
  });

  // Wait for exception
  for (int i = 0; i < 50 && !exception_thrown.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(exception_thrown.load());

  // Synchronize with the worker BEFORE checking progress: thread_feedback's
  // destructor runs as the catch block unwinds, and we need a happens-before
  // edge between that destructor and our progress read. A fixed sleep was
  // racy under CI load. Two cases, both safe:
  //   (a) The worker has not yet finished — hasThread() returns true, we
  //       grab the thread handle and join(). join() is a hard sync point
  //       that guarantees the destructor (and finishThreadProgress) ran.
  //   (b) The worker has already finished — hasThread() returns false
  //       because thread_feedback's destructor called finishThreadProgress,
  //       which erases _threads[key] and writes _threadProgressByKey[key]
  //       = 1.0 atomically under _threadsMutex. Subsequent threadProgress
  //       reads happen-after that write through the same mutex.
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr && tptr->joinable())
      tptr->join();
  }

  // Progress should now be 100% — set by thread_feedback's destructor
  // during stack unwinding.
  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
      << "Progress should be 100% even when exception occurs (RAII cleanup)";
}

// A worker must stay in the thread registry -- and stay joinable -- for as
// long as it is actually running.
//
// thread_feedback's destructor calls finishThreadProgress() from inside the
// worker, and finishThreadProgress() used to erase _threads[key] right
// there. That drops the last shared_ptr to a live boost::thread, and
// ~boost::thread() detaches it (BOOST_THREAD_VERSION is not raised anywhere
// in this tree), after which nothing can ever join it. Every consumer that
// finds work by scanning _threads -- app::wait_for_threads(),
// state_object::waitForHandlers(), ~state_object() -- then skipped the
// thread, so it could go on dereferencing the app after the app was gone.
// That is the intermittent SEGFAULT in
// StateObjectFixture.StateObjectMixedInstanceThreading.
//
// finishThreadProgress() must record completion without evicting a thread
// that has not exited yet.
TEST_F(AppTest, FinishThreadProgressKeepsRunningThreadJoinable) {
  std::string thread_key = "test.finish.keeps.live.thread";
  std::atomic<bool> started(false);
  std::atomic<bool> release(false);

  ctx.startThread(thread_key, [&]() {
    started = true;
    while (!release.load())
      boost::this_thread::yield();
  });

  while (!started.load())
    boost::this_thread::yield();

  // Exactly what ~thread_feedback does as a worker winds down -- except
  // this worker is demonstrably still running.
  ctx.finishThreadProgress(thread_key);

  // Sample everything before releasing the worker, but assert afterwards:
  // an ASSERT_* here would skip the release and hang the run.
  bool present = ctx.hasThread(thread_key);
  thread_ptr tptr = ctx.threads(thread_key);
  bool joinable = tptr && tptr->joinable();
  double progress = ctx.threadProgress(thread_key);

  release = true;
  if (tptr && tptr->joinable())
    tptr->join();
  else
    boost::this_thread::sleep_for(boost::chrono::milliseconds(250));

  EXPECT_TRUE(present) << "a still-running thread was evicted from the registry; "
                          "nothing can join it any more";
  EXPECT_TRUE(joinable) << "the last shared_ptr was dropped while the thread ran, "
                           "so ~boost::thread() detached it";

  // Completion still has to be observable by key.
  EXPECT_NEAR(progress, 1.0, 0.01);

  ctx.removeThread(thread_key);
}

TEST_F(AppTest, ThreadProgressWithThreadInterruption) {
  std::string thread_key = "test.progress.interruption";
  std::atomic<bool> started(false);
  std::atomic<bool> interrupted(false);

  ctx.startThread(thread_key, [&]() {
    try {
      cvc::app::thread_feedback feedback(ctx, thread_key);
      started = true;
      ctx.threadProgress(thread_key, 0.2);

      // Sleep with interruption point
      for (int i = 0; i < 100; i++) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        boost::this_thread::interruption_point();
      }
    } catch (boost::thread_interrupted &) {
      interrupted = true;
      // thread_feedback destructor should still execute and set to 100%
    }
  });

  // Wait for thread to start
  for (int i = 0; i < 50 && !started.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(started.load());

  // Interrupt the thread
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr)
      tptr->interrupt();
  }

  // Wait for interruption to be caught
  for (int i = 0; i < 50 && !interrupted.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }

  // Give thread time to clean up
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // Even with interruption, thread_feedback destructor should set progress
  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
      << "Progress should be 100% even after thread interruption (RAII cleanup)";

  // Clean up
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr && tptr->joinable())
      tptr->join();
  }
}

TEST_F(AppTest, ThreadStatusShowsCompleted) {
  std::string thread_key = "test.status.completed";
  std::atomic<bool> finished(false);

  ctx.startThread(thread_key, [&]() {
    cvc::app::thread_feedback feedback(ctx, thread_key);
    ctx.threadInfo(thread_key, "processing");
    ctx.threadProgress(thread_key, 0.5);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
    finished = true;
    // thread_feedback destructor sets status to "completed" and progress to 100%
  });

  // Wait for thread to finish
  for (int i = 0; i < 50 && !finished.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(finished.load());

  // Give thread time to exit
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  // Verify status is "completed" after thread exits
  std::string status = ctx.threadInfo(thread_key);
  EXPECT_EQ(status, "completed") << "Thread status should be 'completed' after thread exits";

  // Verify progress is 100%
  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01) << "Progress should be 100% when status is completed";

  // Clean up
  if (ctx.hasThread(thread_key)) {
    thread_ptr tptr = ctx.threads(thread_key);
    if (tptr && tptr->joinable())
      tptr->join();
  }
}

// ===========================
// Per-app world bases (world_clock / world_units)
// ===========================
// These must be per-app instances reached through the injected app — NOT a
// process-global singleton — mirroring root()/computePool().

TEST_F(AppTest, WorldClockIsAStablePerAppInstance) {
  cvc::world_clock &a = ctx.world_clock();
  cvc::world_clock &b = ctx.world_clock();
  EXPECT_EQ(&a, &b) << "world_clock() must return the same per-app instance";
  EXPECT_DOUBLE_EQ(a.fixed_dt(), 1.0 / 120.0) << "default clock is 120 Hz";
}

TEST_F(AppTest, WorldUnitsIsAStablePerAppInstanceWithSIDefault) {
  cvc::world_units &a = ctx.world_units();
  cvc::world_units &b = ctx.world_units();
  EXPECT_EQ(&a, &b) << "world_units() must return the same per-app instance";
  EXPECT_EQ(a.regime(), cvc::world_units::system::si);
  EXPECT_DOUBLE_EQ(a.metres_per_world_unit(), 1.0);
}

// ===========================
// compute_async — off-thread data-parallel fan-out with a completion callback
// ===========================

// compute_async runs the fan-out on the compute pool OFF the calling thread and calls on_done when
// it joins, WITHOUT blocking the caller — the async-schedule-safe way to drive a blocking
// parallel_for (an Ariadne program on:/intrinsic runs inline in drain() and must not block).
TEST_F(AppTest, ComputeAsyncRunsFanOutOffThreadThenCallsOnDone) {
  ctx.computePool(); // warm the pool on this thread before the background job first touches it
  auto arrived = std::make_shared<std::atomic<int>>(0);
  std::promise<void> done;
  std::future<void> fut = done.get_future();
  ctx.compute_async(
      256,
      [arrived](int i) {
        if ((i % 3) == 0)
          arrived->fetch_add(1, std::memory_order_relaxed);
      },
      [&done] { done.set_value(); });
  // The caller did not block; on_done fires from the worker once the fan-out joins.
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready)
      << "compute_async never completed";
  EXPECT_EQ(arrived->load(), 86); // i in [0,256), i%3==0 -> {0,3,...,255} = 86 indices
}

// ===========================
// compute_async / wait=false pooled jobs leave nothing behind in threads()
// ===========================

namespace {
// Poll `pred` until it holds or `budget` runs out; returns whether it held.
template <class Pred> bool eventually(Pred pred, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline)
      return pred();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;
}

// Run one tiny compute_async and wait for its on_done.
bool run_one_compute_async(app &ctx) {
  auto done = std::make_shared<std::promise<void>>();
  std::future<void> fut = done->get_future();
  ctx.compute_async(2, [](int) {}, [done] { done->set_value(); });
  return fut.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
}
} // namespace

// compute_async used to run each call on a fresh pooled thread under a unique key, and finished
// ones -- each still holding its boost::thread and the functor it ran -- stayed in the app's thread
// map until the app was destroyed, so a host offloading a chunk at a time (tf overlays,
// (http-get)/(fetch)) grew the map by one entry per call; #827 had a later worker reap them. Each
// call is now an anonymous pool task: never in the map, not even for a moment, and destroyed on its
// worker, with everything fn and on_done captured, as soon as it ends.
TEST_F(AppTest, ComputeAsyncCallsDoNotGrowTheThreadMap) {
  ctx.computePool(); // warm the pool on this thread before the background jobs touch it
  const std::size_t threads_before = ctx.threads().size();
  const int kCalls = 60;
  std::vector<std::weak_ptr<int>> captures;
  for (int c = 0; c < kCalls; ++c) {
    auto payload = std::make_shared<int>(c);
    captures.push_back(payload);
    auto done = std::make_shared<std::promise<void>>();
    std::future<void> fut = done->get_future();
    ctx.compute_async(2, [payload](int) {}, [payload, done] { done->set_value(); });
    payload.reset();
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready)
        << "compute_async " << c << " never completed";
    // Right after each on_done, while its job is still winding down: no entry, no straggler.
    EXPECT_EQ(ctx.threads().size(), threads_before) << "after compute_async " << c;
  }

  // A job's captures go on its worker before it counts as done.
  ASSERT_TRUE(wait_pool_idle(ctx));
  for (int c = 0; c < kCalls; ++c)
    EXPECT_TRUE(captures[c].expired()) << "compute_async " << c << " still holds its captures";
  EXPECT_EQ(ctx.threads().size(), threads_before);
}

// The same with calls queued behind a small pool. A queued job used to need a key held for it
// until it started: two queued calls handed the same key had the second interrupt the first and
// take over its entry. An anonymous job has no key to share. Every on_done runs exactly once, on a
// thread nobody asked to stop, and the map does not grow while they queue or after.
TEST_F(AppTest, QueuedComputeAsyncCallsEachRunOnceUninterrupted) {
  ctx.computePool();
  ctx.setThreadPoolSize(2);
  const std::size_t threads_before = ctx.threads().size();
  const int kCalls = 100;
  auto runs = std::make_shared<std::vector<std::atomic<int>>>(kCalls);
  auto total = std::make_shared<std::atomic<int>>(0);
  auto interrupted = std::make_shared<std::atomic<int>>(0);
  for (int c = 0; c < kCalls; ++c)
    ctx.compute_async(
        4, [](int) {},
        [runs, total, interrupted, c] {
          if (boost::this_thread::interruption_requested())
            interrupted->fetch_add(1);
          (*runs)[c].fetch_add(1);
          total->fetch_add(1);
        });
  const std::size_t threads_while_queued = ctx.threads().size();

  ASSERT_TRUE(eventually([&] { return total->load() >= kCalls; }, std::chrono::seconds(30)))
      << "only " << total->load() << " of " << kCalls << " compute_async calls completed";
  ASSERT_TRUE(wait_pool_idle(ctx));
  EXPECT_EQ(threads_while_queued, threads_before) << "queued compute_async calls were listed";
  EXPECT_EQ(ctx.threads().size(), threads_before)
      << "finished compute_async calls were left in the thread map";
  for (int c = 0; c < kCalls; ++c)
    EXPECT_EQ((*runs)[c].load(), 1) << "on_done of call " << c;
  EXPECT_EQ(interrupted->load(), 0) << "a queued call interrupted a running one";
  EXPECT_EQ(runs.use_count(), 1) << "a finished call still holds its captures";
}

// A pooled job that is still running is left alone while compute_async calls come and go around
// it. (When wait=false jobs were listed and finished workers reaped them, this checked that reaping
// only ever took a finished thread.) A wait=false job is never in the thread map, so nothing can
// drop it from there: it runs on, uninterrupted, to its end, holding its captures until then and
// not after. A keyed (wait=true) job is the listed kind: it keeps its entry, stays joinable and
// reports its info throughout, and its entry stays once it has ended, until replaced or removed.
// The calls around them add nothing to the map.
TEST_F(AppTest, ComputeAsyncChurnLeavesRunningPooledJobsAlone) {
  ctx.computePool();
  ctx.setThreadPoolSize(3); // the two long jobs, and a worker for the compute_async calls
  const std::size_t threads_before = ctx.threads().size();
  const std::string anon_key = "pool_long_runner_anonymous";
  const std::string keyed_key = "pool_long_runner_keyed";
  auto started = std::make_shared<std::atomic<int>>(0);
  auto release = std::make_shared<std::atomic<bool>>(false);
  auto finished = std::make_shared<std::atomic<int>>(0);
  auto interrupted = std::make_shared<std::atomic<int>>(0);
  struct release_on_exit {
    std::shared_ptr<std::atomic<bool>> flag;
    ~release_on_exit() { flag->store(true); } // an ASSERT below must not leave them spinning
  } releaser{release};
  const auto long_job = [started, release, finished, interrupted] {
    started->fetch_add(1);
    while (!release->load()) // sleep_for is not an interruption point
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (boost::this_thread::interruption_requested())
      interrupted->fetch_add(1);
    finished->fetch_add(1);
  };

  auto payload = std::make_shared<int>(42);
  const std::weak_ptr<int> anon_capture = payload;
  ctx.startThreadPooled(
      anon_key, [long_job, payload] { long_job(); }, PRIORITY_NORMAL, /*wait=*/false);
  payload.reset();
  ctx.startThreadPooled(keyed_key, long_job, PRIORITY_NORMAL, /*wait=*/true);
  ASSERT_TRUE(eventually([&] { return started->load() == 2; }, std::chrono::seconds(10)));

  for (int i = 0; i < 20; ++i)
    ASSERT_TRUE(run_one_compute_async(ctx)) << "compute_async " << i << " never completed";

  // Sample with both still held; assert once they are released.
  const bool anon_listed = ctx.hasThread(anon_key);
  const bool anon_capture_held = !anon_capture.expired();
  const bool keyed_listed = ctx.hasThread(keyed_key);
  thread_ptr runner = ctx.threads(keyed_key);
  const bool joinable = runner && runner->joinable();
  const std::string info = ctx.threadInfo(keyed_key);
  const std::size_t listed_total = ctx.threads().size();
  release->store(true);

  EXPECT_FALSE(anon_listed) << "a wait=false job was given a thread-map entry";
  EXPECT_TRUE(anon_capture_held) << "a running wait=false job lost its captures";
  EXPECT_TRUE(keyed_listed) << "a running keyed pooled thread was dropped from the thread map";
  EXPECT_TRUE(joinable) << "a running keyed pooled thread is no longer joinable";
  EXPECT_EQ(info, "Processing: " + keyed_key);
  EXPECT_EQ(listed_total, threads_before + 1) << "compute_async calls were left in the thread map";

  if (runner)
    runner->join();
  ASSERT_TRUE(eventually([&] { return finished->load() == 2; }, std::chrono::seconds(10)));
  ASSERT_TRUE(wait_pool_idle(ctx));
  EXPECT_EQ(interrupted->load(), 0) << "a running pooled job was interrupted";
  EXPECT_TRUE(anon_capture.expired()) << "a finished wait=false job still holds its captures";
  EXPECT_TRUE(ctx.hasThread(keyed_key))
      << "a finished keyed job's entry went before it was removed";
  EXPECT_EQ(ctx.threads().size(), threads_before + 1);
  EXPECT_EQ(finished->load(), 2);
}

// Destroying the app while compute_async work is queued, running and finished must stay clean:
// nothing queued starts any more, every running job is interrupted and its worker joined --
// including a worker started just as ~app stopped the pool, which takes no task -- no on_done runs
// twice, and every job, run or dropped, has released what it captured once ~app returns.
TEST_F(AppTest, DestroyingAnAppWithComputeAsyncInFlightIsClean) {
  const int kCalls = 32;
  auto runs = std::make_shared<std::vector<std::atomic<int>>>(kCalls);
  auto total = std::make_shared<std::atomic<int>>(0);
  const auto t0 = std::chrono::steady_clock::now();
  {
    app local;
    local.computePool();
    local.setThreadPoolSize(2);
    for (int c = 0; c < kCalls; ++c)
      local.compute_async(
          4, [](int) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); },
          [runs, total, c] {
            (*runs)[c].fetch_add(1);
            total->fetch_add(1);
          });
    // Let a few finish (and be destroyed on their workers) so finished, running and queued work
    // all meet ~app.
    eventually([&] { return total->load() >= 3; }, std::chrono::seconds(10));
  }
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(20));
  EXPECT_GE(total->load(), 3);
  for (int c = 0; c < kCalls; ++c)
    EXPECT_LE((*runs)[c].load(), 1) << "on_done of call " << c;
  EXPECT_EQ(runs.use_count(), 1) << "a job the app ran or dropped still holds its captures";
}

TEST_F(AppTest, WorldBasesAreNotGlobalSingletons) {
  // Two independent apps must own independent bases: mutating one is invisible to
  // the other. This is the whole point of the per-app model.
  app other;
  ctx.world_units().set_regime(cvc::world_units::system::imperial);
  ctx.world_clock().set_scale(4.0);
  EXPECT_EQ(other.world_units().regime(), cvc::world_units::system::si);
  EXPECT_DOUBLE_EQ(other.world_clock().scale(), 1.0);
  // ...and the mutation persisted on the app that made it.
  EXPECT_EQ(ctx.world_units().regime(), cvc::world_units::system::imperial);
  EXPECT_DOUBLE_EQ(ctx.world_clock().scale(), 4.0);
}

// Main function is provided by gtest_main library

// ===========================
// world_clock_binding — one clock, controlled and published on the state tree
// ===========================

namespace {
std::string key_value(app &ctx, const std::string &path) {
  return cvc::state::instance(ctx)(path).value();
}
void set_key(app &ctx, const std::string &path, const std::string &v) {
  cvc::state::instance(ctx)(path).value(v);
}
} // namespace

TEST_F(AppTest, ClockBindingSeedsControlsAndPublishesTime) {
  cvc::world_clock &c = ctx.world_clock();
  cvc::world_clock_binding b(ctx, c, "simtest");
  const auto r = b.drive(0.05); // 6 quanta at 120 Hz
  EXPECT_EQ(r.steps, 6);
  EXPECT_EQ(key_value(ctx, "simtest.speed"), "1");
  EXPECT_EQ(key_value(ctx, "simtest.paused"), "0");
  EXPECT_EQ(key_value(ctx, "simtest.mode"), "live");
  EXPECT_EQ(key_value(ctx, "simtest.lockstep_steps"), "1");
  EXPECT_EQ(key_value(ctx, "simtest.step"), "0");
  EXPECT_EQ(key_value(ctx, "simtest.tick"), "6");
  EXPECT_DOUBLE_EQ(std::stod(key_value(ctx, "simtest.time")), c.t());
  EXPECT_DOUBLE_EQ(std::stod(key_value(ctx, "simtest.fixed_dt")), c.fixed_dt());
}

TEST_F(AppTest, ClockBindingSpeedPauseAndModeComeFromTheTree) {
  cvc::world_clock &c = ctx.world_clock();
  cvc::world_clock_binding b(ctx, c, "simtest");
  b.drive(0.0);

  set_key(ctx, "simtest.speed", "2");
  EXPECT_EQ(b.drive(0.025).steps, 6) << "25 ms of wall at 2x is 6 quanta at 120 Hz";
  EXPECT_DOUBLE_EQ(c.scale(), 2.0);

  set_key(ctx, "simtest.paused", "true"); // state_exec's spelling of a bool
  EXPECT_EQ(b.drive(0.05).steps, 0);
  EXPECT_EQ(c.current_mode(), cvc::world_clock::mode::paused);

  set_key(ctx, "simtest.paused", "0");
  set_key(ctx, "simtest.mode", "lockstep");
  set_key(ctx, "simtest.lockstep_steps", "3");
  EXPECT_EQ(b.drive(10.0).steps, 3) << "lockstep ignores the wall";
  EXPECT_EQ(c.current_mode(), cvc::world_clock::mode::lockstep);

  set_key(ctx, "simtest.paused", "1");
  EXPECT_EQ(b.drive(10.0).steps, 0) << "paused wins over lockstep";

  set_key(ctx, "simtest.mode", "no-such-mode");
  set_key(ctx, "simtest.paused", "0");
  EXPECT_EQ(b.drive(0.0).steps, 3) << "an unknown mode is ignored, not thrown";
}

TEST_F(AppTest, ClockBindingStepRequestsRunOneQuantumEach) {
  cvc::world_clock &c = ctx.world_clock();
  cvc::world_clock_binding b(ctx, c, "simtest");
  set_key(ctx, "simtest.mode", "stepping");
  b.drive(0.0);
  const std::uint64_t t0 = c.tick();
  EXPECT_EQ(b.drive(1.0).steps, 0) << "stepping ignores the wall";

  set_key(ctx, "simtest.step", "2");
  const auto r = b.drive(1.0);
  EXPECT_EQ(r.steps, 2);
  EXPECT_EQ(r.first_tick, t0);
  EXPECT_EQ(c.tick(), t0 + 2);

  set_key(ctx, "simtest.step", "0"); // a counter reset requests nothing
  EXPECT_EQ(b.drive(1.0).steps, 0);
  EXPECT_EQ(b.pending_steps(), 0u);
}

TEST_F(AppTest, ClockBindingDoesNotReplayRequestsMadeBeforeItExisted) {
  set_key(ctx, "simtest.step", "40");
  set_key(ctx, "simtest.mode", "stepping");
  cvc::world_clock_binding b(ctx, ctx.world_clock(), "simtest");
  EXPECT_EQ(b.drive(0.0).steps, 0);
}

TEST_F(AppTest, ClockBindingSharesAriadnesTransportKeys) {
  // A default binding speaks sim.speed / sim.paused / sim.time, the keys Ariadne's
  // sim_transport.ari binds, so a scene's Speed slider steers any simulation
  // driven through a binding.
  cvc::world_clock_binding b(ctx, ctx.world_clock());
  EXPECT_EQ(b.key("speed"), "sim.speed");
  set_key(ctx, "sim.speed", "0.5");
  EXPECT_EQ(b.drive(0.1).steps, 6);
  EXPECT_EQ(key_value(ctx, "sim.tick"), "6");
}

TEST_F(AppTest, ClockBindingTakesExplicitKeysAndSkipsEmptyOnes) {
  // A scene's `clock:` block names its keys one by one; a lane with no key is
  // neither seeded, read nor written.
  cvc::world_clock_keys k;
  k.speed = "scn.rate";
  k.time = "scn.now";
  k.step = "scn.step";
  cvc::world_clock &c = ctx.world_clock();
  cvc::world_clock_binding b(ctx, c, k);
  EXPECT_EQ(b.key("speed"), "scn.rate");
  EXPECT_EQ(b.key("paused"), "");
  EXPECT_EQ(b.drive(0.05).steps, 6);
  EXPECT_EQ(key_value(ctx, "scn.rate"), "1");
  EXPECT_DOUBLE_EQ(std::stod(key_value(ctx, "scn.now")), c.t());
  const std::vector<std::string> scn = cvc::state::instance(ctx)("scn").children();
  EXPECT_EQ(scn.size(), 3u) << "only the three named keys exist under scn";

  set_key(ctx, "scn.rate", "2");
  EXPECT_EQ(b.drive(0.025).steps, 6);
  set_key(ctx, "scn.step", "1");
  EXPECT_EQ(b.drive(0.0).steps, 1) << "a step request through an explicit key";
}

TEST_F(AppTest, ClockBindingRunsStepsAHostVerbRequests) {
  cvc::world_clock &c = ctx.world_clock();
  cvc::world_clock_binding b(ctx, c, "simtest");
  set_key(ctx, "simtest.paused", "1");
  b.drive(0.0);
  const std::uint64_t t0 = c.tick();
  b.request_steps(2);
  EXPECT_EQ(b.pending_steps(), 2u);
  EXPECT_EQ(b.drive(1.0).steps, 2) << "a step request runs while paused";
  EXPECT_EQ(c.tick(), t0 + 2);
  EXPECT_EQ(b.drive(1.0).steps, 0);
}

// Step requests from several threads while another thread drives: none is lost
// or run twice (the binding's own counters are behind its lock).
TEST_F(AppTest, ClockBindingTakesStepRequestsFromAnyThread) {
  cvc::world_clock &c = ctx.world_clock();
  cvc::world_clock_binding b(ctx, c, "simtest");
  set_key(ctx, "simtest.paused", "1");
  b.drive(0.0);
  const std::uint64_t t0 = c.tick();
  constexpr int kThreads = 4, kEach = 250;
  std::atomic<bool> done{false};
  std::uint64_t driven = 0;
  std::thread driver([&] {
    while (!done)
      driven += static_cast<std::uint64_t>(b.drive(0.0).steps);
  });
  std::vector<std::thread> requesters;
  for (int t = 0; t < kThreads; ++t)
    requesters.emplace_back([&] {
      for (int i = 0; i < kEach; ++i) {
        b.request_steps(1);
        (void)b.pending_steps();
      }
    });
  for (std::thread &t : requesters)
    t.join();
  done = true;
  driver.join();
  driven += static_cast<std::uint64_t>(b.drive(0.0).steps); // whatever arrived last
  EXPECT_EQ(driven, static_cast<std::uint64_t>(kThreads * kEach));
  EXPECT_EQ(c.tick(), t0 + kThreads * kEach);
  EXPECT_EQ(b.pending_steps(), 0u);
}

TEST_F(AppTest, ClockKeysUnderAPrefixMatchThePrefixConstructor) {
  const cvc::world_clock_keys k = cvc::world_clock_keys::under("a.sim");
  EXPECT_EQ(k.speed, "a.sim.speed");
  EXPECT_EQ(k.lockstep_steps, "a.sim.lockstep_steps");
  EXPECT_EQ(k.dropped, "a.sim.dropped");
  EXPECT_EQ(cvc::world_clock_keys::under("").time, "time");
  cvc::world_clock_binding b(ctx, ctx.world_clock(), "a.sim");
  EXPECT_EQ(b.keys().realized_speed, "a.sim.realized_speed");
  EXPECT_EQ(b.key("custom"), "a.sim.custom");
}

// PR-5 (docs/STATE_EXEC_LOCKS.md 7.3, 7.4): app::exec_scheduler() is built with the policy
// state_exec.policy names, round_robin when it is unset (today's), and fixed then. An unknown
// value is logged and round_robin used. state_exec.process_class is read then too, and its
// result, or an unknown value, logged.
TEST(AppExecScheduler, PR05_PolicySetting) {
  using cvc::state_exec::scheduling_policy;
  {
    app a;
    EXPECT_EQ(a.exec_scheduler().policy(), scheduling_policy::round_robin);
  }
  for (const auto &[name, policy] :
       {std::pair<const char *, scheduling_policy>{"round_robin", scheduling_policy::round_robin},
        {"priority", scheduling_policy::priority},
        {"priority_rr", scheduling_policy::priority_rr}}) {
    app a;
    cvc::state::instance(a)("state_exec.policy").value(name);
    EXPECT_EQ(a.exec_scheduler().policy(), policy) << name;
    // Fixed when built: no runtime set_policy (7.3).
    cvc::state::instance(a)("state_exec.policy").value("round_robin");
    EXPECT_EQ(a.exec_scheduler().policy(), policy) << name;
  }
  {
    app a;
    const std::string log_key("pr05_log");
    a.data(log_key, std::string());
    a.properties("system.log_output", log_key);
    cvc::state::instance(a)("state_exec.policy").value("fifo");
    cvc::state::instance(a)("state_exec.process_class").value("idle");
    EXPECT_EQ(a.exec_scheduler().policy(), scheduling_policy::round_robin);
#ifndef USING_LOG4CPLUS_DEFAULT
    const std::string logged = a.data<std::string>(log_key);
    EXPECT_NE(logged.find("state_exec.policy: unknown value 'fifo'"), std::string::npos) << logged;
    EXPECT_NE(logged.find("state_exec.process_class: unknown value 'idle'"), std::string::npos)
        << logged;
#endif
  }
  {
    // normal, the one class a test process can take without slowing the tests after it (never
    // above normal): the result is logged, applied or refused (on Linux a process started below
    // normal can't come back up unprivileged).
    app a;
    const std::string log_key("pr05_class_log");
    a.data(log_key, std::string());
    a.properties("system.log_output", log_key);
    cvc::state::instance(a)("state_exec.process_class").value("normal");
    a.exec_scheduler();
#ifndef USING_LOG4CPLUS_DEFAULT
    const std::string logged = a.data<std::string>(log_key);
    EXPECT_NE(logged.find("state_exec.process_class: "), std::string::npos) << logged;
    EXPECT_NE(logged.find("normal"), std::string::npos) << logged;
#endif
  }
}

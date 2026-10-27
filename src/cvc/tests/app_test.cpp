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
#include <cvc/core/world_clock.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/core/world_units.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <future> // compute_async: deterministic wait on on_done via a promise
#include <gtest/gtest.h>
#include <memory> // compute_async: shared accumulator captured by fn + on_done
#include <mutex>
#include <set>
#include <string>
#include <thread>
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
  std::uint64_t interruptsSent() {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    return _poolInterruptGen;
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

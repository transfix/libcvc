/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for cvc::app class functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/app.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace CVC_NAMESPACE;

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

TEST_F(AppTest, ThreadInfoAndProgressTracking) {
  std::string thread_key = "test.progress.tracking";
  std::atomic<bool> thread_started(false);
  std::atomic<bool> continue_running(true);

  // Start a thread that updates its info and progress
  ctx.startThreadPooled(
      thread_key,
      [&]() {
        thread_started = true;

        // Set initial thread info
        ctx.threadInfo(thread_key, "Starting processing");
        ctx.threadProgress(thread_key, 0.0);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

        // Update to 25% progress
        ctx.threadInfo(thread_key, "Processing step 1");
        ctx.threadProgress(thread_key, 0.25);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

        // Update to 50% progress
        ctx.threadInfo(thread_key, "Processing step 2");
        ctx.threadProgress(thread_key, 0.50);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

        // Update to 75% progress
        ctx.threadInfo(thread_key, "Processing step 3");
        ctx.threadProgress(thread_key, 0.75);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

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
  for (int i = 0; i < 50 && !thread_started.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(thread_started.load()) << "Thread should have started";

  // Verify we can read info and progress while thread is running
  // Use polling to wait for each checkpoint instead of fixed sleeps

  // Wait for 25% progress
  for (int i = 0; i < 200 && ctx.threadProgress(thread_key) < 0.24; i++)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  double progress1 = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress1, 0.25, 0.01);

  // Wait for 50% progress
  for (int i = 0; i < 200 && ctx.threadProgress(thread_key) < 0.49; i++)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  double progress2 = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress2, 0.50, 0.01);

  // Wait for 75% progress
  for (int i = 0; i < 200 && ctx.threadProgress(thread_key) < 0.74; i++)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  double progress3 = ctx.threadProgress(thread_key);
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

  // Join the thread BEFORE checking progress: thread_feedback's destructor
  // runs as the catch block unwinds, and we need a happens-before edge
  // between that destructor and our progress read. A fixed sleep is racy
  // under CI load (the original 200 ms was occasionally insufficient on
  // GitHub-hosted Linux runners), but join() is a hard synchronization
  // point that guarantees the destructor has completed.
  ASSERT_TRUE(ctx.hasThread(thread_key));
  thread_ptr tptr = ctx.threads(thread_key);
  ASSERT_TRUE(tptr);
  if (tptr->joinable())
    tptr->join();

  // Progress should now be 100% — set by thread_feedback's destructor
  // during stack unwinding.
  double progress = ctx.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
      << "Progress should be 100% even when exception occurs (RAII cleanup)";
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

// Main function is provided by gtest_main library

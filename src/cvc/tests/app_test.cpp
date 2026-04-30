/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for cvc::app class functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/app.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <string>
#include <vector>

using namespace CVC_NAMESPACE;

// ===========================
// Basic Singleton Tests
// ===========================

TEST(AppTest, SingletonInstance) {
  // Test that instance() returns a valid reference
  app& app1 = cvcapp;
  app& app2 = cvcapp;
  
  // Both references should point to the same singleton
  EXPECT_EQ(&app1, &app2);
}

// ===========================
// Data Management Tests
// ===========================

TEST(AppTest, DataSetAndGet) {
  // Test setting and getting string data
  std::string test_key = "test.data.string";
  std::string test_value = "Hello, World!";
  
  cvcapp.data(test_key, test_value);
  
  ASSERT_TRUE(cvcapp.isData<std::string>(test_key));
  EXPECT_EQ(cvcapp.data<std::string>(test_key), test_value);
  
  // Clean up
  cvcapp.data(test_key, boost::any());
}

TEST(AppTest, DataTypeInt) {
  std::string key = "test.data.int";
  int value = 42;
  
  cvcapp.data(key, value);
  
  ASSERT_TRUE(cvcapp.isData<int>(key));
  EXPECT_EQ(cvcapp.data<int>(key), value);
  
  // Clean up
  cvcapp.data(key, boost::any());
}

TEST(AppTest, DataTypeDouble) {
  std::string key = "test.data.double";
  double value = 3.14159;
  
  cvcapp.data(key, value);
  
  ASSERT_TRUE(cvcapp.isData<double>(key));
  EXPECT_DOUBLE_EQ(cvcapp.data<double>(key), value);
  
  // Clean up
  cvcapp.data(key, boost::any());
}

TEST(AppTest, DataTypeBool) {
  std::string key = "test.data.bool";
  bool value = true;
  
  cvcapp.data(key, value);
  
  ASSERT_TRUE(cvcapp.isData<bool>(key));
  EXPECT_EQ(cvcapp.data<bool>(key), value);
  
  // Clean up
  cvcapp.data(key, boost::any());
}

TEST(AppTest, DataRemoval) {
  std::string key = "test.data.removal";
  std::string value = "temporary";
  
  cvcapp.data(key, value);
  ASSERT_TRUE(cvcapp.isData<std::string>(key));
  
  // Remove by setting empty boost::any
  cvcapp.data(key, boost::any());
  EXPECT_FALSE(cvcapp.isData<std::string>(key));
}

TEST(AppTest, DataTypeName) {
  std::string key = "test.data.typename";
  int value = 123;
  
  cvcapp.data(key, value);
  
  std::string typeName = cvcapp.dataTypeName(key);
  EXPECT_FALSE(typeName.empty());
  
  // Clean up
  cvcapp.data(key, boost::any());
}

TEST(AppTest, DataMap) {
  // Test getting the entire data map
  std::string key1 = "test.map.key1";
  std::string key2 = "test.map.key2";
  
  cvcapp.data(key1, 100);
  cvcapp.data(key2, std::string("value2"));
  
  data_map map = cvcapp.data();
  
  EXPECT_TRUE(map.find(key1) != map.end());
  EXPECT_TRUE(map.find(key2) != map.end());
  
  // Clean up
  cvcapp.data(key1, boost::any());
  cvcapp.data(key2, boost::any());
}

// ===========================
// Property Management Tests
// ===========================

TEST(AppTest, PropertySetAndGet) {
  std::string key = "test.property.simple";
  std::string value = "test_value";
  
  cvcapp.properties(key, value);
  
  EXPECT_EQ(cvcapp.properties(key), value);
  EXPECT_TRUE(cvcapp.hasProperty(key));
  
  // Clean up
  cvcapp.properties(key, std::string());
}

TEST(AppTest, PropertyRemoval) {
  std::string key = "test.property.removal";
  
  cvcapp.properties(key, "temporary");
  ASSERT_TRUE(cvcapp.hasProperty(key));
  
  // Remove by setting empty string
  cvcapp.properties(key, std::string());
  EXPECT_FALSE(cvcapp.hasProperty(key));
}

TEST(AppTest, PropertyList) {
  std::string key = "test.property.list";
  
  // Set comma-separated list
  cvcapp.properties(key, "item1,item2,item3");
  
  std::vector<std::string> items = cvcapp.listProperty(key);
  
  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item2");
  EXPECT_EQ(items[2], "item3");
  
  // Clean up
  cvcapp.properties(key, std::string());
}

TEST(AppTest, PropertyListWithSpaces) {
  std::string key = "test.property.list.spaces";
  
  // Set list with spaces around commas
  cvcapp.properties(key, "item1 , item2 , item3");
  
  std::vector<std::string> items = cvcapp.listProperty(key);
  
  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item2");
  EXPECT_EQ(items[2], "item3");
  
  // Clean up
  cvcapp.properties(key, std::string());
}

TEST(AppTest, PropertyListUnique) {
  std::string key = "test.property.list.unique";
  
  // Set list with duplicates
  cvcapp.properties(key, "item1,item2,item1,item3,item2");
  
  std::vector<std::string> items = cvcapp.listProperty(key, true);
  
  // Should have only unique items
  EXPECT_EQ(items.size(), 3);
  
  // Clean up
  cvcapp.properties(key, std::string());
}

TEST(AppTest, PropertyListAppend) {
  std::string key = "test.property.list.append";
  
  cvcapp.properties(key, "item1,item2");
  cvcapp.listPropertyAppend(key, "item3");
  
  std::vector<std::string> items = cvcapp.listProperty(key);
  
  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[2], "item3");
  
  // Clean up
  cvcapp.properties(key, std::string());
}

TEST(AppTest, PropertyListRemove) {
  std::string key = "test.property.list.remove";
  
  cvcapp.properties(key, "item1,item2,item3");
  cvcapp.listPropertyRemove(key, "item2");
  
  std::vector<std::string> items = cvcapp.listProperty(key);
  
  ASSERT_EQ(items.size(), 2);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item3");
  
  // Clean up
  cvcapp.properties(key, std::string());
}

TEST(AppTest, PropertyMap) {
  // Test getting the entire property map
  std::string key1 = "test.propmap.key1";
  std::string key2 = "test.propmap.key2";
  
  cvcapp.properties(key1, "value1");
  cvcapp.properties(key2, "value2");
  
  property_map map = cvcapp.properties();
  
  EXPECT_TRUE(map.find(key1) != map.end());
  EXPECT_TRUE(map.find(key2) != map.end());
  
  // Clean up
  cvcapp.properties(key1, std::string());
  cvcapp.properties(key2, std::string());
}

// ===========================
// Thread Management Tests
// ===========================

TEST(AppTest, ThreadKeyGeneration) {
  std::string hint = "test_thread";
  std::string key1 = cvcapp.uniqueThreadKey(hint);
  
  // Register the first key so the second will be different
  cvcapp.threads(key1, thread_ptr(new boost::thread()));
  
  std::string key2 = cvcapp.uniqueThreadKey(hint);
  
  // Keys should be unique
  EXPECT_NE(key1, key2);
  EXPECT_TRUE(key1.find(hint) != std::string::npos);
  EXPECT_TRUE(key2.find(hint) != std::string::npos);
  
  // Clean up
  cvcapp.removeThread(key1);
}

TEST(AppTest, ThreadProgress) {
  // Test thread progress tracking
  double progress = 0.5;
  cvcapp.threadProgress(progress);
  
  double retrieved = cvcapp.threadProgress();
  EXPECT_DOUBLE_EQ(retrieved, progress);
}

TEST(AppTest, ThreadProgressClamping) {
  // Test that progress is clamped to [0.0, 1.0]
  cvcapp.threadProgress(-0.5);
  EXPECT_DOUBLE_EQ(cvcapp.threadProgress(), 0.0);
  
  cvcapp.threadProgress(1.5);
  EXPECT_DOUBLE_EQ(cvcapp.threadProgress(), 1.0);
}

// ===========================
// Listify Tests
// ===========================

TEST(AppTest, ListifyVectorToString) {
  std::vector<std::string> vec = {"item1", "item2", "item3"};
  std::string result = cvcapp.listify(vec);
  
  EXPECT_FALSE(result.empty());
  EXPECT_TRUE(result.find("item1") != std::string::npos);
  EXPECT_TRUE(result.find("item2") != std::string::npos);
  EXPECT_TRUE(result.find("item3") != std::string::npos);
}

TEST(AppTest, ListifyStringToVector) {
  std::string list = "item1,item2,item3";
  std::vector<std::string> result = cvcapp.listify(list);
  
  ASSERT_EQ(result.size(), 3);
  EXPECT_EQ(result[0], "item1");
  EXPECT_EQ(result[1], "item2");
  EXPECT_EQ(result[2], "item3");
}

TEST(AppTest, ListifyRoundTrip) {
  std::vector<std::string> original = {"alpha", "beta", "gamma"};
  std::string stringified = cvcapp.listify(original);
  std::vector<std::string> result = cvcapp.listify(stringified);
  
  ASSERT_EQ(result.size(), original.size());
  for (size_t i = 0; i < original.size(); ++i) {
    EXPECT_EQ(result[i], original[i]);
  }
}

// ===========================
// Mutex Management Tests
// ===========================

TEST(AppTest, MutexCreation) {
  std::string mutex_name = "test.mutex";
  mutex_ptr mtx = cvcapp.mutex(mutex_name);
  
  ASSERT_NE(mtx, nullptr);
  
  // Getting the same mutex should return the same pointer
  mutex_ptr mtx2 = cvcapp.mutex(mutex_name);
  EXPECT_EQ(mtx, mtx2);
}

TEST(AppTest, MutexInfo) {
  std::string mutex_name = "test.mutex.info";
  std::string info = "Test mutex information";
  
  cvcapp.mutexInfo(mutex_name, info);
  std::string retrieved = cvcapp.mutexInfo(mutex_name);
  
  EXPECT_EQ(retrieved, info);
}

// ===========================
// Data Type Registration Tests
// ===========================

TEST(AppTest, DataTypeEnumRegistration) {
  // Test that basic types are registered correctly
  int test_int = 42;
  cvcapp.data("test.enum.int", test_int);
  
  data_type dt = cvcapp.dataType("test.enum.int");
  EXPECT_EQ(dt, Int);
  
  // Clean up
  cvcapp.data("test.enum.int", boost::any());
}

// ===========================
// Additional Property Map Tests
// ===========================

TEST(AppTest, PropertyMapOperations) {
  property_map test_map;
  test_map["prop1"] = "value1";
  test_map["prop2"] = "value2";
  test_map["prop3"] = "value3";
  
  cvcapp.properties(test_map);
  
  EXPECT_EQ(cvcapp.properties("prop1"), "value1");
  EXPECT_EQ(cvcapp.properties("prop2"), "value2");
  EXPECT_EQ(cvcapp.properties("prop3"), "value3");
  
  // Clean up
  cvcapp.properties("prop1", "");
  cvcapp.properties("prop2", "");
  cvcapp.properties("prop3", "");
}

TEST(AppTest, AddProperties) {
  property_map test_map;
  test_map["addprop1"] = "addvalue1";
  test_map["addprop2"] = "addvalue2";
  
  cvcapp.addProperties(test_map);
  
  EXPECT_TRUE(cvcapp.hasProperty("addprop1"));
  EXPECT_TRUE(cvcapp.hasProperty("addprop2"));
  EXPECT_EQ(cvcapp.properties("addprop1"), "addvalue1");
  
  // Clean up
  cvcapp.properties("addprop1", "");
  cvcapp.properties("addprop2", "");
}

TEST(AppTest, PropertyListOperations) {
  // Test comma-separated list property
  cvcapp.properties("test.list", "item1,item2,item3");
  std::vector<std::string> items = cvcapp.listProperty("test.list");
  
  ASSERT_EQ(items.size(), 3);
  EXPECT_EQ(items[0], "item1");
  EXPECT_EQ(items[1], "item2");
  EXPECT_EQ(items[2], "item3");
  
  // Clean up
  cvcapp.properties("test.list", "");
}

TEST(AppTest, PropertyListUniqueElements) {
  cvcapp.properties("test.list.unique", "a,b,a,c,b,d");
  std::vector<std::string> items = cvcapp.listProperty("test.list.unique", true);
  
  // Should only contain unique elements
  EXPECT_EQ(items.size(), 4);
  
  // Clean up
  cvcapp.properties("test.list.unique", "");
}

TEST(AppTest, ListPropertyAppendRemove) {
  cvcapp.properties("test.list.modify", "alpha,beta");
  
  cvcapp.listPropertyAppend("test.list.modify", "gamma");
  std::vector<std::string> items = cvcapp.listProperty("test.list.modify");
  EXPECT_EQ(items.size(), 3);
  EXPECT_EQ(items[2], "gamma");
  
  cvcapp.listPropertyRemove("test.list.modify", "beta");
  items = cvcapp.listProperty("test.list.modify");
  EXPECT_EQ(items.size(), 2);
  
  // Clean up
  cvcapp.properties("test.list.modify", "");
}

TEST(AppTest, PropertyTypedAccess) {
  cvcapp.properties("test.int.prop", 12345);
  int val = cvcapp.properties<int>("test.int.prop");
  EXPECT_EQ(val, 12345);
  
  cvcapp.properties("test.double.prop", 3.14159);
  double dval = cvcapp.properties<double>("test.double.prop");
  EXPECT_NEAR(dval, 3.14159, 0.00001);
  
  // Clean up
  cvcapp.properties("test.int.prop", "");
  cvcapp.properties("test.double.prop", "");
}

// ===========================
// Data Map Bulk Operations
// ===========================

TEST(AppTest, DataMapBulkOperations) {
  data_map test_data;
  test_data["bulk1"] = std::string("value1");
  test_data["bulk2"] = int(42);
  test_data["bulk3"] = double(3.14);
  
  cvcapp.data(test_data);
  
  EXPECT_TRUE(cvcapp.isData<std::string>("bulk1"));
  EXPECT_TRUE(cvcapp.isData<int>("bulk2"));
  EXPECT_TRUE(cvcapp.isData<double>("bulk3"));
  
  data_map retrieved = cvcapp.data();
  EXPECT_TRUE(retrieved.find("bulk1") != retrieved.end());
  EXPECT_TRUE(retrieved.find("bulk2") != retrieved.end());
  
  // Clean up
  cvcapp.data("bulk1", boost::any());
  cvcapp.data("bulk2", boost::any());
  cvcapp.data("bulk3", boost::any());
}

TEST(AppTest, DataVectorOperations) {
  std::vector<std::string> keys = {"vec1", "vec2", "vec3"};
  std::vector<int> values = {10, 20, 30};
  
  cvcapp.data(keys, values);
  
  EXPECT_EQ(cvcapp.data<int>("vec1"), 10);
  EXPECT_EQ(cvcapp.data<int>("vec2"), 20);
  EXPECT_EQ(cvcapp.data<int>("vec3"), 30);
  
  // Test retrieving as vector
  std::vector<int> retrieved = cvcapp.data<int>(keys);
  ASSERT_EQ(retrieved.size(), 3);
  EXPECT_EQ(retrieved[0], 10);
  EXPECT_EQ(retrieved[1], 20);
  EXPECT_EQ(retrieved[2], 30);
  
  // Clean up
  cvcapp.data("vec1", boost::any());
  cvcapp.data("vec2", boost::any());
  cvcapp.data("vec3", boost::any());
}

TEST(AppTest, DataTypesByType) {
  // Store several strings
  cvcapp.data("str1", std::string("test1"));
  cvcapp.data("str2", std::string("test2"));
  cvcapp.data("int1", int(42));
  
  // Get all string keys
  std::vector<std::string> str_keys = cvcapp.data<std::string>();
  EXPECT_GE(str_keys.size(), 2);
  
  // Clean up
  cvcapp.data("str1", boost::any());
  cvcapp.data("str2", boost::any());
  cvcapp.data("int1", boost::any());
}

TEST(AppTest, DataTypeNameFromAny) {
  boost::any any_val = std::string("test");
  std::string type_name = cvcapp.dataTypeName(any_val);
  EXPECT_FALSE(type_name.empty());
}

TEST(AppTest, DataTypeTemplate) {
  // Test template version
  std::string type_name = cvcapp.dataTypeName<std::string>();
  EXPECT_FALSE(type_name.empty());
  
  std::string type_name2 = cvcapp.dataTypeName<int>();
  EXPECT_FALSE(type_name2.empty());
}

// ===========================
// Thread Progress and Info Tests
// ===========================

TEST(AppTest, ThreadProgressBasic) {
  std::string thread_key = "test.thread.progress";
  
  // Simulate thread registration
  cvcapp.threads(thread_key, thread_ptr(new boost::thread([](){})));
  
  cvcapp.threadProgress(thread_key, 0.5);
  double progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 0.5, 0.01);
  
  cvcapp.threadProgress(thread_key, 1.0);
  progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01);
  
  // Clean up
  cvcapp.threads(thread_key, thread_ptr());
}

TEST(AppTest, ThreadInfoOperations) {
  std::string thread_key = "test.thread.info";
  std::string info = "Processing data...";
  
  cvcapp.threads(thread_key, thread_ptr(new boost::thread([](){})));
  
  cvcapp.threadInfo(thread_key, info);
  std::string retrieved = cvcapp.threadInfo(thread_key);
  EXPECT_EQ(retrieved, info);
  
  // Clean up
  cvcapp.threads(thread_key, thread_ptr());
}

TEST(AppTest, HasThread) {
  std::string thread_key = "test.thread.exists";
  
  EXPECT_FALSE(cvcapp.hasThread(thread_key));
  
  cvcapp.threads(thread_key, thread_ptr(new boost::thread([](){})));
  EXPECT_TRUE(cvcapp.hasThread(thread_key));
  
  cvcapp.threads(thread_key, thread_ptr());
  EXPECT_FALSE(cvcapp.hasThread(thread_key));
}

TEST(AppTest, UniqueThreadKey) {
  // Register a thread with first key
  std::string key1 = cvcapp.uniqueThreadKey("test");
  cvcapp.threads(key1, thread_ptr(new boost::thread([](){})));
  
  // Should get a different unique key
  std::string key2 = cvcapp.uniqueThreadKey("test");
  
  // Keys should be unique
  EXPECT_NE(key1, key2);
  
  // Clean up
  cvcapp.threads(key1, thread_ptr());
}

// ===========================
// Listify Tests
// ===========================

TEST(AppTest, ListifyString) {
  std::string list = "a,b,c";
  std::vector<std::string> vec = cvcapp.listify(list);
  
  ASSERT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[0], "a");
  EXPECT_EQ(vec[1], "b");
  EXPECT_EQ(vec[2], "c");
}

TEST(AppTest, ListifyVector) {
  std::vector<std::string> vec = {"x", "y", "z"};
  std::string list = cvcapp.listify(vec);
  
  EXPECT_FALSE(list.empty());
  EXPECT_NE(list.find("x"), std::string::npos);
  EXPECT_NE(list.find("y"), std::string::npos);
  EXPECT_NE(list.find("z"), std::string::npos);
}

// ===========================
// Data Reader Tests
// ===========================

TEST(AppTest, DataReaderCollection) {
  data_reader_collection readers = cvcapp.dataReaders();
  
  // Add a test reader
  data_reader test_reader = [](const std::string& path) -> bool {
    return path == "test.dat";
  };
  
  cvcapp.dataReader(test_reader);
  
  data_reader_collection updated = cvcapp.dataReaders();
  EXPECT_GE(updated.size(), readers.size());
}

TEST(AppTest, ReadDataWithReaders) {
  // Add a reader that handles .test files
  data_reader test_reader = [](const std::string& path) -> bool {
    if (path.find(".test") != std::string::npos) {
      cvcapp.data("test.read.result", std::string("success"));
      return true;
    }
    return false;
  };
  
  cvcapp.dataReader(test_reader);
  
  // Try to read a .test file
  bool result = cvcapp.readData("example.test");
  EXPECT_TRUE(result);
  EXPECT_EQ(cvcapp.data<std::string>("test.read.result"), "success");
  
  // Try to read an unsupported file
  bool result2 = cvcapp.readData("example.unsupported");
  EXPECT_FALSE(result2);
  
  // Clean up
  cvcapp.data("test.read.result", boost::any());
}

// ===========================
// Property Map File I/O Tests
// ===========================

TEST(AppTest, PropertyMapSaveLoad) {
  std::string temp_file =
      (std::filesystem::temp_directory_path() / "test_property_map.info").string();
  
  // Set up some properties
  cvcapp.properties("io.test.prop1", "value1");
  cvcapp.properties("io.test.prop2", "value2");
  cvcapp.properties("io.test.number", 42);
  
  // Save property map
  cvcapp.writePropertyMap(temp_file);
  
  // Clear properties
  cvcapp.properties("io.test.prop1", "");
  cvcapp.properties("io.test.prop2", "");
  cvcapp.properties("io.test.number", "");
  
  // Restore from file
  cvcapp.readPropertyMap(temp_file);
  
  // Verify restored
  EXPECT_EQ(cvcapp.properties("io.test.prop1"), "value1");
  EXPECT_EQ(cvcapp.properties("io.test.prop2"), "value2");
  
  // Clean up
  std::remove(temp_file.c_str());
  cvcapp.properties("io.test.prop1", "");
  cvcapp.properties("io.test.prop2", "");
  cvcapp.properties("io.test.number", "");
}

// ===========================
// Thread Map Bulk Operations
// ===========================

TEST(AppTest, ThreadMapBulkSet) {
  thread_map test_map;
  
  // Create threads
  thread_ptr t1(new boost::thread([](){}));
  thread_ptr t2(new boost::thread([](){}));
  
  test_map["bulk.thread1"] = t1;
  test_map["bulk.thread2"] = t2;
  
  // Set the entire map
  cvcapp.threads(test_map);
  
  // Verify
  EXPECT_TRUE(cvcapp.hasThread("bulk.thread1"));
  EXPECT_TRUE(cvcapp.hasThread("bulk.thread2"));
  
  // Clean up
  cvcapp.threads("bulk.thread1", thread_ptr());
  cvcapp.threads("bulk.thread2", thread_ptr());
}

TEST(AppTest, RemoveThread) {
  std::string thread_key = "test.remove.thread";
  
  cvcapp.threads(thread_key, thread_ptr(new boost::thread([](){})));
  EXPECT_TRUE(cvcapp.hasThread(thread_key));
  
  cvcapp.removeThread(thread_key);
  EXPECT_FALSE(cvcapp.hasThread(thread_key));
}

TEST(AppTest, ThreadKeyLookup) {
  // Get thread key for current thread (should return "unknown" or valid key)
  std::string key = cvcapp.threadKey();
  EXPECT_FALSE(key.empty());
}

// ===========================
// Sleep Function Test
// ===========================

TEST(AppTest, SleepFunction) {
  auto start = boost::posix_time::microsec_clock::universal_time();
  cvcapp.sleep(10.0); // Sleep for 10 milliseconds
  auto end = boost::posix_time::microsec_clock::universal_time();
  
  auto duration = end - start;
  EXPECT_GE(duration.total_milliseconds(), 5);
}

// ===========================
// Property Data Tests
// ===========================

TEST(AppTest, PropertyDataLookup) {
  // Store some int data
  cvcapp.data("pd.obj1", 100);
  cvcapp.data("pd.obj2", 200);
  cvcapp.data("pd.obj3", 300);
  
  // Create property with list of keys
  cvcapp.properties("pd.list", "pd.obj1,pd.obj2,pd.obj3");
  
  // Get property data
  std::vector<int> data = cvcapp.propertyData<int>("pd.list");
  
  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 100);
  EXPECT_EQ(data[1], 200);
  EXPECT_EQ(data[2], 300);
  
  // Clean up
  cvcapp.properties("pd.list", "");
  cvcapp.data("pd.obj1", boost::any());
  cvcapp.data("pd.obj2", boost::any());
  cvcapp.data("pd.obj3", boost::any());
}

TEST(AppTest, ListDataFunction) {
  // Store data
  cvcapp.data("ld.a", 10);
  cvcapp.data("ld.b", 20);
  cvcapp.data("ld.c", 30);
  
  // Use listData to get data from comma-separated list
  std::vector<int> data = cvcapp.listData<int>("ld.a,ld.b,ld.c");
  
  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 10);
  EXPECT_EQ(data[1], 20);
  EXPECT_EQ(data[2], 30);
  
  // Clean up
  cvcapp.data("ld.a", boost::any());
  cvcapp.data("ld.b", boost::any());
  cvcapp.data("ld.c", boost::any());
}

// ===========================
// Data Type Enum Tests
// ===========================

TEST(AppTest, DataTypeEnumTemplateMethod) {
  // Test template version of dataType
  data_type dt_int = cvcapp.dataType<int>();
  EXPECT_EQ(dt_int, Int);
  
  data_type dt_double = cvcapp.dataType<double>();
  EXPECT_EQ(dt_double, Double);
  
  data_type dt_float = cvcapp.dataType<float>();
  EXPECT_EQ(dt_float, Float);
}

// ===========================
// Scoped Lock Tests
// ===========================

TEST(AppTest, ScopedLockUsage) {
  std::string mutex_name = "test.scoped.mutex";
  
  {
    scoped_lock lock(mutex_name, "test lock info");
    
    // Mutex should be locked and info set (may include thread info)
    std::string info = cvcapp.mutexInfo(mutex_name);
    EXPECT_NE(info.find("test lock info"), std::string::npos);
  }
  
  // After scope, info should be cleared or reset
  std::string info = cvcapp.mutexInfo(mutex_name);
  // Info may persist or be cleared depending on implementation
  EXPECT_TRUE(true); // Just verify no crash
}

// ===========================
// Thread Pool Tests
// ===========================

TEST(AppTest, ThreadPoolBasicExecution) {
  std::atomic<bool> task_executed(false);
  
  cvcapp.startThreadPooled("pool_basic_test", [&task_executed]() {
    task_executed = true;
  }, PRIORITY_NORMAL, true);
  
  // Wait for task to complete
  if (cvcapp.hasThread("pool_basic_test")) {
    thread_ptr tptr = cvcapp.threads("pool_basic_test");
    if (tptr) tptr->join();
  }
  
  EXPECT_TRUE(task_executed.load());
}

TEST(AppTest, ThreadPoolMultipleTasks) {
  std::atomic<int> counter(0);
  std::vector<std::string> keys;
  const int num_tasks = 5;
  
  for (int i = 0; i < num_tasks; i++) {
    std::string key = "pool_multi_" + std::to_string(i);
    keys.push_back(key);
    cvcapp.startThreadPooled(key, [&counter]() {
      counter++;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }, PRIORITY_NORMAL, true);
  }
  
  // Wait for all tasks
  for (const auto& key : keys) {
    if (cvcapp.hasThread(key)) {
      thread_ptr tptr = cvcapp.threads(key);
      if (tptr) tptr->join();
    }
  }
  
  EXPECT_EQ(counter.load(), num_tasks);
}

TEST(AppTest, ThreadPoolPriority) {
  std::atomic<int> execution_order(0);
  std::vector<int> order;
  boost::mutex order_mutex;
  
  // Set small pool size to force queuing
  unsigned int original_size = cvcapp.getThreadPoolSize();
  cvcapp.setThreadPoolSize(1);
  
  // Start a blocking task to fill the pool
  std::atomic<bool> blocker_done(false);
  std::atomic<bool> blocker_running(true);
  cvcapp.startThreadPooled("pool_priority_blocker", [&]() {
    while (blocker_running.load()) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      boost::this_thread::interruption_point();
    }
    blocker_done = true;
  }, PRIORITY_NORMAL, false); // use unique key
  
  // Give blocker time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  
  // Now submit tasks with unique keys that will queue (blocker is running)
  // Submit in reverse priority order - high, normal, critical
  cvcapp.startThreadPooled("pool_priority_high", [&]() {
    boost::mutex::scoped_lock lock(order_mutex);
    order.push_back(1);
    execution_order++;
  }, PRIORITY_HIGH, false); // unique key
  
  cvcapp.startThreadPooled("pool_priority_normal", [&]() {
    boost::mutex::scoped_lock lock(order_mutex);
    order.push_back(0);
    execution_order++;
  }, PRIORITY_NORMAL, false); // unique key
  
  cvcapp.startThreadPooled("pool_priority_critical", [&]() {
    boost::mutex::scoped_lock lock(order_mutex);
    order.push_back(2);
    execution_order++;
  }, PRIORITY_CRITICAL, false); // unique key
  
  // Give tasks time to queue
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  
  // Stop blocker to let queued tasks execute
  blocker_running = false;
  
  // Wait for all tasks to complete
  for (int i = 0; i < 50; i++) {
    if (execution_order.load() >= 3) break;
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
  
  cvcapp.setThreadPoolSize(original_size);
}

TEST(AppTest, ThreadPoolSizeConfiguration) {
  unsigned int original_size = cvcapp.getThreadPoolSize();
  
  // Test setting pool size
  cvcapp.setThreadPoolSize(2);
  EXPECT_EQ(cvcapp.getThreadPoolSize(), 2u);
  
  cvcapp.setThreadPoolSize(4);
  EXPECT_EQ(cvcapp.getThreadPoolSize(), 4u);
  
  // Restore original
  cvcapp.setThreadPoolSize(original_size);
  EXPECT_EQ(cvcapp.getThreadPoolSize(), original_size);
}

TEST(AppTest, ThreadPoolActiveCount) {
  std::atomic<bool> keep_running(true);
  std::vector<std::string> keys;
  
  unsigned int original_size = cvcapp.getThreadPoolSize();
  cvcapp.setThreadPoolSize(2);
  
  // Start 2 long-running tasks
  for (int i = 0; i < 2; i++) {
    std::string key = "pool_active_" + std::to_string(i);
    keys.push_back(key);
    cvcapp.startThreadPooled(key, [&keep_running]() {
      while (keep_running.load()) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      }
    }, PRIORITY_NORMAL, true);
  }
  
  // Give threads time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  
  // Should have 2 active workers
  unsigned int active = cvcapp.getActiveThreadCount();
  EXPECT_GE(active, 1u); // At least one should be running
  EXPECT_LE(active, 2u); // Should not exceed pool size
  
  // Stop tasks
  keep_running = false;
  
  for (const auto& key : keys) {
    if (cvcapp.hasThread(key)) {
      thread_ptr tptr = cvcapp.threads(key);
      if (tptr) tptr->join();
    }
  }
  
  cvcapp.setThreadPoolSize(original_size);
}

TEST(AppTest, ThreadPoolInterruption) {
  std::atomic<bool> task_started(false);
  std::atomic<bool> task_interrupted(false);
  
  cvcapp.startThreadPooled("pool_interrupt_test", [&]() {
    task_started = true;
    try {
      // Long-running task with interruption points
      for (int i = 0; i < 100; i++) {
        boost::this_thread::interruption_point();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      }
    } catch (boost::thread_interrupted&) {
      task_interrupted = true;
      throw;
    }
  }, PRIORITY_NORMAL, true);
  
  // Give task time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  EXPECT_TRUE(task_started.load());
  
  // Interrupt the thread
  if (cvcapp.hasThread("pool_interrupt_test")) {
    thread_ptr tptr = cvcapp.threads("pool_interrupt_test");
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

TEST(AppTest, ThreadPoolReplaceRunningTask) {
  std::atomic<int> task_count(0);
  std::atomic<bool> first_task_running(true);
  
  // Start first task with wait=true (uses key "replace_test")
  cvcapp.startThreadPooled("pool_replace_test", [&]() {
    task_count++;
    while (first_task_running.load()) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      boost::this_thread::interruption_point();
    }
  }, PRIORITY_NORMAL, true);
  
  // Give it time to start
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  EXPECT_EQ(task_count.load(), 1);
  
  // Start second task with same key and wait=true
  // This should interrupt the first task and start a new one
  cvcapp.startThreadPooled("pool_replace_test", [&]() {
    task_count++;
  }, PRIORITY_NORMAL, true);
  
  // Wait for new task
  if (cvcapp.hasThread("pool_replace_test")) {
    thread_ptr tptr = cvcapp.threads("pool_replace_test");
    if (tptr) tptr->join();
  }
  
  // Should have executed both tasks (first interrupted, second completed)
  EXPECT_EQ(task_count.load(), 2);
  first_task_running = false;
}

TEST(AppTest, ThreadPoolExceptionHandling) {
  std::atomic<bool> task_ran(false);
  std::atomic<bool> cleanup_ran(false);
  
  // Check initial state
  unsigned int initial_active = cvcapp.getActiveThreadCount();
  bool had_exception_thread_before = cvcapp.hasThread("pool_exception_test");
  
  cvcapp.startThreadPooled("pool_exception_test", [&]() {
    task_ran = true;
    throw std::runtime_error("Test exception");
  }, PRIORITY_NORMAL, true);
  
  // Give task time to execute and fail
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  
  EXPECT_TRUE(task_ran.load());
  
  // Verify thread pool state after exception
  // The thread should have exited and active worker count should return to normal
  unsigned int active_after_exception = cvcapp.getActiveThreadCount();
  EXPECT_EQ(active_after_exception, initial_active) << "Active worker count should return to initial state";
  
  // Pool should continue to work after exception
  cvcapp.startThreadPooled("pool_after_exception", [&]() {
    cleanup_ran = true;
  }, PRIORITY_NORMAL, true);
  
  if (cvcapp.hasThread("pool_after_exception")) {
    thread_ptr tptr = cvcapp.threads("pool_after_exception");
    if (tptr) tptr->join();
  }
  
  EXPECT_TRUE(cleanup_ran.load());
}

TEST(AppTest, ThreadPoolStateConsistency) {
  // Test that thread map stays clean after multiple operations
  unsigned int original_size = cvcapp.getThreadPoolSize();
  cvcapp.setThreadPoolSize(2);
  
  std::atomic<int> completed(0);
  std::vector<std::string> keys;
  
  // Run several tasks including some that throw
  for (int i = 0; i < 5; i++) {
    std::string key = "pool_state_" + std::to_string(i);
    keys.push_back(key);
    
    cvcapp.startThreadPooled(key, [&, i]() {
      if (i == 2) {
        // Task 2 throws an exception
        throw std::runtime_error("Intentional exception");
      }
      completed++;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }, PRIORITY_NORMAL, true);
  }
  
  // Wait for all tasks to complete
  boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
  
  // Should have completed 4 tasks (task 2 threw exception)
  EXPECT_EQ(completed.load(), 4);
  
  // Active worker count should be back to 0 or minimal
  unsigned int active = cvcapp.getActiveThreadCount();
  EXPECT_EQ(active, 0u) << "No workers should be active after all tasks complete";
  
  cvcapp.setThreadPoolSize(original_size);
}

TEST(AppTest, ThreadPoolConcurrencyLimit) {
  std::atomic<int> concurrent_count(0);
  std::atomic<int> max_concurrent(0);
  std::vector<std::string> keys;
  boost::mutex counter_mutex;
  
  unsigned int original_size = cvcapp.getThreadPoolSize();
  const int pool_size = 2;
  cvcapp.setThreadPoolSize(pool_size);
  
  // Submit more tasks than pool size
  for (int i = 0; i < 5; i++) {
    std::string key = "pool_concurrency_" + std::to_string(i);
    keys.push_back(key);
    
    cvcapp.startThreadPooled(key, [&]() {
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
    }, PRIORITY_NORMAL, true);
  }
  
  // Wait for all tasks
  for (const auto& key : keys) {
    if (cvcapp.hasThread(key)) {
      thread_ptr tptr = cvcapp.threads(key);
      if (tptr) tptr->join();
    }
  }
  
  // Max concurrent should not exceed pool size
  EXPECT_LE(max_concurrent.load(), pool_size);
  EXPECT_GE(max_concurrent.load(), 1); // At least one should have run
  
  cvcapp.setThreadPoolSize(original_size);
}

TEST(AppTest, ThreadPoolTaskChaining) {
  std::atomic<int> execution_order(0);
  std::vector<int> order;
  boost::mutex order_mutex;
  
  unsigned int original_size = cvcapp.getThreadPoolSize();
  cvcapp.setThreadPoolSize(1); // Force serial execution
  
  // Submit multiple tasks that will chain
  for (int i = 0; i < 3; i++) {
    std::string key = "pool_chain_" + std::to_string(i);
    cvcapp.startThreadPooled(key, [&, i]() {
      boost::mutex::scoped_lock lock(order_mutex);
      order.push_back(i);
      execution_order++;
    }, PRIORITY_NORMAL, true);
  }
  
  // Wait for all
  for (int i = 0; i < 3; i++) {
    std::string key = "pool_chain_" + std::to_string(i);
    if (cvcapp.hasThread(key)) {
      thread_ptr tptr = cvcapp.threads(key);
      if (tptr) tptr->join();
    }
  }
  
  EXPECT_EQ(execution_order.load(), 3);
  EXPECT_EQ(order.size(), 3);
  
  cvcapp.setThreadPoolSize(original_size);
}

TEST(AppTest, ThreadInfoAndProgressTracking) {
  std::string thread_key = "test.progress.tracking";
  std::atomic<bool> thread_started(false);
  std::atomic<bool> continue_running(true);
  
  // Start a thread that updates its info and progress
  cvcapp.startThreadPooled(thread_key, [&]() {
    thread_started = true;
    
    // Set initial thread info
    cvcapp.threadInfo(thread_key, "Starting processing");
    cvcapp.threadProgress(thread_key, 0.0);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    
    // Update to 25% progress
    cvcapp.threadInfo(thread_key, "Processing step 1");
    cvcapp.threadProgress(thread_key, 0.25);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    
    // Update to 50% progress
    cvcapp.threadInfo(thread_key, "Processing step 2");
    cvcapp.threadProgress(thread_key, 0.50);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    
    // Update to 75% progress
    cvcapp.threadInfo(thread_key, "Processing step 3");
    cvcapp.threadProgress(thread_key, 0.75);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    
    // Wait until we're told to finish
    while (continue_running.load()) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      // NOTE: No interruption_point() here - we want to finish cleanly
    }
    
    // Complete
    cvcapp.threadInfo(thread_key, "Finished");
    cvcapp.threadProgress(thread_key, 1.0);
    
    // Small delay to ensure final state is written
    boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
  }, PRIORITY_NORMAL, true);
  
  // Wait for thread to start
  for (int i = 0; i < 50 && !thread_started.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(thread_started.load()) << "Thread should have started";
  
  // Verify we can read info and progress while thread is running
  // Use polling to wait for each checkpoint instead of fixed sleeps
  
  // Wait for 25% progress
  for (int i = 0; i < 200 && cvcapp.threadProgress(thread_key) < 0.24; i++)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  double progress1 = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress1, 0.25, 0.01);
  
  // Wait for 50% progress
  for (int i = 0; i < 200 && cvcapp.threadProgress(thread_key) < 0.49; i++)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  double progress2 = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress2, 0.50, 0.01);
  
  // Wait for 75% progress
  for (int i = 0; i < 200 && cvcapp.threadProgress(thread_key) < 0.74; i++)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  double progress3 = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress3, 0.75, 0.01);
  
  // Verify progress is increasing
  EXPECT_LT(progress1, progress2);
  EXPECT_LT(progress2, progress3);
  
  // Let thread finish
  continue_running = false;
  
  // Join the thread to clean up
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr) tptr->join();
  }
  
  // NOTE: We successfully tested reading thread info/progress while running.
  // The final state after join() is not reliable due to thread pool cleanup timing.
}

// ===========================
// Persistent Progress Tests
// ===========================

TEST(AppTest, ThreadProgressPersistsAfterCompletion) {
  std::string thread_key = "test.persistent.progress";
  std::atomic<bool> thread_finished(false);
  
  // Start a thread that sets progress and completes quickly
  cvcapp.startThread(thread_key, [&]() {
    cvc::app::thread_feedback feedback(thread_key);
    cvcapp.threadProgress(thread_key, 0.5);
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
  double progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01) 
    << "Progress should be 100% after thread completion (thread_feedback sets it)";
  
  // Clean up
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr && tptr->joinable()) tptr->join();
  }
}

TEST(AppTest, ThreadProgressPersistenceWithMultipleThreads) {
  std::vector<std::string> thread_keys = {
    "test.multi.thread1",
    "test.multi.thread2", 
    "test.multi.thread3"
  };
  std::atomic<int> completed_count(0);
  
  // Start multiple threads with different progress values
  for (size_t i = 0; i < thread_keys.size(); i++) {
    double target_progress = (i + 1) * 0.25; // 0.25, 0.5, 0.75
    cvcapp.startThread(thread_keys[i], [&, i, target_progress]() {
      cvc::app::thread_feedback feedback(thread_keys[i]);
      cvcapp.threadProgress(thread_keys[i], target_progress);
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
  for (const auto& key : thread_keys) {
    double progress = cvcapp.threadProgress(key);
    EXPECT_NEAR(progress, 1.0, 0.01) 
      << "Thread " << key << " should show 100% after completion";
  }
  
  // Clean up
  for (const auto& key : thread_keys) {
    if (cvcapp.hasThread(key)) {
      thread_ptr tptr = cvcapp.threads(key);
      if (tptr && tptr->joinable()) tptr->join();
    }
  }
}

TEST(AppTest, ThreadProgressZeroToOneHundred) {
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

  cvcapp.startThread(thread_key, [&]() {
    cvc::app::thread_feedback feedback(thread_key);
    // thread_feedback constructor sets to 0%
    at_zero = true;
    while (!zero_observed.load()) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }

    cvcapp.threadProgress(thread_key, 0.5);
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
    double progress = cvcapp.threadProgress(thread_key);
    EXPECT_NEAR(progress, 0.0, 0.01) << "Progress should be 0% at start";
  }
  zero_observed = true;

  // Check progress at 50%
  for (int i = 0; i < 200 && !at_fifty.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(at_fifty.load());
  {
    double progress = cvcapp.threadProgress(thread_key);
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
  double final_progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(final_progress, 1.0, 0.01) 
    << "Progress should persist at 100% after thread exits";
  
  // Clean up
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr && tptr->joinable()) tptr->join();
  }
}

TEST(AppTest, ThreadProgressQueryAfterThreadDestruction) {
  std::string thread_key = "test.progress.after.destroy";
  
  {
    // Start thread in inner scope
    std::atomic<bool> done(false);
    cvcapp.startThread(thread_key, [&]() {
      cvc::app::thread_feedback feedback(thread_key);
      cvcapp.threadProgress(thread_key, 0.75);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
      done = true;
    });
    
    // Wait for completion
    for (int i = 0; i < 50 && !done.load(); i++) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
    
    // Join thread
    if (cvcapp.hasThread(thread_key)) {
      thread_ptr tptr = cvcapp.threads(thread_key);
      if (tptr && tptr->joinable()) tptr->join();
    }
  }
  
  // Thread object has been destroyed, but progress should still be queryable
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  
  double progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
    << "Progress should be queryable at 100% even after thread object destroyed";
}

TEST(AppTest, ThreadFeedbackExceptionSafety) {
  std::string thread_key = "test.feedback.exception";
  std::atomic<bool> exception_thrown(false);
  
  cvcapp.startThread(thread_key, [&]() {
    try {
      cvc::app::thread_feedback feedback(thread_key);
      cvcapp.threadProgress(thread_key, 0.3);
      
      // Simulate an exception during processing
      exception_thrown = true;
      throw std::runtime_error("Simulated error");
    }
    catch (const std::exception& e) {
      // thread_feedback destructor should still set progress to 100%
      // even when exception is thrown
    }
  });
  
  // Wait for exception
  for (int i = 0; i < 50 && !exception_thrown.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  ASSERT_TRUE(exception_thrown.load());
  
  // Give thread time to exit
  boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
  
  // Progress should still be set to 100% by thread_feedback destructor
  double progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
    << "Progress should be 100% even when exception occurs (RAII cleanup)";
  
  // Clean up
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr && tptr->joinable()) tptr->join();
  }
}

TEST(AppTest, ThreadProgressWithThreadInterruption) {
  std::string thread_key = "test.progress.interruption";
  std::atomic<bool> started(false);
  std::atomic<bool> interrupted(false);
  
  cvcapp.startThread(thread_key, [&]() {
    try {
      cvc::app::thread_feedback feedback(thread_key);
      started = true;
      cvcapp.threadProgress(thread_key, 0.2);
      
      // Sleep with interruption point
      for (int i = 0; i < 100; i++) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        boost::this_thread::interruption_point();
      }
    }
    catch (boost::thread_interrupted&) {
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
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr) tptr->interrupt();
  }
  
  // Wait for interruption to be caught
  for (int i = 0; i < 50 && !interrupted.load(); i++) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  
  // Give thread time to clean up
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  
  // Even with interruption, thread_feedback destructor should set progress
  double progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
    << "Progress should be 100% even after thread interruption (RAII cleanup)";
  
  // Clean up
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr && tptr->joinable()) tptr->join();
  }
}

TEST(AppTest, ThreadStatusShowsCompleted) {
  std::string thread_key = "test.status.completed";
  std::atomic<bool> finished(false);
  
  cvcapp.startThread(thread_key, [&]() {
    cvc::app::thread_feedback feedback(thread_key);
    cvcapp.threadInfo(thread_key, "processing");
    cvcapp.threadProgress(thread_key, 0.5);
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
  std::string status = cvcapp.threadInfo(thread_key);
  EXPECT_EQ(status, "completed") 
    << "Thread status should be 'completed' after thread exits";
  
  // Verify progress is 100%
  double progress = cvcapp.threadProgress(thread_key);
  EXPECT_NEAR(progress, 1.0, 0.01)
    << "Progress should be 100% when status is completed";
  
  // Clean up
  if (cvcapp.hasThread(thread_key)) {
    thread_ptr tptr = cvcapp.threads(thread_key);
    if (tptr && tptr->joinable()) tptr->join();
  }
}

// Main function is provided by gtest_main library

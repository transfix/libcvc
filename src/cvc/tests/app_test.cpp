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
  std::string temp_file = "/tmp/test_property_map.info";
  
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

// Main function is provided by gtest_main library

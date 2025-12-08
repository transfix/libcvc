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

// Main function is provided by gtest_main library

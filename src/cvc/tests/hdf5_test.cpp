/*
  Copyright 2025 The University of Texas at Austin

  Unit tests for HDF5 utilities functionality

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <boost/format.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/bounding_box.h>
#include <cvc/dimension.h>
#include <cvc/hdf5_utils.h>
#include <cvc/volume.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#define CVC_GETPID() ::_getpid()
#else
#include <unistd.h>
#define CVC_GETPID() ::getpid()
#endif

using namespace cvc;

// ===========================
// HDF5 Test Fixture
// ===========================
// Creates temporary test files in a thread/process-unique directory
// to allow parallel test execution

class HDF5Test : public ::testing::Test {
protected:
  std::string test_dir;
  app ctx;

  virtual void SetUp() {
    // Create a unique directory for this test instance. We need to be unique
    // both across parallel ctest *processes* (so we include the pid) and across
    // tests within the same process (so we include a per-process atomic
    // counter and high-resolution clock). std::this_thread::get_id() alone
    // collides across freshly-spawned processes on macOS, where the main
    // thread id is often reused, which lets parallel ctest workers stomp on
    // each other's HDF5 files and surface as flaky
    // "objectExists() == false" / "unknown file: Failure" failures.
    namespace fs = std::filesystem;
    static std::atomic<unsigned> counter{0};
    std::ostringstream oss;
    oss << "hdf5_test_" << CVC_GETPID() << "_" << std::this_thread::get_id() << "_"
        << counter.fetch_add(1, std::memory_order_relaxed) << "_"
        << std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path base = std::getenv("CMAKE_CURRENT_BINARY_DIR")
                        ? fs::path(std::getenv("CMAKE_CURRENT_BINARY_DIR"))
                        : fs::current_path();
    fs::path dir = base / oss.str();
    fs::create_directories(dir);
    test_dir = dir.string();
  }

  virtual void TearDown() {
    std::error_code ec;
    std::filesystem::remove_all(test_dir, ec);
  }

  std::string getTestFilePath(const std::string &filename) { return test_dir + "/" + filename; }
};

// ===========================
// Basic File Operations
// ===========================

TEST_F(HDF5Test, GetH5FileCreate) {
  std::string filepath = getTestFilePath("test_create.h5");

  // Test creating a new file
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);
  EXPECT_TRUE(file.get() != NULL);

  file.reset(); // Close file

  // Verify file exists
  std::ifstream check(filepath.c_str());
  EXPECT_TRUE(check.good());
}

TEST_F(HDF5Test, GetH5FileOpen) {
  std::string filepath = getTestFilePath("test_open.h5");

  // Create file first
  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);
    EXPECT_TRUE(file.get() != NULL);
  }

  // Now open it
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, false);
  EXPECT_TRUE(file.get() != NULL);
}

TEST_F(HDF5Test, GetH5FileOpenNonExistent) {
  std::string filepath = getTestFilePath("nonexistent.h5");

  // The implementation catches exceptions and creates a file if one doesn't exist
  // So we just verify that we can call it without error
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, false);
  EXPECT_TRUE(file.get() != NULL);
}

// ===========================
// Group Operations
// ===========================

TEST_F(HDF5Test, GetGroupCreate) {
  std::string filepath = getTestFilePath("test_groups.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create a group
  boost::shared_ptr<H5::Group> group = hdf5_utils::getGroup(*file, "mygroup", true);
  EXPECT_TRUE(group.get() != NULL);
}

TEST_F(HDF5Test, GetGroupNested) {
  std::string filepath = getTestFilePath("test_nested_groups.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create nested groups
  boost::shared_ptr<H5::Group> group = hdf5_utils::getGroup(*file, "level1/level2/level3", true);
  EXPECT_TRUE(group.get() != NULL);
}

TEST_F(HDF5Test, GetGroupOpenExisting) {
  std::string filepath = getTestFilePath("test_group_open.h5");

  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);
    hdf5_utils::getGroup(*file, "mygroup", true);
  }

  // Reopen and verify group exists
  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, false);
    boost::shared_ptr<H5::Group> group = hdf5_utils::getGroup(*file, "mygroup", false);
    EXPECT_TRUE(group.get() != NULL);
  }
}

// ===========================
// DataSet Operations
// ===========================

TEST_F(HDF5Test, GetDataSetCreateUChar) {
  std::string filepath = getTestFilePath("test_dataset_uchar.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create dataspace and dataset
  hsize_t dims[1] = {10};
  H5::DataSpace dataspace(1, dims);
  boost::shared_ptr<H5::DataSet> dataset(
      new H5::DataSet(file->createDataSet("data", H5::PredType::NATIVE_UCHAR, dataspace)));
  EXPECT_TRUE(dataset.get() != NULL);
}

TEST_F(HDF5Test, GetDataSetCreateNested) {
  std::string filepath = getTestFilePath("test_dataset_nested.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create nested groups first
  boost::shared_ptr<H5::Group> group = hdf5_utils::getGroup(*file, "group1/group2", true);

  // Now create dataset in the group
  hsize_t dims[1] = {10};
  H5::DataSpace dataspace(1, dims);
  boost::shared_ptr<H5::DataSet> dataset(
      new H5::DataSet(group->createDataSet("mydata", H5::PredType::NATIVE_UCHAR, dataspace)));
  EXPECT_TRUE(dataset.get() != NULL);
}

// ===========================
// Attribute Operations
// ===========================

TEST_F(HDF5Test, HasAttributeTrue) {
  std::string filepath = getTestFilePath("test_attribute.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Add an attribute
  double value = 3.14159;
  H5::Attribute attr =
      file->createAttribute("test_attr", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(H5S_SCALAR));
  attr.write(H5::PredType::NATIVE_DOUBLE, &value);

  // Check it exists
  EXPECT_TRUE(hdf5_utils::hasAttribute(*file, "test_attr"));
}

TEST_F(HDF5Test, HasAttributeFalse) {
  std::string filepath = getTestFilePath("test_no_attribute.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Check non-existent attribute
  EXPECT_FALSE(hdf5_utils::hasAttribute(*file, "nonexistent_attr"));
}

TEST_F(HDF5Test, GetAttributeDouble) {
  std::string filepath = getTestFilePath("test_get_attribute.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Set an attribute with a vector of doubles
  std::vector<double> values = {1.1, 2.2, 3.3};
  H5::DataSpace ds(1, new hsize_t[1]{3});
  H5::Attribute attr = file->createAttribute("doubles", H5::PredType::NATIVE_DOUBLE, ds);
  attr.write(H5::PredType::NATIVE_DOUBLE, values.data());

  // Read it back
  std::vector<double> read_values(3);
  hdf5_utils::getAttribute(*file, "doubles", 3, read_values.data());

  EXPECT_DOUBLE_EQ(read_values[0], 1.1);
  EXPECT_DOUBLE_EQ(read_values[1], 2.2);
  EXPECT_DOUBLE_EQ(read_values[2], 3.3);
}

TEST_F(HDF5Test, SetGetAttributeUInt) {
  std::string filepath = getTestFilePath("test_uint_attr.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Set attributes with unsigned ints
  std::vector<unsigned int> values = {10, 20, 30};
  H5::DataSpace ds(1, new hsize_t[1]{3});
  H5::Attribute attr = file->createAttribute("uints", H5::PredType::NATIVE_UINT, ds);
  attr.write(H5::PredType::NATIVE_UINT, values.data());

  // Read them back
  std::vector<unsigned int> read_values(3);
  hdf5_utils::getAttribute(*file, "uints", 3, read_values.data());

  EXPECT_EQ(read_values[0], 10U);
  EXPECT_EQ(read_values[1], 20U);
  EXPECT_EQ(read_values[2], 30U);
}

// ===========================
// Unlink Operations
// ===========================

TEST_F(HDF5Test, UnlinkDataSet) {
  std::string filepath = getTestFilePath("test_unlink_dataset.h5");

  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

    // Create a dataset explicitly
    hsize_t dims[1] = {10};
    H5::DataSpace dataspace(1, dims);
    file->createDataSet("data_to_remove", H5::PredType::NATIVE_DOUBLE, dataspace);
    file->flush(H5F_SCOPE_GLOBAL);
    EXPECT_TRUE(hdf5_utils::objectExists(ctx, filepath, "data_to_remove"));

    // Unlink it
    hdf5_utils::unlink(*file, "data_to_remove");
    file->flush(H5F_SCOPE_GLOBAL);
    EXPECT_FALSE(hdf5_utils::objectExists(ctx, filepath, "data_to_remove"));
  }
}

TEST_F(HDF5Test, UnlinkNestedObject) {
  std::string filepath = getTestFilePath("test_unlink_nested.h5");

  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

    // Create nested groups and dataset
    {
      boost::shared_ptr<H5::Group> grp = hdf5_utils::getGroup(*file, "group/subgroup", true);

      hsize_t dims[1] = {10};
      H5::DataSpace dataspace(1, dims);
      grp->createDataSet("data", H5::PredType::NATIVE_DOUBLE, dataspace);
    }
    // Flush so the secondary file handle opened by objectExists() observes
    // the freshly-created dataset metadata. On macOS Release the H5 library
    // is otherwise free to defer the flush past the inner-handle release,
    // which makes objectExists() race against the still-buffered group
    // state and report a missing object.
    file->flush(H5F_SCOPE_GLOBAL);
    EXPECT_TRUE(hdf5_utils::objectExists(ctx, filepath, "group/subgroup/data"));

    // Unlink it
    hdf5_utils::unlink(*file, "group/subgroup/data");
    file->flush(H5F_SCOPE_GLOBAL);
    EXPECT_FALSE(hdf5_utils::objectExists(ctx, filepath, "group/subgroup/data"));
  }
}

// ===========================
// PredType Conversion
// ===========================

TEST_F(HDF5Test, GetPredTypeUChar) {
  H5::PredType pt = hdf5_utils::getPredType(UChar);
  EXPECT_TRUE(pt == H5::PredType::NATIVE_UCHAR);
}

TEST_F(HDF5Test, GetPredTypeUShort) {
  H5::PredType pt = hdf5_utils::getPredType(UShort);
  EXPECT_TRUE(pt == H5::PredType::NATIVE_USHORT);
}

TEST_F(HDF5Test, GetPredTypeUInt) {
  H5::PredType pt = hdf5_utils::getPredType(UInt);
  EXPECT_TRUE(pt == H5::PredType::NATIVE_UINT);
}

TEST_F(HDF5Test, GetPredTypeUInt64) {
  H5::PredType pt = hdf5_utils::getPredType(UInt64);
  EXPECT_TRUE(pt == H5::PredType::NATIVE_UINT64);
}

TEST_F(HDF5Test, GetPredTypeFloat) {
  H5::PredType pt = hdf5_utils::getPredType(Float);
  EXPECT_TRUE(pt == H5::PredType::NATIVE_FLOAT);
}

TEST_F(HDF5Test, GetPredTypeDouble) {
  H5::PredType pt = hdf5_utils::getPredType(Double);
  EXPECT_TRUE(pt == H5::PredType::NATIVE_DOUBLE);
}

// ===========================
// String Attribute Operations
// ===========================

TEST_F(HDF5Test, SetGetStringAttribute) {
  std::string filepath = getTestFilePath("test_string_attr.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create a simple dataset to attach attributes to
  hsize_t dims[1] = {10};
  H5::DataSpace dataspace(1, dims);
  H5::DataSet dataset = file->createDataSet("data", H5::PredType::NATIVE_DOUBLE, dataspace);

  // Set a string attribute
  std::string test_string = "Hello HDF5";
  H5::StrType stype(H5::PredType::C_S1, test_string.length() + 1);
  H5::Attribute attr = dataset.createAttribute("name", stype, H5::DataSpace(H5S_SCALAR));
  attr.write(stype, test_string);

  // Verify it exists
  EXPECT_TRUE(hdf5_utils::hasAttribute(dataset, "name"));
}

// ===========================
// Complex Multi-type Tests
// ===========================

TEST_F(HDF5Test, MultipleGroupsAndDataSets) {
  std::string filepath = getTestFilePath("test_complex.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create multiple groups and datasets
  for (int i = 0; i < 5; ++i) {
    std::string group_path = std::string("group") + boost::lexical_cast<std::string>(i);
    boost::shared_ptr<H5::Group> group = hdf5_utils::getGroup(*file, group_path, true);

    for (int j = 0; j < 3; ++j) {
      // Create dataset explicitly
      hsize_t dims[1] = {10};
      H5::DataSpace dataspace(1, dims);
      std::string ds_name = std::string("data") + boost::lexical_cast<std::string>(j);
      group->createDataSet(ds_name, H5::PredType::NATIVE_DOUBLE, dataspace);
    }
  }

  // Verify all groups were created
  EXPECT_TRUE(hdf5_utils::objectExists(ctx, filepath, "group0"));
  EXPECT_TRUE(hdf5_utils::objectExists(ctx, filepath, "group4"));
}

TEST_F(HDF5Test, MixedAttributes) {
  std::string filepath = getTestFilePath("test_mixed_attrs.h5");
  boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

  // Create a dataset
  hsize_t dims[1] = {10};
  H5::DataSpace dataspace(1, dims);
  H5::DataSet dataset = file->createDataSet("mydata", H5::PredType::NATIVE_DOUBLE, dataspace);

  // Add multiple attributes of different types
  double dval = 3.14159;
  unsigned int uival = 42;

  H5::Attribute attr_d =
      dataset.createAttribute("pi", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(H5S_SCALAR));
  attr_d.write(H5::PredType::NATIVE_DOUBLE, &dval);

  H5::Attribute attr_u =
      dataset.createAttribute("answer", H5::PredType::NATIVE_UINT, H5::DataSpace(H5S_SCALAR));
  attr_u.write(H5::PredType::NATIVE_UINT, &uival);

  // Verify they exist
  EXPECT_TRUE(hdf5_utils::hasAttribute(dataset, "pi"));
  EXPECT_TRUE(hdf5_utils::hasAttribute(dataset, "answer"));
  EXPECT_FALSE(hdf5_utils::hasAttribute(dataset, "nonexistent"));
}

// ===========================
// Round-trip Tests
// ===========================

TEST_F(HDF5Test, CreateModifyRead) {
  std::string filepath = getTestFilePath("test_roundtrip.h5");

  // Create and write
  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, true);

    boost::shared_ptr<H5::Group> group = hdf5_utils::getGroup(*file, "data", true);

    // Create dataset explicitly
    hsize_t dims[1] = {10};
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = group->createDataSet("values", H5::PredType::NATIVE_DOUBLE, dataspace);

    // Write an attribute
    double dval = 3.14159;
    H5::Attribute attr = dataset.createAttribute("stored_value", H5::PredType::NATIVE_DOUBLE,
                                                 H5::DataSpace(H5S_SCALAR));
    attr.write(H5::PredType::NATIVE_DOUBLE, &dval);
  }

  // Reopen and verify
  {
    boost::shared_ptr<H5::H5File> file = hdf5_utils::getH5File(filepath, false);

    EXPECT_TRUE(hdf5_utils::objectExists(ctx, filepath, "data"));
    EXPECT_TRUE(hdf5_utils::objectExists(ctx, filepath, "data/values"));

    boost::shared_ptr<H5::DataSet> dataset = hdf5_utils::getDataSet(*file, "data/values", false);

    EXPECT_TRUE(hdf5_utils::hasAttribute(*dataset, "stored_value"));
  }
}

// Main function is provided by gtest_main library

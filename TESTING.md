# Unit Testing Guide for trans-cvc

## Overview

The trans-cvc library uses Google Test (gtest) for unit testing. Tests are organized to validate the core functionality of the `cvc::app` and `cvc::state` classes, which form the foundation of the library's application state management system.

## Building Tests

### CMake Option

Tests are controlled by the `CVC_BUILD_TESTS` CMake option, which is **ON by default**. To disable tests:

```bash
cmake -DCVC_BUILD_TESTS=OFF /path/to/trans-cvc
```

### Building with Tests

```bash
# Configure with tests enabled (default)
cmake -B build -S . -DCVC_BUILD_TESTS=ON

# Build the project including tests
cmake --build build

# Or build only the test executables
cmake --build build --target app_test state_test
```

### Google Test Integration

The build system automatically fetches Google Test v1.14.0 from GitHub using CMake's FetchContent module. No manual installation of Google Test is required.

## Running Tests

### Using CTest

CTest is the standard way to run tests in CMake projects:

```bash
# Run all tests
cd build
ctest

# Run tests with verbose output
ctest --output-on-failure

# Run tests in parallel
ctest -j$(nproc)

# Run specific test by name
ctest -R app_test
ctest -R state_test
```

### Using the Custom 'check' Target

A convenient `check` target is provided:

```bash
cmake --build build --target check
```

This runs all tests and displays output on failure.

### Running Test Executables Directly

You can also run the test executables directly for more control:

```bash
# Run app tests
./build/bin/app_test

# Run state tests
./build/bin/state_test

# Run with filters
./build/bin/app_test --gtest_filter=AppTest.DataSetAndGet

# List all tests
./build/bin/app_test --gtest_list_tests

# Run tests multiple times to detect flakiness
./build/bin/app_test --gtest_repeat=100
```

## Test Organization

### Test Files

- **`src/cvc/tests/app_test.cpp`**: Unit tests for `cvc::app` class
- **`src/cvc/tests/state_test.cpp`**: Unit tests for `cvc::state` class
- **`src/cvc/tests/CMakeLists.txt`**: Test build configuration

### Test Coverage

#### `cvc::app` Tests (app_test.cpp)

The `app` class tests validate:

- **Singleton Pattern**: Verify single instance across accesses
- **Data Management**: Set/get data with various types (int, double, string, bool)
- **Data Type Registry**: Verify type registration and enumeration
- **Property Management**: Set/get properties, list properties
- **Property Lists**: Comma-separated values, unique elements, append/remove
- **Thread Management**: Thread key generation, progress tracking, thread info
- **Mutex Management**: Named mutex creation and information
- **Utility Functions**: `listify()` conversions between strings and vectors

Example test structure:
```cpp
TEST(AppTest, DataSetAndGet) {
  std::string test_key = "test.data.string";
  std::string test_value = "Hello, World!";
  
  cvcapp.data(test_key, test_value);
  
  ASSERT_TRUE(cvcapp.isData<std::string>(test_key));
  EXPECT_EQ(cvcapp.data<std::string>(test_key), test_value);
  
  // Clean up
  cvcapp.data(test_key, boost::any());
}
```

#### `cvc::state` Tests (state_test.cpp)

The `state` class tests validate:

- **Singleton Pattern**: Verify singleton state object
- **Value Management**: Set/get values with type conversion
- **Value Lists**: Comma-separated values with trimming and uniqueness
- **Data Storage**: Arbitrary typed data via boost::any
- **Hierarchy**: Parent-child relationships, navigation
- **Children Listing**: Enumerate children, regex filtering
- **Metadata**: Comments, hidden flag, initialization status
- **Timestamps**: Last modification tracking
- **State Manipulation**: Touch, reset operations
- **Serialization**: Property tree conversion, JSON export/import
- **ValueData**: Reference data objects by key lists
- **Traversal**: Tree traversal with callbacks

Example test structure:
```cpp
TEST(StateTest, ValueSetAndGet) {
  std::string test_value = "test_string_value";
  cvcstate("test.value.simple").value(test_value);
  
  EXPECT_EQ(cvcstate("test.value.simple").value(), test_value);
  EXPECT_TRUE(cvcstate("test.value.simple").initialized());
  
  // Clean up
  cvcstate("test.value.simple").reset();
}
```

## Test Design Principles

### 1. Isolation

Each test is independent and cleans up after itself:
```cpp
// Test code
cvcstate("test.example").value("test");
// Assertions
EXPECT_EQ(cvcstate("test.example").value(), "test");
// Cleanup
cvcstate("test.example").reset();
```

### 2. Namespace Usage

Tests use unique namespaces to avoid conflicts:
- `test.value.*` - Value-related tests
- `test.data.*` - Data storage tests
- `test.property.*` - Property tests
- `test.children.*` - Hierarchy tests

### 3. Thread Safety

The `app` and `state` classes are designed to be thread-safe. Tests verify single-threaded behavior; future tests may add concurrency validation.

## Adding New Tests

### Step 1: Choose the Appropriate File

- Add to `app_test.cpp` for `cvc::app` functionality
- Add to `state_test.cpp` for `cvc::state` functionality
- Create a new test file for new components

### Step 2: Follow the TEST Macro Pattern

```cpp
TEST(TestSuiteName, TestName) {
  // Arrange: Set up test conditions
  std::string key = "test.new.feature";
  
  // Act: Exercise the functionality
  cvcstate(key).value("test");
  
  // Assert: Verify expected outcomes
  EXPECT_EQ(cvcstate(key).value(), "test");
  
  // Cleanup: Remove test data
  cvcstate(key).reset();
}
```

### Step 3: Use Appropriate Assertions

Google Test provides various assertion macros:

- `EXPECT_EQ(a, b)` - Values are equal
- `EXPECT_NE(a, b)` - Values are not equal
- `EXPECT_TRUE(condition)` - Condition is true
- `EXPECT_FALSE(condition)` - Condition is false
- `EXPECT_LT/LE/GT/GE(a, b)` - Comparison assertions
- `EXPECT_DOUBLE_EQ(a, b)` - Floating point equality
- `ASSERT_*` versions - Stop test on failure

### Step 4: Rebuild and Test

```bash
cmake --build build --target app_test state_test
ctest --output-on-failure
```

## Common Testing Patterns

### Testing Exception Handling

```cpp
TEST(StateTest, InvalidOperation) {
  EXPECT_THROW({
    // Code that should throw
    cvcstate("test").data<int>(); // Empty data, should throw
  }, boost::bad_any_cast);
}
```

### Testing with Fixtures

For tests requiring common setup:

```cpp
class AppTestFixture : public ::testing::Test {
protected:
  void SetUp() override {
    // Common setup
    cvcapp.data("fixture.key", 42);
  }
  
  void TearDown() override {
    // Common cleanup
    cvcapp.data("fixture.key", boost::any());
  }
};

TEST_F(AppTestFixture, UseFixture) {
  EXPECT_EQ(cvcapp.data<int>("fixture.key"), 42);
}
```

### Parameterized Tests

For testing multiple inputs:

```cpp
class ValueTypeTest : public ::testing::TestWithParam<int> {};

TEST_P(ValueTypeTest, MultipleValues) {
  int value = GetParam();
  cvcstate("test.param").value(value);
  EXPECT_EQ(cvcstate("test.param").value<int>(), value);
  cvcstate("test.param").reset();
}

INSTANTIATE_TEST_SUITE_P(
  IntValues,
  ValueTypeTest,
  ::testing::Values(0, 1, 42, -100, 999)
);
```

## Code Coverage

### Overview

Trans-cvc includes integrated code coverage support using gcov/lcov. Coverage analysis shows which lines of code are executed by your tests, helping identify untested areas.

### Prerequisites

Install coverage tools (Ubuntu/Debian):

```bash
sudo apt-get install lcov
```

GCC or Clang compiler is required (MSVC not currently supported for coverage).

### Quick Start - Automated Script

The easiest way to generate coverage:

```bash
./generate_coverage.sh
```

This script automatically:
1. Configures the build with coverage enabled
2. Builds the project
3. Runs all tests
4. Generates HTML coverage report
5. Shows coverage summary

The report will be in `build-coverage/coverage_html/index.html`.

### Manual Coverage Generation

If you prefer manual control:

```bash
# Step 1: Configure with coverage enabled
cmake -B build-coverage -S . \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCVC_BUILD_TESTS=ON \
  -DCVC_ENABLE_COVERAGE=ON

# Step 2: Build the project
cmake --build build-coverage -j$(nproc)

# Step 3: Generate coverage report
cmake --build build-coverage --target coverage

# Step 4: View the report
xdg-open build-coverage/coverage_html/index.html
# Or use the convenience target
cmake --build build-coverage --target coverage-view
```

### Coverage Targets

- **`coverage`** - Runs tests and generates HTML report
- **`coverage-view`** - Opens the coverage report in a browser

### Understanding Coverage Reports

The HTML report shows:

- **Line Coverage**: Percentage of code lines executed
- **Function Coverage**: Percentage of functions called
- **Branch Coverage**: Percentage of conditional branches taken

Color coding:
- **Green**: Code executed by tests
- **Red**: Code not executed
- **Orange**: Partially executed (branches)

### Coverage Workflow

1. **Initial Coverage**: Run coverage on current tests
   ```bash
   ./generate_coverage.sh
   ```

2. **Identify Gaps**: Look for red (uncovered) sections in the report

3. **Add Tests**: Write tests for uncovered code

4. **Re-run Coverage**: Verify improvement
   ```bash
   ./generate_coverage.sh
   ```

5. **Iterate**: Repeat until target coverage reached

### Coverage Goals

Recommended coverage targets:
- **Core Libraries**: 80%+ line coverage
- **Critical Paths**: 100% coverage
- **Utility Functions**: 70%+ coverage
- **Overall Project**: 75%+ coverage

### Filtering Coverage

The coverage report automatically excludes:
- System headers (`/usr/*`)
- Test files (`*/test/*`, `*/tests/*`)
- Third-party dependencies (`*/_deps/*`, `*/googletest/*`)

To customize filtering, edit the `coverage` target in `CMakeLists.txt`.

### Coverage with Specific Tests

Run coverage for specific test suites:

```bash
cd build-coverage

# Clear previous data
lcov --directory . --zerocounters

# Run specific tests
./bin/app_test --gtest_filter=AppTest.Data*

# Capture and generate report
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/test/*' -o coverage_filtered.info
genhtml coverage_filtered.info -o coverage_html
```

### Coverage in CI/CD

Example GitHub Actions workflow:

```yaml
- name: Configure with Coverage
  run: cmake -B build -DCVC_ENABLE_COVERAGE=ON -DCVC_BUILD_TESTS=ON

- name: Build
  run: cmake --build build

- name: Generate Coverage
  run: cmake --build build --target coverage

- name: Upload Coverage to Codecov
  uses: codecov/codecov-action@v3
  with:
    files: ./build/coverage_filtered.info
```

### Troubleshooting Coverage

**Issue**: "lcov not found"
```bash
sudo apt-get install lcov
```

**Issue**: Coverage data is empty
- Ensure you built with `-DCVC_ENABLE_COVERAGE=ON`
- Verify tests actually ran (`ctest` output)
- Check that Debug build is used (coverage works best with Debug)

**Issue**: Low coverage on new code
- Make sure tests exercise the new code paths
- Check for untested error handling branches
- Add tests for edge cases

## Continuous Integration

### Running Tests in CI

Example GitHub Actions workflow snippet:

```yaml
- name: Configure CMake
  run: cmake -B build -DCVC_BUILD_TESTS=ON

- name: Build
  run: cmake --build build

- name: Test
  run: |
    cd build
    ctest --output-on-failure
```

### Test Coverage

To generate test coverage reports (requires gcov/lcov):

```bash
# Configure with coverage flags
cmake -B build -DCMAKE_CXX_FLAGS="--coverage" -DCVC_BUILD_TESTS=ON

# Build and run tests
cmake --build build
cd build && ctest

# Generate coverage report
lcov --capture --directory . --output-file coverage.info
lcov --remove coverage.info '/usr/*' --output-file coverage.info
lcov --list coverage.info
```

## Troubleshooting

### Tests Not Building

1. Ensure `CVC_BUILD_TESTS` is ON:
   ```bash
   cmake -B build -DCVC_BUILD_TESTS=ON
   ```

2. Check Google Test was fetched successfully:
   ```bash
   ls build/_deps/googletest-src
   ```

3. Verify network connectivity (required for FetchContent)

### Tests Failing

1. Run with verbose output:
   ```bash
   ctest --output-on-failure --verbose
   ```

2. Run individual test executable:
   ```bash
   ./build/bin/app_test --gtest_filter=*FailingTest*
   ```

3. Check for proper cleanup in previous tests

### Link Errors

If you see undefined references to Boost or other dependencies:

1. Verify dependencies are found:
   ```bash
   cmake -B build --debug-find
   ```

2. Check that test CMakeLists.txt links against `cvc` library

## Performance Considerations

- Tests use the real singleton instances of `app` and `state`
- Each test should clean up to avoid state leakage
- Tests run sequentially by default (CTest can parallelize test executables)
- Avoid expensive operations in tests; use mocks if needed

## Future Enhancements

Potential additions to the test suite:

1. **Concurrency Tests**: Multi-threaded access to `app` and `state`
2. **Performance Tests**: Benchmark critical operations
3. **Integration Tests**: Test interaction between components
4. **Fuzzing**: Random input testing for robustness
5. **Memory Tests**: Valgrind/AddressSanitizer integration
6. **Mock Objects**: Test in isolation from file I/O, etc.

## References

- [Google Test Documentation](https://google.github.io/googletest/)
- [CMake CTest Documentation](https://cmake.org/cmake/help/latest/manual/ctest.1.html)
- [Google Test Primer](https://google.github.io/googletest/primer.html)
- [Google Test Advanced Guide](https://google.github.io/googletest/advanced.html)

## Getting Help

- Check test output for specific assertion failures
- Review the test source code for expected behavior
- Consult Google Test documentation for assertion options
- Contact maintainers if tests consistently fail on your platform

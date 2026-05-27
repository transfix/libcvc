# Contributing to libcvc

Thank you for your interest in contributing to libcvc! This document provides guidelines for contributing to the project.

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help maintain code quality and documentation

## Getting Started

### Development Environment Setup

1. **Fork and clone the repository**
   ```bash
   git clone https://github.com/transfix/libcvc.git
   cd libcvc
   ```

2. **Install dependencies** (see README.md)

3. **Configure for development**
   ```bash
   cmake --preset dev
   # or manually:
   cmake -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
   ```

4. **Build**
   ```bash
   cmake --build build -j$(nproc)
   ```

## Development Workflow

### Branch Strategy

- `master` - Stable releases
- `develop` - Integration branch (if using git-flow)
- Feature branches: `feature/description`
- Bug fixes: `fix/issue-number-description`

### Making Changes

1. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes**
   - Write clear, commented code
   - Follow the existing code style
   - Update documentation as needed

3. **Test your changes**
   ```bash
   cmake --build build
   ./build/bin/libcvc  # Test the executable
   ```

4. **Commit your changes**
   ```bash
   git add .
   git commit -m "Brief description of changes"
   ```

### Commit Message Guidelines

Use clear, descriptive commit messages:

```
Short summary (50 chars or less)

More detailed explanation if needed. Wrap at 72 characters.
Explain what and why, not how (code shows how).

- Bullet points are fine
- Use present tense ("Add feature" not "Added feature")
- Reference issues: "Fixes #123" or "Relates to #456"
```

Examples:
```
Add bilateral filter support for 16-bit volumes

Update CMake to use modern target-based linking

Fix memory leak in geometry loader
Fixes #42
```

## Code Style Guidelines

### C++ Style

1. **Naming Conventions**
   ```cpp
   class MyClass { };           // PascalCase for classes
   void my_function();          // snake_case for functions
   int my_variable;             // snake_case for variables
   const int MY_CONSTANT = 42;  // UPPER_CASE for constants
   ```

2. **Formatting**
   - Use 2 spaces for indentation (matching existing code)
   - Opening braces on same line for functions/classes
   - Use meaningful variable names
   - Avoid overly long lines (prefer <100 chars)

3. **Modern C++ Practices**
   ```cpp
   // Use smart pointers instead of raw pointers
   std::unique_ptr<MyClass> obj = std::make_unique<MyClass>();
   
   // Use auto for complex types
   auto iter = container.begin();
   
   // Use nullptr instead of NULL
   MyClass* ptr = nullptr;
   
   // Use range-based for loops
   for (const auto& item : container) {
     // ...
   }
   ```

4. **Comments**
   ```cpp
   /**
    * Brief description of function
    * 
    * @param volume Input volume data
    * @param threshold Threshold value for processing
    * @return Processed geometry
    */
   geometry process_volume(const volume& volume, double threshold);
   ```

### CMake Style

1. **Commands in lowercase**
   ```cmake
   add_library(mylib source.cpp)  # Good
   ADD_LIBRARY(mylib source.cpp)  # Avoid
   ```

2. **Use target-based commands**
   ```cmake
   # Modern approach
   target_include_directories(mylib PUBLIC inc/)
   target_link_libraries(mylib PUBLIC Boost::filesystem)
   
   # Avoid global commands
   # include_directories(inc/)  # Don't do this
   ```

3. **Option naming**
   ```cmake
   option(CVC_FEATURE_NAME "Description of feature" OFF)
   ```

## Testing

### Current Status

⚠️ **Unit tests are not yet implemented.** This is a high-priority contribution area!

### Adding Tests (Future)

When the test framework is added:

1. Create tests in `test/` directory
2. Name test files `test_<feature>.cpp`
3. Ensure all tests pass before committing
4. Add tests for bug fixes to prevent regression

Example test structure (Catch2):
```cpp
#include <catch2/catch.hpp>
#include <cvc/volume/volume.h>

TEST_CASE("Volume loading", "[volume][io]") {
  SECTION("Load RAWIV file") {
    cvc::volume vol("test_data/sample.rawiv");
    REQUIRE(vol.is_valid());
    REQUIRE(vol.dimension().width() == 256);
  }
}
```

## Documentation

### Inline Documentation

- Document all public APIs with Doxygen-style comments
- Explain complex algorithms with inline comments
- Update header file comments when changing interfaces

### README and Guides

- Update README.md for user-facing changes
- Add examples for new features

## Pull Request Process

1. **Before submitting**
   - Ensure code compiles without warnings
   - Test on your platform
   - Update documentation
   - Check code style matches project conventions

2. **Create pull request**
   - Provide clear description of changes
   - Reference related issues
   - List any breaking changes
   - Note any new dependencies

3. **PR Review**
   - Address reviewer feedback
   - Keep PR focused (one feature/fix per PR)
   - Be patient and respectful

4. **After merge**
   - Delete your feature branch
   - Update your local repository

## Priority Contribution Areas

### High Priority

1. **Unit Tests** - Critical for code quality
   - Choose framework (Catch2 or Google Test)
   - Set up test infrastructure
   - Add tests for core functionality

2. **Documentation**
   - API documentation (Doxygen)
   - Usage examples
   - Tutorial guides

3. **Bug Fixes**
   - Address known issues
   - Fix memory leaks
   - Improve error handling

### Medium Priority

4. **Code Modernization**
   - Address TODO comments
   - Refactor duplicate code (e.g., VolMagick)
   - Implement CUDA kernels (infrastructure ready)

5. **New Features**
   - Additional file format support
   - New filtering algorithms
   - Performance optimizations

6. **Platform Support**
   - Test on various platforms
   - Fix platform-specific issues
   - Improve cross-platform compatibility

### Nice to Have

7. **CI/CD**
   - GitHub Actions workflows
   - Automated testing
   - Code coverage reports

8. **Packaging**
   - Debian/Ubuntu packages
   - Homebrew formula
   - vcpkg port

## Building Documentation

When Doxygen is set up:

```bash
cd build
cmake .. -DBUILD_DOCUMENTATION=ON
make doc
# Open build/doc/html/index.html
```

## Questions?

- Open an issue for bugs or feature requests
- Tag issues with appropriate labels
- Be as specific as possible

## License

By contributing, you agree that your contributions will be licensed under the same license as the project.

## Recognition

Contributors will be acknowledged in release notes and documentation.

Thank you for helping improve libcvc!

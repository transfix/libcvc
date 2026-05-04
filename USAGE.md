# Using libcvc from a CMake project

The libcvc binary archives (`libcvc-<version>-<os>-<arch>-<config>.{tar.gz,zip}`)
are **relocatable**: extract them anywhere and point CMake at the extracted
directory. No system installation, no `LD_LIBRARY_PATH` games.

## 1. Extract the archive

```bash
# Linux / macOS
tar xf libcvc-3.1.0-linux-x86_64-release.tar.gz   # → ./libcvc-3.1.0/
# or
unzip libcvc-3.1.0-macos-arm64-release.zip        # → ./libcvc-3.1.0/

# Windows (PowerShell)
Expand-Archive libcvc-3.1.0-windows-x86_64-release.zip
```

The archive lays out the standard GNU install tree:

```
libcvc-3.1.0/
├── bin/                   # libcvc.dll (Windows only)
├── include/cvc/           # Public headers
├── lib/
│   ├── libcvc.{so,dylib,a,lib}
│   └── cmake/cvc/
│       ├── cvcConfig.cmake
│       ├── cvcConfigVersion.cmake
│       └── cvcTargets.cmake
├── README.md
├── LICENSE
└── USAGE.md               # this file
```

## 2. Point CMake at the extracted tree

The most portable way is `CMAKE_PREFIX_PATH`:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/libcvc-3.1.0
```

You can also set `cvc_DIR` directly:

```bash
cmake -B build -Dcvc_DIR=/path/to/libcvc-3.1.0/lib/cmake/cvc
```

## 3. Use it from your `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_app CXX)

find_package(cvc REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE cvc::cvc)
```

That's it — the `cvc::cvc` imported target carries every `INTERFACE_*`
property (include dirs, link dependencies, compile defs) needed to build
against libcvc, and `find_dependency()` in `cvcConfig.cmake` will locate
Boost / HDF5 / FFTW / etc. on your system.

## 4. Picking matching configurations

The Debug archive (`...-debug.{tar.gz,zip}`) contains a debug build of
`libcvc` plus debug-info side files where applicable. On MSVC, mixing a
Release consumer with a Debug `libcvc.lib` is **not** supported — pick the
matching configuration. On Linux/macOS you can mix freely if you accept the
performance cost of debug runtime checks.

## 5. Minimal smoke-test project

```cpp
// main.cpp
#include <cvc/app.h>
#include <iostream>
int main() {
    cvc::app ctx;
    std::cout << "libcvc OK, " << ctx.threads().size() << " threads\n";
}
```

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/libcvc-3.1.0
cmake --build build
./build/my_app
```

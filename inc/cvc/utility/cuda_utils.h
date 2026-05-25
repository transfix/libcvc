/*
  Copyright 2007-2025 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __VOLMAGICK_CUDA_UTILS_H__
#define __VOLMAGICK_CUDA_UTILS_H__

#include <cvc/core/exception.h>
#include <cvc/core/namespace.h>
#include <cvc/core/types.h>

#ifdef CVC_USING_CUDA
#include <cuda_runtime.h>
#include <sstream>
#include <string>
#include <vector>
#endif

namespace cvc {
CVC_DEF_EXCEPTION(cuda_error);
CVC_DEF_EXCEPTION(cuda_not_available);

#ifdef CVC_USING_CUDA

// CUDA error checking macro
#define CUDA_CHECK(call)                                                                           \
  do {                                                                                             \
    cudaError_t err = call;                                                                        \
    if (err != cudaSuccess) {                                                                      \
      std::ostringstream oss;                                                                      \
      oss << "CUDA error at " << __FILE__ << ":" << __LINE__ << ": " << cudaGetErrorString(err);   \
      throw cuda_error(oss.str());                                                                 \
    }                                                                                              \
  } while (0)

/**
 * @struct gpu_device_info
 * @brief Information about a CUDA-capable GPU device
 */
struct gpu_device_info {
  int device_id;
  std::string name;
  size_t total_memory;
  size_t free_memory;
  int compute_capability_major;
  int compute_capability_minor;
  int multiprocessor_count;
  bool unified_memory_supported;
  bool peer_access_supported;

  gpu_device_info()
      : device_id(-1), total_memory(0), free_memory(0), compute_capability_major(0),
        compute_capability_minor(0), multiprocessor_count(0), unified_memory_supported(false),
        peer_access_supported(false) {}
};

/**
 * @class cuda_device_manager
 * @brief Manages CUDA device selection and provides device information
 *
 * Thread-safe singleton that manages GPU device selection per-thread.
 * Each thread can select its own GPU device independently.
 */
class cuda_device_manager {
public:
  // Get singleton instance
  static cuda_device_manager &instance();

  // Query available devices
  static bool cuda_available();
  static int device_count();
  static std::vector<gpu_device_info> get_device_info();
  static gpu_device_info get_device_info(int device_id);

  // Device selection (per-thread)
  static int get_current_device();
  static void set_current_device(int device_id);

  // Device properties
  static bool supports_unified_memory(int device_id = -1);
  static bool can_access_peer(int device_id, int peer_device_id);
  static void enable_peer_access(int device_id, int peer_device_id);

  // Memory info
  static size_t get_free_memory(int device_id = -1);
  static size_t get_total_memory(int device_id = -1);

private:
  cuda_device_manager();
  ~cuda_device_manager();

  // Non-copyable
  cuda_device_manager(const cuda_device_manager &) = delete;
  cuda_device_manager &operator=(const cuda_device_manager &) = delete;

  bool _cuda_available;
  int _device_count;
  std::vector<gpu_device_info> _devices;

  void initialize();
};

/**
 * @brief CUDA unified memory allocator for boost::multi_array
 *
 * Custom allocator that uses cudaMallocManaged for unified memory
 * allocation, enabling seamless CPU/GPU data access.
 */
template <typename T> class cuda_unified_allocator {
public:
  typedef T value_type;
  typedef T *pointer;
  typedef const T *const_pointer;
  typedef T &reference;
  typedef const T &const_reference;
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;

  template <typename U> struct rebind {
    typedef cuda_unified_allocator<U> other;
  };

  cuda_unified_allocator() throw() : _device_id(-1) {
    _device_id = cuda_device_manager::get_current_device();
  }

  cuda_unified_allocator(int device_id) throw() : _device_id(device_id) {}

  template <typename U>
  cuda_unified_allocator(const cuda_unified_allocator<U> &other) throw()
      : _device_id(other.device_id()) {}

  ~cuda_unified_allocator() throw() {}

  pointer allocate(size_type n, const void *hint = 0) {
    if (n == 0)
      return nullptr;

    // Set the device before allocation
    int old_device = -1;
    if (_device_id >= 0) {
      CUDA_CHECK(cudaGetDevice(&old_device));
      if (old_device != _device_id) {
        CUDA_CHECK(cudaSetDevice(_device_id));
      }
    }

    pointer result = nullptr;
    cudaError_t err = cudaMallocManaged(&result, n * sizeof(T));

    // Restore old device
    if (old_device >= 0 && old_device != _device_id) {
      cudaSetDevice(old_device);
    }

    if (err != cudaSuccess) {
      throw std::bad_alloc();
    }

    return result;
  }

  void deallocate(pointer p, size_type n) {
    if (p != nullptr) {
      cudaFree(p);
    }
  }

  void construct(pointer p, const_reference val) { new (p) T(val); }

  void destroy(pointer p) { p->~T(); }

  size_type max_size() const throw() { return std::numeric_limits<size_type>::max() / sizeof(T); }

  int device_id() const { return _device_id; }

private:
  int _device_id;
};

template <typename T1, typename T2>
bool operator==(const cuda_unified_allocator<T1> &a, const cuda_unified_allocator<T2> &b) {
  return a.device_id() == b.device_id();
}

template <typename T1, typename T2>
bool operator!=(const cuda_unified_allocator<T1> &a, const cuda_unified_allocator<T2> &b) {
  return !(a == b);
}

#else // !CVC_USING_CUDA

// Stub implementations when CUDA is not available
struct gpu_device_info {
  int device_id;
  std::string name;
  size_t total_memory;
  size_t free_memory;

  gpu_device_info() : device_id(-1), total_memory(0), free_memory(0) {}
};

class cuda_device_manager {
public:
  static bool cuda_available() { return false; }
  static int device_count() { return 0; }
  static std::vector<gpu_device_info> get_device_info() { return std::vector<gpu_device_info>(); }
  static gpu_device_info get_device_info(int) { return gpu_device_info(); }
  static int get_current_device() { return -1; }
  static void set_current_device(int) { throw cuda_not_available("CUDA not available"); }
  static bool supports_unified_memory(int = -1) { return false; }
  static bool can_access_peer(int, int) { return false; }
  static void enable_peer_access(int, int) { throw cuda_not_available("CUDA not available"); }
  static size_t get_free_memory(int = -1) { return 0; }
  static size_t get_total_memory(int = -1) { return 0; }
};

#endif // CVC_USING_CUDA

} // namespace cvc

#endif // __VOLMAGICK_CUDA_UTILS_H__

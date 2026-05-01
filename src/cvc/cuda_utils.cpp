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

#include <cvc/cuda_utils.h>

#ifdef CVC_USING_CUDA
#include <algorithm>
#include <cuda_runtime.h>
#include <sstream>
#endif

namespace CVC_NAMESPACE {

#ifdef CVC_USING_CUDA

cuda_device_manager::cuda_device_manager() : _cuda_available(false), _device_count(0) {
  initialize();
}

cuda_device_manager::~cuda_device_manager() {
  // Cleanup - reset all devices
  if (_cuda_available) {
    cudaDeviceReset();
  }
}

cuda_device_manager &cuda_device_manager::instance() {
  static cuda_device_manager mgr;
  return mgr;
}

void cuda_device_manager::initialize() {
  cudaError_t err = cudaGetDeviceCount(&_device_count);
  if (err != cudaSuccess || _device_count == 0) {
    _cuda_available = false;
    _device_count = 0;
    return;
  }

  _cuda_available = true;
  _devices.resize(_device_count);

  // Query each device for information
  for (int i = 0; i < _device_count; ++i) {
    cudaDeviceProp prop;
    if (cudaGetDeviceProperties(&prop, i) == cudaSuccess) {
      _devices[i].device_id = i;
      _devices[i].name = prop.name;
      _devices[i].total_memory = prop.totalGlobalMem;
      _devices[i].compute_capability_major = prop.major;
      _devices[i].compute_capability_minor = prop.minor;
      _devices[i].multiprocessor_count = prop.multiProcessorCount;
      _devices[i].unified_memory_supported =
          (prop.major >= 3); // Unified memory requires compute capability 3.0+

      // Get free memory
      size_t free_mem, total_mem;
      int old_device;
      cudaGetDevice(&old_device);
      cudaSetDevice(i);
      if (cudaMemGetInfo(&free_mem, &total_mem) == cudaSuccess) {
        _devices[i].free_memory = free_mem;
      }
      cudaSetDevice(old_device);
    }
  }
}

bool cuda_device_manager::cuda_available() { return instance()._cuda_available; }

int cuda_device_manager::device_count() { return instance()._device_count; }

std::vector<gpu_device_info> cuda_device_manager::get_device_info() { return instance()._devices; }

gpu_device_info cuda_device_manager::get_device_info(int device_id) {
  auto &inst = instance();
  if (device_id < 0 || device_id >= inst._device_count) {
    throw cuda_error("Invalid device ID");
  }
  return inst._devices[device_id];
}

int cuda_device_manager::get_current_device() {
  if (!cuda_available()) {
    return -1;
  }

  int device = -1;
  cudaError_t err = cudaGetDevice(&device);
  if (err != cudaSuccess) {
    return -1;
  }
  return device;
}

void cuda_device_manager::set_current_device(int device_id) {
  if (!cuda_available()) {
    throw cuda_not_available("CUDA not available");
  }

  auto &inst = instance();
  if (device_id < 0 || device_id >= inst._device_count) {
    std::ostringstream oss;
    oss << "Invalid device ID: " << device_id << " (available: 0-" << (inst._device_count - 1)
        << ")";
    throw cuda_error(oss.str());
  }

  CUDA_CHECK(cudaSetDevice(device_id));
}

bool cuda_device_manager::supports_unified_memory(int device_id) {
  if (!cuda_available()) {
    return false;
  }

  if (device_id < 0) {
    device_id = get_current_device();
  }

  auto &inst = instance();
  if (device_id < 0 || device_id >= inst._device_count) {
    return false;
  }

  return inst._devices[device_id].unified_memory_supported;
}

bool cuda_device_manager::can_access_peer(int device_id, int peer_device_id) {
  if (!cuda_available()) {
    return false;
  }

  auto &inst = instance();
  if (device_id < 0 || device_id >= inst._device_count || peer_device_id < 0 ||
      peer_device_id >= inst._device_count) {
    return false;
  }

  int can_access = 0;
  cudaError_t err = cudaDeviceCanAccessPeer(&can_access, device_id, peer_device_id);
  return (err == cudaSuccess && can_access != 0);
}

void cuda_device_manager::enable_peer_access(int device_id, int peer_device_id) {
  if (!cuda_available()) {
    throw cuda_not_available("CUDA not available");
  }

  if (!can_access_peer(device_id, peer_device_id)) {
    std::ostringstream oss;
    oss << "Cannot enable peer access from device " << device_id << " to device " << peer_device_id;
    throw cuda_error(oss.str());
  }

  int old_device = get_current_device();
  set_current_device(device_id);

  cudaError_t err = cudaDeviceEnablePeerAccess(peer_device_id, 0);
  if (err != cudaSuccess && err != cudaErrorPeerAccessAlreadyEnabled) {
    set_current_device(old_device);
    CUDA_CHECK(err);
  }

  set_current_device(old_device);
}

size_t cuda_device_manager::get_free_memory(int device_id) {
  if (!cuda_available()) {
    return 0;
  }

  if (device_id < 0) {
    device_id = get_current_device();
  }

  auto &inst = instance();
  if (device_id < 0 || device_id >= inst._device_count) {
    return 0;
  }

  int old_device = get_current_device();
  set_current_device(device_id);

  size_t free_mem = 0, total_mem = 0;
  cudaMemGetInfo(&free_mem, &total_mem);

  set_current_device(old_device);
  return free_mem;
}

size_t cuda_device_manager::get_total_memory(int device_id) {
  if (!cuda_available()) {
    return 0;
  }

  if (device_id < 0) {
    device_id = get_current_device();
  }

  auto &inst = instance();
  if (device_id < 0 || device_id >= inst._device_count) {
    return 0;
  }

  return inst._devices[device_id].total_memory;
}

#endif // CVC_USING_CUDA

} // namespace CVC_NAMESPACE

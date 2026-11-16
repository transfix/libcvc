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

#include <cvc/utility/cuda_utils.h>

#ifdef CVC_USING_CUDA
#include <algorithm>
#include <cuda_runtime.h>
#include <sstream>
#endif

namespace cvc {

#ifdef CVC_USING_CUDA

namespace {

// The devices the CUDA runtime sees, asked each time rather than cached for the process: the
// runtime keeps the answer itself, so the call is cheap, and nothing here is shared between apps.
int count_devices() {
  int n = 0;
  if (cudaGetDeviceCount(&n) != cudaSuccess)
    return 0;
  return n;
}

bool valid_device(int device_id) { return device_id >= 0 && device_id < count_devices(); }

// Device I's description, or a default one (device_id -1) when the runtime cannot describe it.
gpu_device_info describe_device(int i) {
  gpu_device_info info;
  cudaDeviceProp prop;
  if (cudaGetDeviceProperties(&prop, i) != cudaSuccess)
    return info;
  info.device_id = i;
  info.name = prop.name;
  info.total_memory = prop.totalGlobalMem;
  info.compute_capability_major = prop.major;
  info.compute_capability_minor = prop.minor;
  info.multiprocessor_count = prop.multiProcessorCount;
  info.unified_memory_supported =
      (prop.major >= 3); // Unified memory requires compute capability 3.0+

  // Free memory is the device's own context's answer, so ask it on that device.
  int old_device = 0;
  if (cudaGetDevice(&old_device) == cudaSuccess && cudaSetDevice(i) == cudaSuccess) {
    size_t free_mem = 0, total_mem = 0;
    if (cudaMemGetInfo(&free_mem, &total_mem) == cudaSuccess)
      info.free_memory = free_mem;
    cudaSetDevice(old_device);
  }
  return info;
}

} // namespace

bool cuda_device_manager::cuda_available() { return count_devices() > 0; }

int cuda_device_manager::device_count() { return count_devices(); }

std::vector<gpu_device_info> cuda_device_manager::get_device_info() {
  std::vector<gpu_device_info> devices;
  const int n = count_devices();
  devices.reserve(n);
  for (int i = 0; i < n; ++i)
    devices.push_back(describe_device(i));
  return devices;
}

gpu_device_info cuda_device_manager::get_device_info(int device_id) {
  if (!valid_device(device_id)) {
    throw cuda_error("Invalid device ID");
  }
  return describe_device(device_id);
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

  const int n = count_devices();
  if (device_id < 0 || device_id >= n) {
    std::ostringstream oss;
    oss << "Invalid device ID: " << device_id << " (available: 0-" << (n - 1) << ")";
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

  if (!valid_device(device_id)) {
    return false;
  }

  // Unified memory requires compute capability 3.0+
  int major = 0;
  return cudaDeviceGetAttribute(&major, cudaDevAttrComputeCapabilityMajor, device_id) ==
             cudaSuccess &&
         major >= 3;
}

bool cuda_device_manager::can_access_peer(int device_id, int peer_device_id) {
  if (!cuda_available()) {
    return false;
  }

  if (!valid_device(device_id) || !valid_device(peer_device_id)) {
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

  if (!valid_device(device_id)) {
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

  if (!valid_device(device_id)) {
    return 0;
  }

  cudaDeviceProp prop;
  if (cudaGetDeviceProperties(&prop, device_id) != cudaSuccess) {
    return 0;
  }
  return prop.totalGlobalMem;
}

#endif // CVC_USING_CUDA

} // namespace cvc

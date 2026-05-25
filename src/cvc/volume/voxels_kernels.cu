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
#include <cvc/core/types.h>

namespace cvc
{

// Device function for trilinear interpolation (GPU version of getTriVal)
__device__ inline double getTriVal_device(double val[8], double x, double y, double z,
                                          double resX, double resY, double resZ)
{
  double x_ratio, y_ratio, z_ratio;
  double temp1, temp2, temp3, temp4, temp5, temp6;
  
  x_ratio = x / resX;
  y_ratio = y / resY;
  z_ratio = z / resZ;
  
  if (x_ratio == 1.0) x_ratio = 0.0;
  if (y_ratio == 1.0) y_ratio = 0.0;
  if (z_ratio == 1.0) z_ratio = 0.0;
  
  temp1 = val[0] + (val[1] - val[0]) * x_ratio;
  temp2 = val[4] + (val[5] - val[4]) * x_ratio;
  temp3 = val[2] + (val[3] - val[2]) * x_ratio;
  temp4 = val[6] + (val[7] - val[6]) * x_ratio;
  temp5 = temp1 + (temp3 - temp1) * y_ratio;
  temp6 = temp2 + (temp4 - temp2) * y_ratio;
  
  return temp5 + (temp6 - temp5) * z_ratio;
}

// Template kernel for different voxel types
template<typename T>
__global__ void resize_trilinear_kernel(
    const T* src_data,
    T* dst_data,
    uint64 src_x, uint64 src_y, uint64 src_z,
    uint64 dst_x, uint64 dst_y, uint64 dst_z,
    double inSpaceX, double inSpaceY, double inSpaceZ)
{
  // 3D grid indexing
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  if (i >= dst_x || j >= dst_y || k >= dst_z) return;
  
  // Calculate position in source volume
  double x = double(i) * inSpaceX;
  double y = double(j) * inSpaceY;
  double z = double(k) * inSpaceZ;
  
  uint64 resXIndex = uint64(x);
  uint64 resYIndex = uint64(y);
  uint64 resZIndex = uint64(z);
  
  double xPosition = x - double(resXIndex);
  double yPosition = y - double(resYIndex);
  double zPosition = z - double(resZIndex);
  
  // Find indices for eight corner voxels
  uint64 ValIndex[8];
  ValIndex[0] = resZIndex * src_x * src_y + resYIndex * src_x + resXIndex;
  ValIndex[1] = ValIndex[0] + 1;
  ValIndex[2] = resZIndex * src_x * src_y + (resYIndex + 1) * src_x + resXIndex;
  ValIndex[3] = ValIndex[2] + 1;
  ValIndex[4] = (resZIndex + 1) * src_x * src_y + resYIndex * src_x + resXIndex;
  ValIndex[5] = ValIndex[4] + 1;
  ValIndex[6] = (resZIndex + 1) * src_x * src_y + (resYIndex + 1) * src_x + resXIndex;
  ValIndex[7] = ValIndex[6] + 1;
  
  // Handle boundary conditions
  if (resXIndex >= src_x - 1) {
    ValIndex[1] = ValIndex[0];
    ValIndex[3] = ValIndex[2];
    ValIndex[5] = ValIndex[4];
    ValIndex[7] = ValIndex[6];
  }
  if (resYIndex >= src_y - 1) {
    ValIndex[2] = ValIndex[0];
    ValIndex[3] = ValIndex[1];
    ValIndex[6] = ValIndex[4];
    ValIndex[7] = ValIndex[5];
  }
  if (resZIndex >= src_z - 1) {
    ValIndex[4] = ValIndex[0];
    ValIndex[5] = ValIndex[1];
    ValIndex[6] = ValIndex[2];
    ValIndex[7] = ValIndex[3];
  }
  
  // Read eight corner values
  double val[8];
  for (int idx = 0; idx < 8; idx++) {
    val[idx] = double(src_data[ValIndex[idx]]);
  }
  
  // Perform trilinear interpolation
  double result = getTriVal_device(val, xPosition, yPosition, zPosition, 1.0, 1.0, 1.0);
  
  // Write result
  uint64 dst_idx = k * dst_x * dst_y + j * dst_x + i;
  dst_data[dst_idx] = T(result);
}

// Explicit template instantiations for all voxel types
template __global__ void resize_trilinear_kernel<unsigned char>(
    const unsigned char*, unsigned char*, uint64, uint64, uint64,
    uint64, uint64, uint64, double, double, double);

template __global__ void resize_trilinear_kernel<unsigned short>(
    const unsigned short*, unsigned short*, uint64, uint64, uint64,
    uint64, uint64, uint64, double, double, double);

template __global__ void resize_trilinear_kernel<unsigned int>(
    const unsigned int*, unsigned int*, uint64, uint64, uint64,
    uint64, uint64, uint64, double, double, double);

template __global__ void resize_trilinear_kernel<float>(
    const float*, float*, uint64, uint64, uint64,
    uint64, uint64, uint64, double, double, double);

template __global__ void resize_trilinear_kernel<double>(
    const double*, double*, uint64, uint64, uint64,
    uint64, uint64, uint64, double, double, double);

template __global__ void resize_trilinear_kernel<uint64>(
    const uint64*, uint64*, uint64, uint64, uint64,
    uint64, uint64, uint64, double, double, double);


// Host function to launch kernel with appropriate type
extern "C" void cuda_resize_trilinear(
    void* src_data,
    void* dst_data,
    uint64 src_x, uint64 src_y, uint64 src_z,
    uint64 dst_x, uint64 dst_y, uint64 dst_z,
    double inSpaceX, double inSpaceY, double inSpaceZ,
    data_type voxel_type)
{
  // Configure kernel launch parameters
  dim3 blockSize(8, 8, 8);  // 512 threads per block
  dim3 gridSize(
      (dst_x + blockSize.x - 1) / blockSize.x,
      (dst_y + blockSize.y - 1) / blockSize.y,
      (dst_z + blockSize.z - 1) / blockSize.z
  );
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      resize_trilinear_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(src_data),
          static_cast<unsigned char*>(dst_data),
          src_x, src_y, src_z, dst_x, dst_y, dst_z,
          inSpaceX, inSpaceY, inSpaceZ);
      break;
      
    case UShort:
      resize_trilinear_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(src_data),
          static_cast<unsigned short*>(dst_data),
          src_x, src_y, src_z, dst_x, dst_y, dst_z,
          inSpaceX, inSpaceY, inSpaceZ);
      break;
      
    case UInt:
      resize_trilinear_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(src_data),
          static_cast<unsigned int*>(dst_data),
          src_x, src_y, src_z, dst_x, dst_y, dst_z,
          inSpaceX, inSpaceY, inSpaceZ);
      break;
      
    case Float:
      resize_trilinear_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(src_data),
          static_cast<float*>(dst_data),
          src_x, src_y, src_z, dst_x, dst_y, dst_z,
          inSpaceX, inSpaceY, inSpaceZ);
      break;
      
    case Double:
      resize_trilinear_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(src_data),
          static_cast<double*>(dst_data),
          src_x, src_y, src_z, dst_x, dst_y, dst_z,
          inSpaceX, inSpaceY, inSpaceZ);
      break;
      
    case UInt64:
      resize_trilinear_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(src_data),
          static_cast<uint64*>(dst_data),
          src_x, src_y, src_z, dst_x, dst_y, dst_z,
          inSpaceX, inSpaceY, inSpaceZ);
      break;
  }
  
  // Check for kernel launch errors
  CUDA_CHECK(cudaGetLastError());
  
  // Synchronize to ensure completion
  CUDA_CHECK(cudaDeviceSynchronize());
}

// Template kernel for bounding box aware resize
template<typename T>
__global__ void resize_bbox_trilinear_kernel(
    const T* src_data,
    T* dst_data,
    uint64 dim_x, uint64 dim_y, uint64 dim_z,
    double offset_x, double offset_y, double offset_z,
    double scale_x, double scale_y, double scale_z)
{
  // 3D grid indexing
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  if (i >= dim_x || j >= dim_y || k >= dim_z) return;
  
  // Calculate position in source volume using bbox transformation
  double x = offset_x + double(i) * scale_x;
  double y = offset_y + double(j) * scale_y;
  double z = offset_z + double(k) * scale_z;
  
  uint64 resXIndex = uint64(x);
  uint64 resYIndex = uint64(y);
  uint64 resZIndex = uint64(z);
  
  double xPosition = x - double(resXIndex);
  double yPosition = y - double(resYIndex);
  double zPosition = z - double(resZIndex);
  
  // Find indices for eight corner voxels
  uint64 ValIndex[8];
  ValIndex[0] = resZIndex * dim_x * dim_y + resYIndex * dim_x + resXIndex;
  ValIndex[1] = ValIndex[0] + 1;
  ValIndex[2] = resZIndex * dim_x * dim_y + (resYIndex + 1) * dim_x + resXIndex;
  ValIndex[3] = ValIndex[2] + 1;
  ValIndex[4] = (resZIndex + 1) * dim_x * dim_y + resYIndex * dim_x + resXIndex;
  ValIndex[5] = ValIndex[4] + 1;
  ValIndex[6] = (resZIndex + 1) * dim_x * dim_y + (resYIndex + 1) * dim_x + resXIndex;
  ValIndex[7] = ValIndex[6] + 1;
  
  // Handle boundary conditions
  if (resXIndex >= dim_x - 1) {
    ValIndex[1] = ValIndex[0];
    ValIndex[3] = ValIndex[2];
    ValIndex[5] = ValIndex[4];
    ValIndex[7] = ValIndex[6];
  }
  if (resYIndex >= dim_y - 1) {
    ValIndex[2] = ValIndex[0];
    ValIndex[3] = ValIndex[1];
    ValIndex[6] = ValIndex[4];
    ValIndex[7] = ValIndex[5];
  }
  if (resZIndex >= dim_z - 1) {
    ValIndex[4] = ValIndex[0];
    ValIndex[5] = ValIndex[1];
    ValIndex[6] = ValIndex[2];
    ValIndex[7] = ValIndex[3];
  }
  
  // Read eight corner values
  double val[8];
  for (int idx = 0; idx < 8; idx++) {
    val[idx] = double(src_data[ValIndex[idx]]);
  }
  
  // Perform trilinear interpolation
  double result = getTriVal_device(val, xPosition, yPosition, zPosition, 1.0, 1.0, 1.0);
  
  // Write result
  uint64 dst_idx = k * dim_x * dim_y + j * dim_x + i;
  dst_data[dst_idx] = T(result);
}

// Host function to launch bbox-aware resize kernel
extern "C" void cuda_resize_bbox_trilinear(
    void* src_data,
    void* dst_data,
    uint64 dim_x, uint64 dim_y, uint64 dim_z,
    double offset_x, double offset_y, double offset_z,
    double scale_x, double scale_y, double scale_z,
    data_type voxel_type)
{
  // Configure kernel launch parameters
  dim3 blockSize(8, 8, 8);  // 512 threads per block
  dim3 gridSize(
      (dim_x + blockSize.x - 1) / blockSize.x,
      (dim_y + blockSize.y - 1) / blockSize.y,
      (dim_z + blockSize.z - 1) / blockSize.z
  );
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      resize_bbox_trilinear_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(src_data),
          static_cast<unsigned char*>(dst_data),
          dim_x, dim_y, dim_z,
          offset_x, offset_y, offset_z,
          scale_x, scale_y, scale_z);
      break;
      
    case UShort:
      resize_bbox_trilinear_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(src_data),
          static_cast<unsigned short*>(dst_data),
          dim_x, dim_y, dim_z,
          offset_x, offset_y, offset_z,
          scale_x, scale_y, scale_z);
      break;
      
    case UInt:
      resize_bbox_trilinear_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(src_data),
          static_cast<unsigned int*>(dst_data),
          dim_x, dim_y, dim_z,
          offset_x, offset_y, offset_z,
          scale_x, scale_y, scale_z);
      break;
      
    case Float:
      resize_bbox_trilinear_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(src_data),
          static_cast<float*>(dst_data),
          dim_x, dim_y, dim_z,
          offset_x, offset_y, offset_z,
          scale_x, scale_y, scale_z);
      break;
      
    case Double:
      resize_bbox_trilinear_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(src_data),
          static_cast<double*>(dst_data),
          dim_x, dim_y, dim_z,
          offset_x, offset_y, offset_z,
          scale_x, scale_y, scale_z);
      break;
      
    case UInt64:
      resize_bbox_trilinear_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(src_data),
          static_cast<uint64*>(dst_data),
          dim_x, dim_y, dim_z,
          offset_x, offset_y, offset_z,
          scale_x, scale_y, scale_z);
      break;
  }
  
  // Check for kernel launch errors
  CUDA_CHECK(cudaGetLastError());
  
  // Synchronize to ensure completion
  CUDA_CHECK(cudaDeviceSynchronize());
}

// =============================================================================
// Min/Max Reduction Kernels
// =============================================================================

// Kernel to compute partial min/max for a block of voxels
template<typename T>
__global__ void minmax_reduction_kernel(
    const T* data,
    double* block_mins,
    double* block_maxs,
    uint64 off_x, uint64 off_y, uint64 off_z,
    uint64 dim_x, uint64 dim_y, uint64 dim_z,
    uint64 vol_x, uint64 vol_y, uint64 vol_z)
{
  // Shared memory for block-level reduction
  extern __shared__ double sdata[];
  double* s_min = sdata;
  double* s_max = &sdata[blockDim.x * blockDim.y * blockDim.z];
  
  // 3D grid indexing within the subvolume
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  // Thread index within block
  int tid = threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
  
  // Initialize with extreme values
  double local_min = INFINITY;
  double local_max = -INFINITY;
  
  // Compute local min/max if within bounds
  if (i < dim_x && j < dim_y && k < dim_z) {
    uint64 global_idx = (k + off_z) * vol_x * vol_y + (j + off_y) * vol_x + (i + off_x);
    double val = double(data[global_idx]);
    local_min = val;
    local_max = val;
  }
  
  // Store to shared memory
  s_min[tid] = local_min;
  s_max[tid] = local_max;
  __syncthreads();
  
  // Block-level reduction in shared memory
  for (int stride = (blockDim.x * blockDim.y * blockDim.z) / 2; stride > 0; stride >>= 1) {
    if (tid < stride) {
      s_min[tid] = fmin(s_min[tid], s_min[tid + stride]);
      s_max[tid] = fmax(s_max[tid], s_max[tid + stride]);
    }
    __syncthreads();
  }
  
  // Thread 0 writes block result to global memory
  if (tid == 0) {
    int block_id = blockIdx.z * gridDim.x * gridDim.y + blockIdx.y * gridDim.x + blockIdx.x;
    block_mins[block_id] = s_min[0];
    block_maxs[block_id] = s_max[0];
  }
}

// Host function to compute min over a subvolume
extern "C" double cuda_compute_min(
    void* data,
    uint64 off_x, uint64 off_y, uint64 off_z,
    uint64 dim_x, uint64 dim_y, uint64 dim_z,
    uint64 vol_x, uint64 vol_y, uint64 vol_z,
    data_type voxel_type)
{
  // Use 8x8x8 thread blocks
  dim3 blockSize(8, 8, 8);
  dim3 gridSize(
      (dim_x + blockSize.x - 1) / blockSize.x,
      (dim_y + blockSize.y - 1) / blockSize.y,
      (dim_z + blockSize.z - 1) / blockSize.z
  );
  
  int num_blocks = gridSize.x * gridSize.y * gridSize.z;
  size_t shared_mem_size = 2 * blockSize.x * blockSize.y * blockSize.z * sizeof(double);
  
  // Allocate device memory for block results
  double *d_block_mins, *d_block_maxs;
  CUDA_CHECK(cudaMalloc(&d_block_mins, num_blocks * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_block_maxs, num_blocks * sizeof(double)));
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      minmax_reduction_kernel<unsigned char><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const unsigned char*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case UShort:
      minmax_reduction_kernel<unsigned short><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const unsigned short*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case UInt:
      minmax_reduction_kernel<unsigned int><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const unsigned int*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case Float:
      minmax_reduction_kernel<float><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const float*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case Double:
      minmax_reduction_kernel<double><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const double*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case UInt64:
      minmax_reduction_kernel<uint64><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const uint64*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
  }
  
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  
  // Copy block results back to host
  std::vector<double> h_block_mins(num_blocks);
  CUDA_CHECK(cudaMemcpy(h_block_mins.data(), d_block_mins, 
                        num_blocks * sizeof(double), cudaMemcpyDeviceToHost));
  
  // Free device memory
  CUDA_CHECK(cudaFree(d_block_mins));
  CUDA_CHECK(cudaFree(d_block_maxs));
  
  // Final reduction on host
  double result = h_block_mins[0];
  for (int i = 1; i < num_blocks; i++) {
    if (h_block_mins[i] < result) {
      result = h_block_mins[i];
    }
  }
  
  return result;
}

// Host function to compute max over a subvolume
extern "C" double cuda_compute_max(
    void* data,
    uint64 off_x, uint64 off_y, uint64 off_z,
    uint64 dim_x, uint64 dim_y, uint64 dim_z,
    uint64 vol_x, uint64 vol_y, uint64 vol_z,
    data_type voxel_type)
{
  // Use 8x8x8 thread blocks
  dim3 blockSize(8, 8, 8);
  dim3 gridSize(
      (dim_x + blockSize.x - 1) / blockSize.x,
      (dim_y + blockSize.y - 1) / blockSize.y,
      (dim_z + blockSize.z - 1) / blockSize.z
  );
  
  int num_blocks = gridSize.x * gridSize.y * gridSize.z;
  size_t shared_mem_size = 2 * blockSize.x * blockSize.y * blockSize.z * sizeof(double);
  
  // Allocate device memory for block results
  double *d_block_mins, *d_block_maxs;
  CUDA_CHECK(cudaMalloc(&d_block_mins, num_blocks * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_block_maxs, num_blocks * sizeof(double)));
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      minmax_reduction_kernel<unsigned char><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const unsigned char*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case UShort:
      minmax_reduction_kernel<unsigned short><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const unsigned short*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case UInt:
      minmax_reduction_kernel<unsigned int><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const unsigned int*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case Float:
      minmax_reduction_kernel<float><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const float*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case Double:
      minmax_reduction_kernel<double><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const double*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
    case UInt64:
      minmax_reduction_kernel<uint64><<<gridSize, blockSize, shared_mem_size>>>(
          static_cast<const uint64*>(data),
          d_block_mins, d_block_maxs,
          off_x, off_y, off_z, dim_x, dim_y, dim_z, vol_x, vol_y, vol_z);
      break;
  }
  
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  
  // Copy block results back to host
  std::vector<double> h_block_maxs(num_blocks);
  CUDA_CHECK(cudaMemcpy(h_block_maxs.data(), d_block_maxs, 
                        num_blocks * sizeof(double), cudaMemcpyDeviceToHost));
  
  // Free device memory
  CUDA_CHECK(cudaFree(d_block_mins));
  CUDA_CHECK(cudaFree(d_block_maxs));
  
  // Final reduction on host
  double result = h_block_maxs[0];
  for (int i = 1; i < num_blocks; i++) {
    if (h_block_maxs[i] > result) {
      result = h_block_maxs[i];
    }
  }
  
  return result;
}

// Anisotropic diffusion kernel - processes one slice at a time
template<typename T>
__global__ void anisotropic_diffusion_slice_kernel(
    const T* src_data,
    T* dst_data,
    uint64 xdim, uint64 ydim,
    uint64 slice_idx, uint64 zdim,
    double K_para, double Lambda_para)
{
  // 2D grid indexing for a slice
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  
  if (i >= xdim || j >= ydim) return;
  
  uint64 k = slice_idx;
  
  // Calculate deltas for all 6 neighbors
  double delta_n, delta_s, delta_e, delta_w, delta_u, delta_d;
  
  // Current voxel value
  uint64 current_idx = k * xdim * ydim + j * xdim + i;
  double current_val = double(src_data[current_idx]);
  
  // South neighbor (j+1)
  if (j < ydim - 1) {
    uint64 idx = k * xdim * ydim + (j + 1) * xdim + i;
    delta_s = double(src_data[idx]) - current_val;
  } else {
    delta_s = 0.0;
  }
  
  // North neighbor (j-1)
  if (j > 0) {
    uint64 idx = k * xdim * ydim + (j - 1) * xdim + i;
    delta_n = double(src_data[idx]) - current_val;
  } else {
    delta_n = 0.0;
  }
  
  // East neighbor (i+1)
  if (i < xdim - 1) {
    uint64 idx = k * xdim * ydim + j * xdim + (i + 1);
    delta_e = double(src_data[idx]) - current_val;
  } else {
    delta_e = 0.0;
  }
  
  // West neighbor (i-1)
  if (i > 0) {
    uint64 idx = k * xdim * ydim + j * xdim + (i - 1);
    delta_w = double(src_data[idx]) - current_val;
  } else {
    delta_w = 0.0;
  }
  
  // Up neighbor (k+1)
  if (k < zdim - 1) {
    uint64 idx = (k + 1) * xdim * ydim + j * xdim + i;
    delta_u = double(src_data[idx]) - current_val;
  } else {
    delta_u = 0.0;
  }
  
  // Down neighbor (k-1)
  if (k > 0) {
    uint64 idx = (k - 1) * xdim * ydim + j * xdim + i;
    delta_d = double(src_data[idx]) - current_val;
  } else {
    delta_d = 0.0;
  }
  
  // Calculate conductance coefficients
  double K_squared = K_para * K_para;
  double cn = 1.0 / (1.0 + (delta_n * delta_n) / K_squared);
  double cs = 1.0 / (1.0 + (delta_s * delta_s) / K_squared);
  double ce = 1.0 / (1.0 + (delta_e * delta_e) / K_squared);
  double cw = 1.0 / (1.0 + (delta_w * delta_w) / K_squared);
  double cu = 1.0 / (1.0 + (delta_u * delta_u) / K_squared);
  double cd = 1.0 / (1.0 + (delta_d * delta_d) / K_squared);
  
  // Apply anisotropic diffusion update
  double new_val = current_val + Lambda_para * 
                   (cn * delta_n + cs * delta_s + ce * delta_e + 
                    cw * delta_w + cu * delta_u + cd * delta_d);
  
  // Write result
  dst_data[current_idx] = T(new_val);
}

// Explicit template instantiations for all voxel types
template __global__ void anisotropic_diffusion_slice_kernel<unsigned char>(
    const unsigned char*, unsigned char*, uint64, uint64, uint64, uint64, double, double);

template __global__ void anisotropic_diffusion_slice_kernel<unsigned short>(
    const unsigned short*, unsigned short*, uint64, uint64, uint64, uint64, double, double);

template __global__ void anisotropic_diffusion_slice_kernel<unsigned int>(
    const unsigned int*, unsigned int*, uint64, uint64, uint64, uint64, double, double);

template __global__ void anisotropic_diffusion_slice_kernel<float>(
    const float*, float*, uint64, uint64, uint64, uint64, double, double);

template __global__ void anisotropic_diffusion_slice_kernel<double>(
    const double*, double*, uint64, uint64, uint64, uint64, double, double);

template __global__ void anisotropic_diffusion_slice_kernel<uint64>(
    const uint64*, uint64*, uint64, uint64, uint64, uint64, double, double);

// Host function to launch anisotropic diffusion kernel for a single slice
extern "C" void cuda_anisotropic_diffusion_slice(
    void* src_data,
    void* dst_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    uint64 slice_idx,
    double K_para, double Lambda_para,
    data_type voxel_type)
{
  // Configure kernel launch parameters for 2D slice
  dim3 blockSize(16, 16);  // 256 threads per block
  dim3 gridSize(
      (xdim + blockSize.x - 1) / blockSize.x,
      (ydim + blockSize.y - 1) / blockSize.y
  );
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      anisotropic_diffusion_slice_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(src_data),
          static_cast<unsigned char*>(dst_data),
          xdim, ydim, slice_idx, zdim, K_para, Lambda_para);
      break;
      
    case UShort:
      anisotropic_diffusion_slice_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(src_data),
          static_cast<unsigned short*>(dst_data),
          xdim, ydim, slice_idx, zdim, K_para, Lambda_para);
      break;
      
    case UInt:
      anisotropic_diffusion_slice_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(src_data),
          static_cast<unsigned int*>(dst_data),
          xdim, ydim, slice_idx, zdim, K_para, Lambda_para);
      break;
      
    case Float:
      anisotropic_diffusion_slice_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(src_data),
          static_cast<float*>(dst_data),
          xdim, ydim, slice_idx, zdim, K_para, Lambda_para);
      break;
      
    case Double:
      anisotropic_diffusion_slice_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(src_data),
          static_cast<double*>(dst_data),
          xdim, ydim, slice_idx, zdim, K_para, Lambda_para);
      break;
      
    case UInt64:
      anisotropic_diffusion_slice_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(src_data),
          static_cast<uint64*>(dst_data),
          xdim, ydim, slice_idx, zdim, K_para, Lambda_para);
      break;
  }
  
  // Check for kernel launch errors
  CUDA_CHECK(cudaGetLastError());
}

// ============================================================================
// Bilateral Filter CUDA Kernel
// ============================================================================

// CUDA kernel for bilateral filtering
// Uses shared memory for the radiometric table to reduce global memory access
template<typename T>
__global__ void bilateral_filter_kernel(
    const T* src_data,
    T* dst_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    const double* radiometricTable,
    const double* spatialMask,
    int filterRadius,
    double valueRange)
{
  // 3D grid indexing
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  if (i >= xdim || j >= ydim || k >= zdim) return;
  
  int filterDiameter = filterRadius * 2 + 1;
  
  // Get current voxel value
  uint64 current_idx = k * xdim * ydim + j * xdim + i;
  double fsample = double(src_data[current_idx]);
  
  double sum = 0.0;
  double denom = 0.0;
  
  // Iterate over the filter window
  for (int z = 0; z < filterDiameter; z++) {
    int kk = int(k) + z - filterRadius;
    if (kk < 0 || kk >= int(zdim)) continue;
    
    for (int y = 0; y < filterDiameter; y++) {
      int jj = int(j) + y - filterRadius;
      if (jj < 0 || jj >= int(ydim)) continue;
      
      for (int x = 0; x < filterDiameter; x++) {
        int ii = int(i) + x - filterRadius;
        if (ii < 0 || ii >= int(xdim)) continue;
        
        // Get neighbor value - use explicit casts to avoid overflow
        uint64 neighbor_idx = uint64(kk) * xdim * ydim + uint64(jj) * xdim + uint64(ii);
        double neighbor_val = double(src_data[neighbor_idx]);
        
        // Calculate normalized difference
        double normalizedDiff = fabs(fsample - neighbor_val);
        if (valueRange > 0.0) {
          normalizedDiff /= valueRange;
          normalizedDiff *= 255.0;
        } else {
          normalizedDiff = 0.0;
        }
        
        // Clamp to table range
        int table_idx = int(normalizedDiff);
        if (table_idx > 255) table_idx = 255;
        if (table_idx < 0) table_idx = 0;
        
        // Get spatial mask index
        int mask_idx = z * filterDiameter * filterDiameter + y * filterDiameter + x;
        
        // Calculate weight
        double weight = radiometricTable[table_idx] * spatialMask[mask_idx];
        
        denom += weight;
        sum += weight * neighbor_val;
      }
    }
  }
  
  // Write result
  if (denom > 0.0) {
    dst_data[current_idx] = T(sum / denom);
  } else {
    dst_data[current_idx] = T(fsample);
  }
}

// Explicit template instantiations
template __global__ void bilateral_filter_kernel<unsigned char>(
    const unsigned char*, unsigned char*, uint64, uint64, uint64,
    const double*, const double*, int, double);

template __global__ void bilateral_filter_kernel<unsigned short>(
    const unsigned short*, unsigned short*, uint64, uint64, uint64,
    const double*, const double*, int, double);

template __global__ void bilateral_filter_kernel<unsigned int>(
    const unsigned int*, unsigned int*, uint64, uint64, uint64,
    const double*, const double*, int, double);

template __global__ void bilateral_filter_kernel<float>(
    const float*, float*, uint64, uint64, uint64,
    const double*, const double*, int, double);

template __global__ void bilateral_filter_kernel<double>(
    const double*, double*, uint64, uint64, uint64,
    const double*, const double*, int, double);

template __global__ void bilateral_filter_kernel<uint64>(
    const uint64*, uint64*, uint64, uint64, uint64,
    const double*, const double*, int, double);

// Host function to launch bilateral filter kernel
extern "C" void cuda_bilateral_filter(
    void* src_data,
    void* dst_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    double radiometricSigma,
    double spatialSigma,
    unsigned int filterRadius,
    double valueRange,
    data_type voxel_type)
{
  int filterDiameter = filterRadius * 2 + 1;
  
  // Allocate and compute radiometric table on device
  double* d_radiometricTable;
  CUDA_CHECK(cudaMalloc(&d_radiometricTable, 256 * sizeof(double)));
  
  double h_radiometricTable[256];
  for (int c = 0; c < 256; c++) {
    double factor = c * (valueRange / 255.0);
    h_radiometricTable[c] = exp((factor * factor) / (-radiometricSigma * radiometricSigma * 2.0));
  }
  CUDA_CHECK(cudaMemcpy(d_radiometricTable, h_radiometricTable, 
                        256 * sizeof(double), cudaMemcpyHostToDevice));
  
  // Allocate and compute spatial mask on device
  int maskSize = filterDiameter * filterDiameter * filterDiameter;
  double* d_spatialMask;
  CUDA_CHECK(cudaMalloc(&d_spatialMask, maskSize * sizeof(double)));
  
  double* h_spatialMask = new double[maskSize];
  int index = 0;
  for (int k = -int(filterRadius); k <= int(filterRadius); k++) {
    for (int j = -int(filterRadius); j <= int(filterRadius); j++) {
      for (int i = -int(filterRadius); i <= int(filterRadius); i++) {
        h_spatialMask[index++] = exp(double(k*k + j*j + i*i) / 
                                      (-spatialSigma * spatialSigma * 2.0));
      }
    }
  }
  CUDA_CHECK(cudaMemcpy(d_spatialMask, h_spatialMask, 
                        maskSize * sizeof(double), cudaMemcpyHostToDevice));
  delete[] h_spatialMask;
  
  // Configure kernel launch parameters
  dim3 blockSize(8, 8, 8);  // 512 threads per block
  dim3 gridSize(
      (xdim + blockSize.x - 1) / blockSize.x,
      (ydim + blockSize.y - 1) / blockSize.y,
      (zdim + blockSize.z - 1) / blockSize.z
  );
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      bilateral_filter_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(src_data),
          static_cast<unsigned char*>(dst_data),
          xdim, ydim, zdim, d_radiometricTable, d_spatialMask,
          filterRadius, valueRange);
      break;
      
    case UShort:
      bilateral_filter_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(src_data),
          static_cast<unsigned short*>(dst_data),
          xdim, ydim, zdim, d_radiometricTable, d_spatialMask,
          filterRadius, valueRange);
      break;
      
    case UInt:
      bilateral_filter_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(src_data),
          static_cast<unsigned int*>(dst_data),
          xdim, ydim, zdim, d_radiometricTable, d_spatialMask,
          filterRadius, valueRange);
      break;
      
    case Float:
      bilateral_filter_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(src_data),
          static_cast<float*>(dst_data),
          xdim, ydim, zdim, d_radiometricTable, d_spatialMask,
          filterRadius, valueRange);
      break;
      
    case Double:
      bilateral_filter_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(src_data),
          static_cast<double*>(dst_data),
          xdim, ydim, zdim, d_radiometricTable, d_spatialMask,
          filterRadius, valueRange);
      break;
      
    case UInt64:
      bilateral_filter_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(src_data),
          static_cast<uint64*>(dst_data),
          xdim, ydim, zdim, d_radiometricTable, d_spatialMask,
          filterRadius, valueRange);
      break;
  }
  
  // Check for kernel launch errors
  CUDA_CHECK(cudaGetLastError());
  
  // Synchronize and clean up
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaFree(d_radiometricTable));
  CUDA_CHECK(cudaFree(d_spatialMask));
}

// ============================================================================
// Contrast Enhancement Stretching Kernel
// ============================================================================

template<typename T>
__global__ void contrast_enhancement_stretching_kernel(
    const T* src_data,           // Original image data
    const T* upmin_data,         // Bottom-up minimum propagation
    const T* upmax_data,         // Bottom-up maximum propagation  
    const T* downmin_data,       // Top-down minimum propagation
    const T* downmax_data,       // Top-down maximum propagation
    const T* imgavg_data,        // Average image data
    T* result_data,              // Output stretched data
    uint64 xdim, uint64 ydim, uint64 zdim,
    double lmin_global,          // Global minimum for normalization
    double lmax_global)          // Global maximum for normalization
{
  // 3D thread indexing
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  if (i >= xdim || j >= ydim || k >= zdim) return;
  
  uint64 idx = k * xdim * ydim + j * xdim + i;
  
  // Read inputs - convert to double for computation
  double lmin = fmin(double(upmin_data[idx]), double(downmin_data[idx]));
  double lmax = fmax(double(upmax_data[idx]), double(downmax_data[idx]));
  double img = double(src_data[idx]);
  double avg = double(imgavg_data[idx]);
  
  // Compute window
  double window = lmax - lmin;
  window = sqrt(window * (510.0 - window));
  
  double result;
  
  if (lmin != lmax) {
    img = window * (img - lmin) / (lmax - lmin);
    avg = window * (avg - lmin) / (lmax - lmin);
  }
  
  double alpha = (avg - img) / (181.019 * window);
  
  if (alpha != 0.0) {
    double a = 0.707 * alpha;
    double b = 1.414 * alpha * (img - window) - 1.0;
    double c = 0.707 * alpha * img * (img - 2.0 * window) + img;
    double discriminant = b * b - 4.0 * a * c;
    
    if (discriminant >= 0.0) {
      result = lmin + (-b - sqrt(discriminant)) / (2.0 * a);
    } else {
      result = img + lmin;  // Fallback if discriminant is negative
    }
  } else {
    result = img + lmin;
  }
  
  // Write result
  result_data[idx] = T(result);
}

// Template instantiations
template __global__ void contrast_enhancement_stretching_kernel<unsigned char>(
    const unsigned char*, const unsigned char*, const unsigned char*,
    const unsigned char*, const unsigned char*, const unsigned char*,
    unsigned char*, uint64, uint64, uint64, double, double);
    
template __global__ void contrast_enhancement_stretching_kernel<unsigned short>(
    const unsigned short*, const unsigned short*, const unsigned short*,
    const unsigned short*, const unsigned short*, const unsigned short*,
    unsigned short*, uint64, uint64, uint64, double, double);
    
template __global__ void contrast_enhancement_stretching_kernel<unsigned int>(
    const unsigned int*, const unsigned int*, const unsigned int*,
    const unsigned int*, const unsigned int*, const unsigned int*,
    unsigned int*, uint64, uint64, uint64, double, double);
    
template __global__ void contrast_enhancement_stretching_kernel<float>(
    const float*, const float*, const float*,
    const float*, const float*, const float*,
    float*, uint64, uint64, uint64, double, double);
    
template __global__ void contrast_enhancement_stretching_kernel<double>(
    const double*, const double*, const double*,
    const double*, const double*, const double*,
    double*, uint64, uint64, uint64, double, double);
    
template __global__ void contrast_enhancement_stretching_kernel<uint64>(
    const uint64*, const uint64*, const uint64*,
    const uint64*, const uint64*, const uint64*,
    uint64*, uint64, uint64, uint64, double, double);

// Host function to launch contrast enhancement stretching kernel
extern "C" void cuda_contrast_enhancement_stretching(
    void* src_data,
    void* upmin_data,
    void* upmax_data,
    void* downmin_data,
    void* downmax_data,
    void* imgavg_data,
    void* result_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    double lmin_global,
    double lmax_global,
    data_type voxel_type)
{
  // Configure kernel launch parameters
  dim3 blockSize(8, 8, 8);  // 512 threads per block
  dim3 gridSize(
      (xdim + blockSize.x - 1) / blockSize.x,
      (ydim + blockSize.y - 1) / blockSize.y,
      (zdim + blockSize.z - 1) / blockSize.z
  );
  
  // Launch kernel based on voxel type
  switch (voxel_type) {
    case UChar:
      contrast_enhancement_stretching_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(src_data),
          static_cast<const unsigned char*>(upmin_data),
          static_cast<const unsigned char*>(upmax_data),
          static_cast<const unsigned char*>(downmin_data),
          static_cast<const unsigned char*>(downmax_data),
          static_cast<const unsigned char*>(imgavg_data),
          static_cast<unsigned char*>(result_data),
          xdim, ydim, zdim, lmin_global, lmax_global);
      break;
      
    case UShort:
      contrast_enhancement_stretching_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(src_data),
          static_cast<const unsigned short*>(upmin_data),
          static_cast<const unsigned short*>(upmax_data),
          static_cast<const unsigned short*>(downmin_data),
          static_cast<const unsigned short*>(downmax_data),
          static_cast<const unsigned short*>(imgavg_data),
          static_cast<unsigned short*>(result_data),
          xdim, ydim, zdim, lmin_global, lmax_global);
      break;
      
    case UInt:
      contrast_enhancement_stretching_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(src_data),
          static_cast<const unsigned int*>(upmin_data),
          static_cast<const unsigned int*>(upmax_data),
          static_cast<const unsigned int*>(downmin_data),
          static_cast<const unsigned int*>(downmax_data),
          static_cast<const unsigned int*>(imgavg_data),
          static_cast<unsigned int*>(result_data),
          xdim, ydim, zdim, lmin_global, lmax_global);
      break;
      
    case Float:
      contrast_enhancement_stretching_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(src_data),
          static_cast<const float*>(upmin_data),
          static_cast<const float*>(upmax_data),
          static_cast<const float*>(downmin_data),
          static_cast<const float*>(downmax_data),
          static_cast<const float*>(imgavg_data),
          static_cast<float*>(result_data),
          xdim, ydim, zdim, lmin_global, lmax_global);
      break;
      
    case Double:
      contrast_enhancement_stretching_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(src_data),
          static_cast<const double*>(upmin_data),
          static_cast<const double*>(upmax_data),
          static_cast<const double*>(downmin_data),
          static_cast<const double*>(downmax_data),
          static_cast<const double*>(imgavg_data),
          static_cast<double*>(result_data),
          xdim, ydim, zdim, lmin_global, lmax_global);
      break;
      
    case UInt64:
      contrast_enhancement_stretching_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(src_data),
          static_cast<const uint64*>(upmin_data),
          static_cast<const uint64*>(upmax_data),
          static_cast<const uint64*>(downmin_data),
          static_cast<const uint64*>(downmax_data),
          static_cast<const uint64*>(imgavg_data),
          static_cast<uint64*>(result_data),
          xdim, ydim, zdim, lmin_global, lmax_global);
      break;
  }
  
  // Check for kernel launch errors
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
}

// ============================================================================
// GDTV Filter Kernels
// ============================================================================

// Device helper function for phi calculation
__device__ inline float phi_device(float x, float q) {
  return (2.0f - q) * powf(x, 1.0f - q);
}

// Gradient computation kernel (6-neighborhood)
template<typename T>
__global__ void gdtv_gradient_kernel(
    const T* input_data,
    float* grad_data,
    uint64 xdim, uint64 ydim, uint64 zdim)
{
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  if (i >= xdim || j >= ydim || k >= zdim) return;
  
  uint64 idx = k * xdim * ydim + j * xdim + i;
  float center = float(input_data[idx]);
  
  // Compute differences with 6 neighbors
  float val[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  if (i + 1 < xdim) val[0] = float(input_data[idx + 1]) - center;
  if (i >= 1) val[1] = float(input_data[idx - 1]) - center;
  if (j + 1 < ydim) val[2] = float(input_data[idx + xdim]) - center;
  if (j >= 1) val[3] = float(input_data[idx - xdim]) - center;
  if (k + 1 < zdim) val[4] = float(input_data[idx + xdim * ydim]) - center;
  if (k >= 1) val[5] = float(input_data[idx - xdim * ydim]) - center;
  
  // Compute gradient magnitude
  float sum = 0.0f;
  for (int l = 0; l < 6; l++) {
    sum += val[l] * val[l];
  }
  
  grad_data[idx] = sqrtf(sum);
}

// GDTV filtering kernel (6-neighborhood)
template<typename T>
__global__ void gdtv_filter_kernel(
    const T* input_data,
    const float* grad_data,
    const T* funcval_data,
    T* funcvalue_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    float q, float lbda)
{
  uint64 i = blockIdx.x * blockDim.x + threadIdx.x;
  uint64 j = blockIdx.y * blockDim.y + threadIdx.y;
  uint64 k = blockIdx.z * blockDim.z + threadIdx.z;
  
  if (i >= xdim || j >= ydim || k >= zdim) return;
  
  uint64 idx = k * xdim * ydim + j * xdim + i;
  const float epsilon = 0.0001f;
  
  float temp[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float grad_center = grad_data[idx];
  
  if (grad_center != 0.0f) {
    float phi_center = phi_device(grad_center, q) / grad_center;
    
    // Compute weights for each neighbor
    if (i < xdim - 1) {
      uint64 neighbor_idx = idx + 1;
      float grad_neighbor = grad_data[neighbor_idx];
      if (grad_neighbor != 0.0f)
        temp[0] = phi_center + phi_device(grad_neighbor, q) / grad_neighbor;
      else
        temp[0] = phi_center + phi_device(epsilon, q) / epsilon;
    }
    
    if (i > 1) {
      uint64 neighbor_idx = idx - 1;
      float grad_neighbor = grad_data[neighbor_idx];
      if (grad_neighbor != 0.0f)
        temp[1] = phi_center + phi_device(grad_neighbor, q) / grad_neighbor;
      else
        temp[1] = phi_center + phi_device(epsilon, q) / epsilon;
    }
    
    if (j < ydim - 1) {
      uint64 neighbor_idx = idx + xdim;
      float grad_neighbor = grad_data[neighbor_idx];
      if (grad_neighbor != 0.0f)
        temp[2] = phi_center + phi_device(grad_neighbor, q) / grad_neighbor;
      else
        temp[2] = phi_center + phi_device(epsilon, q) / epsilon;
    }
    
    if (j > 1) {
      uint64 neighbor_idx = idx - xdim;
      float grad_neighbor = grad_data[neighbor_idx];
      if (grad_neighbor != 0.0f)
        temp[3] = phi_center + phi_device(grad_neighbor, q) / grad_neighbor;
      else
        temp[3] = phi_center + phi_device(epsilon, q) / epsilon;
    }
    
    if (k < zdim - 1) {
      uint64 neighbor_idx = idx + xdim * ydim;
      float grad_neighbor = grad_data[neighbor_idx];
      if (grad_neighbor != 0.0f)
        temp[4] = phi_center + phi_device(grad_neighbor, q) / grad_neighbor;
      else
        temp[4] = phi_center + phi_device(epsilon, q) / epsilon;
    }
    
    if (k > 1) {
      uint64 neighbor_idx = idx - xdim * ydim;
      float grad_neighbor = grad_data[neighbor_idx];
      if (grad_neighbor != 0.0f)
        temp[5] = phi_center + phi_device(grad_neighbor, q) / grad_neighbor;
      else
        temp[5] = phi_center + phi_device(epsilon, q) / epsilon;
    }
  }
  
  temp[6] = lbda;
  
  // Normalize weights
  float sum = 0.0f;
  for (int l = 0; l < 7; l++) {
    sum += temp[l];
  }
  
  if (sum > 0.0f) {
    for (int l = 0; l < 7; l++) {
      temp[l] = temp[l] / sum;
    }
  }
  
  // Compute weighted average
  double result = 0.0;
  if (i < xdim - 1) result += double(funcval_data[idx + 1]) * temp[0];
  if (i > 1) result += double(funcval_data[idx - 1]) * temp[1];
  if (j < ydim - 1) result += double(funcval_data[idx + xdim]) * temp[2];
  if (j > 1) result += double(funcval_data[idx - xdim]) * temp[3];
  if (k < zdim - 1) result += double(funcval_data[idx + xdim * ydim]) * temp[4];
  if (k > 1) result += double(funcval_data[idx - xdim * ydim]) * temp[5];
  result += double(input_data[idx]) * temp[6];
  
  funcvalue_data[idx] = T(result);
}

// Template instantiations for gradient kernel
template __global__ void gdtv_gradient_kernel<unsigned char>(
    const unsigned char*, float*, uint64, uint64, uint64);
template __global__ void gdtv_gradient_kernel<unsigned short>(
    const unsigned short*, float*, uint64, uint64, uint64);
template __global__ void gdtv_gradient_kernel<unsigned int>(
    const unsigned int*, float*, uint64, uint64, uint64);
template __global__ void gdtv_gradient_kernel<float>(
    const float*, float*, uint64, uint64, uint64);
template __global__ void gdtv_gradient_kernel<double>(
    const double*, float*, uint64, uint64, uint64);
template __global__ void gdtv_gradient_kernel<uint64>(
    const uint64*, float*, uint64, uint64, uint64);

// Template instantiations for filter kernel
template __global__ void gdtv_filter_kernel<unsigned char>(
    const unsigned char*, const float*, const unsigned char*, unsigned char*,
    uint64, uint64, uint64, float, float);
template __global__ void gdtv_filter_kernel<unsigned short>(
    const unsigned short*, const float*, const unsigned short*, unsigned short*,
    uint64, uint64, uint64, float, float);
template __global__ void gdtv_filter_kernel<unsigned int>(
    const unsigned int*, const float*, const unsigned int*, unsigned int*,
    uint64, uint64, uint64, float, float);
template __global__ void gdtv_filter_kernel<float>(
    const float*, const float*, const float*, float*,
    uint64, uint64, uint64, float, float);
template __global__ void gdtv_filter_kernel<double>(
    const double*, const float*, const double*, double*,
    uint64, uint64, uint64, float, float);
template __global__ void gdtv_filter_kernel<uint64>(
    const uint64*, const float*, const uint64*, uint64*,
    uint64, uint64, uint64, float, float);

// Host function for GDTV gradient computation
extern "C" void cuda_gdtv_gradient(
    void* input_data,
    void* grad_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    data_type voxel_type)
{
  dim3 blockSize(8, 8, 8);
  dim3 gridSize(
      (xdim + blockSize.x - 1) / blockSize.x,
      (ydim + blockSize.y - 1) / blockSize.y,
      (zdim + blockSize.z - 1) / blockSize.z
  );
  
  switch (voxel_type) {
    case UChar:
      gdtv_gradient_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(input_data),
          static_cast<float*>(grad_data),
          xdim, ydim, zdim);
      break;
    case UShort:
      gdtv_gradient_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(input_data),
          static_cast<float*>(grad_data),
          xdim, ydim, zdim);
      break;
    case UInt:
      gdtv_gradient_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(input_data),
          static_cast<float*>(grad_data),
          xdim, ydim, zdim);
      break;
    case Float:
      gdtv_gradient_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(input_data),
          static_cast<float*>(grad_data),
          xdim, ydim, zdim);
      break;
    case Double:
      gdtv_gradient_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(input_data),
          static_cast<float*>(grad_data),
          xdim, ydim, zdim);
      break;
    case UInt64:
      gdtv_gradient_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(input_data),
          static_cast<float*>(grad_data),
          xdim, ydim, zdim);
      break;
  }
  
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
}

// Host function for GDTV filter iteration
extern "C" void cuda_gdtv_filter(
    void* input_data,
    void* grad_data,
    void* funcval_data,
    void* funcvalue_data,
    uint64 xdim, uint64 ydim, uint64 zdim,
    float q, float lbda,
    data_type voxel_type)
{
  dim3 blockSize(8, 8, 8);
  dim3 gridSize(
      (xdim + blockSize.x - 1) / blockSize.x,
      (ydim + blockSize.y - 1) / blockSize.y,
      (zdim + blockSize.z - 1) / blockSize.z
  );
  
  switch (voxel_type) {
    case UChar:
      gdtv_filter_kernel<unsigned char><<<gridSize, blockSize>>>(
          static_cast<const unsigned char*>(input_data),
          static_cast<const float*>(grad_data),
          static_cast<const unsigned char*>(funcval_data),
          static_cast<unsigned char*>(funcvalue_data),
          xdim, ydim, zdim, q, lbda);
      break;
    case UShort:
      gdtv_filter_kernel<unsigned short><<<gridSize, blockSize>>>(
          static_cast<const unsigned short*>(input_data),
          static_cast<const float*>(grad_data),
          static_cast<const unsigned short*>(funcval_data),
          static_cast<unsigned short*>(funcvalue_data),
          xdim, ydim, zdim, q, lbda);
      break;
    case UInt:
      gdtv_filter_kernel<unsigned int><<<gridSize, blockSize>>>(
          static_cast<const unsigned int*>(input_data),
          static_cast<const float*>(grad_data),
          static_cast<const unsigned int*>(funcval_data),
          static_cast<unsigned int*>(funcvalue_data),
          xdim, ydim, zdim, q, lbda);
      break;
    case Float:
      gdtv_filter_kernel<float><<<gridSize, blockSize>>>(
          static_cast<const float*>(input_data),
          static_cast<const float*>(grad_data),
          static_cast<const float*>(funcval_data),
          static_cast<float*>(funcvalue_data),
          xdim, ydim, zdim, q, lbda);
      break;
    case Double:
      gdtv_filter_kernel<double><<<gridSize, blockSize>>>(
          static_cast<const double*>(input_data),
          static_cast<const float*>(grad_data),
          static_cast<const double*>(funcval_data),
          static_cast<double*>(funcvalue_data),
          xdim, ydim, zdim, q, lbda);
      break;
    case UInt64:
      gdtv_filter_kernel<uint64><<<gridSize, blockSize>>>(
          static_cast<const uint64*>(input_data),
          static_cast<const float*>(grad_data),
          static_cast<const uint64*>(funcval_data),
          static_cast<uint64*>(funcvalue_data),
          xdim, ydim, zdim, q, lbda);
      break;
  }
  
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
}

} // namespace cvc

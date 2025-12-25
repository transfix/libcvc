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
#include <cvc/types.h>

namespace CVC_NAMESPACE
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
    const T* __restrict__ src_data,
    T* __restrict__ dst_data,
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
    const T* __restrict__ src_data,
    T* __restrict__ dst_data,
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

// Anisotropic diffusion kernel - processes one slice at a time
template<typename T>
__global__ void anisotropic_diffusion_slice_kernel(
    const T* __restrict__ src_data,
    T* __restrict__ dst_data,
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

} // namespace CVC_NAMESPACE

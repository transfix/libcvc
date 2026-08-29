/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// material_train.cu — the CUDA-resident optimizer for CoefEnergyNetMaterial: the
// device twin of material_adam (material_train.cpp). The model weights and the
// Adam (m,u) moments live on the device across steps; each step uploads only the
// flattened gradient and runs the global-norm clip + bias-corrected update on the
// device (the same grad_sqnorm_kernel / adam_kernel shape coef_train.cu uses for
// the base CoefMLP), so a training loop needs no per-step weight round-trip. The
// model's ~50 named tensors are flattened into one contiguous param vector in
// param_names() order (a std::map, so the order is stable). Float-equivalent to
// material_adam — the only numerical difference is the norm reduction order (device
// tree-reduce + atomicAdd in float vs the CPU's sequential double sum). Built
// precise (-fmad=false --prec-div/sqrt --ftz=false, set in CMake) like the other
// nav .cu. Validated by nav_material_adam_cuda_test.

#include <algorithm>
#include <cmath>
#include <cuda_runtime.h>
#include <cvc/nav/material_train.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

namespace {

void mt_cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("cvc::nav::material_train CUDA: ") + what + ": " +
                             cudaGetErrorString(e));
}

// Sum of squares of a flat P-vector -> *out (atomicAdd; caller zeroes *out).
__global__ void mt_grad_sqnorm_kernel(const float *g, int P, float *out) {
  __shared__ float sh[256];
  float acc = 0.0f;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < P; i += gridDim.x * blockDim.x)
    acc += g[i] * g[i];
  sh[threadIdx.x] = acc;
  __syncthreads();
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s)
      sh[threadIdx.x] += sh[threadIdx.x + s];
    __syncthreads();
  }
  if (threadIdx.x == 0)
    atomicAdd(out, sh[0]);
}

// In-place Adam update (identical math to material_adam::step); gscale folds the
// global-norm clip, bc1/bc2 the bias correction. One thread per param.
__global__ void mt_adam_kernel(float *p, float *m, float *u, const float *g, int P, float lr,
                               float b1, float b2, float eps, float bc1, float bc2, float gscale) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= P)
    return;
  const float gg = g[i] * gscale;
  m[i] = b1 * m[i] + (1.0f - b1) * gg;
  u[i] = b2 * u[i] + (1.0f - b2) * gg * gg;
  const float mhat = m[i] / bc1, uhat = u[i] / bc2;
  p[i] -= lr * mhat / (sqrtf(uhat) + eps);
}

} // namespace

bool material_train_cuda_available() {
  int c = 0;
  return cudaGetDeviceCount(&c) == cudaSuccess && c > 0;
}

material_adam_cuda::material_adam_cuda(const coef_energy_net &model, float grad_clip)
    : grad_clip_(grad_clip), names_(model.param_names()) {
  if (!material_train_cuda_available())
    throw std::runtime_error("material_adam_cuda: no CUDA device");
  // Flatten layout (param_names() order) + the initial weights.
  off_.assign(names_.size() + 1, 0);
  for (std::size_t i = 0; i < names_.size(); ++i)
    off_[i + 1] = off_[i] + static_cast<int>(model.param(names_[i]).size());
  P_ = off_.back();
  std::vector<float> flat(P_);
  for (std::size_t i = 0; i < names_.size(); ++i) {
    const std::vector<float> &w = model.param(names_[i]);
    std::copy(w.begin(), w.end(), flat.begin() + off_[i]);
  }
  const std::size_t bytes = static_cast<std::size_t>(P_) * sizeof(float);
  mt_cuda_check(cudaMalloc(&d_p_, bytes), "malloc p");
  mt_cuda_check(cudaMalloc(&d_m_, bytes), "malloc m");
  mt_cuda_check(cudaMalloc(&d_u_, bytes), "malloc u");
  mt_cuda_check(cudaMalloc(&d_g_, bytes), "malloc g");
  mt_cuda_check(cudaMalloc(&d_sq_, sizeof(float)), "malloc sq");
  mt_cuda_check(cudaMemcpy(d_p_, flat.data(), bytes, cudaMemcpyHostToDevice), "H2D p");
  mt_cuda_check(cudaMemset(d_m_, 0, bytes), "memset m");
  mt_cuda_check(cudaMemset(d_u_, 0, bytes), "memset u");
}

material_adam_cuda::~material_adam_cuda() {
  cudaFree(d_p_);
  cudaFree(d_m_);
  cudaFree(d_u_);
  cudaFree(d_g_);
  cudaFree(d_sq_);
}

void material_adam_cuda::step(const coef_energy_net::param_grads &grad, float lr) {
  ++t_;
  // flatten grad in the same order as the weights, upload
  std::vector<float> flat(P_);
  for (std::size_t i = 0; i < names_.size(); ++i) {
    const std::vector<float> &gv = grad.at(names_[i]);
    std::copy(gv.begin(), gv.end(), flat.begin() + off_[i]);
  }
  const std::size_t bytes = static_cast<std::size_t>(P_) * sizeof(float);
  mt_cuda_check(cudaMemcpy(d_g_, flat.data(), bytes, cudaMemcpyHostToDevice), "H2D g");
  // global grad norm on device
  mt_cuda_check(cudaMemset(d_sq_, 0, sizeof(float)), "memset sq");
  mt_grad_sqnorm_kernel<<<32, 256>>>(d_g_, P_, d_sq_);
  mt_cuda_check(cudaGetLastError(), "sqnorm launch");
  float sq = 0.0f;
  mt_cuda_check(cudaMemcpy(&sq, d_sq_, sizeof(float), cudaMemcpyDeviceToHost), "D2H sq");
  const float norm = std::sqrt(sq);
  const float gscale = (grad_clip_ > 0.0f && norm > grad_clip_) ? grad_clip_ / norm : 1.0f;
  const float bc1 = 1.0f - std::pow(b1_, static_cast<float>(t_));
  const float bc2 = 1.0f - std::pow(b2_, static_cast<float>(t_));
  const int PT = 256, PB = (P_ + PT - 1) / PT;
  mt_adam_kernel<<<PB, PT>>>(d_p_, d_m_, d_u_, d_g_, P_, lr, b1_, b2_, eps_, bc1, bc2, gscale);
  mt_cuda_check(cudaGetLastError(), "adam launch");
  mt_cuda_check(cudaDeviceSynchronize(), "sync");
}

void material_adam_cuda::sync_to(coef_energy_net &model) const {
  std::vector<float> flat(P_);
  mt_cuda_check(cudaMemcpy(flat.data(), d_p_, static_cast<std::size_t>(P_) * sizeof(float),
                           cudaMemcpyDeviceToHost),
                "D2H p");
  for (std::size_t i = 0; i < names_.size(); ++i) {
    std::vector<float> &w = model.mutable_param(names_[i]);
    std::copy(flat.begin() + off_[i], flat.begin() + off_[i + 1], w.begin());
  }
}

} // namespace nav
} // namespace cvc

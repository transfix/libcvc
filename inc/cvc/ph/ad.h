/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::ad: reverse-mode automatic differentiation on a tape, for training cvc::ph's component
// laws (train.h).
//
// A tape records every operation as a node holding the local partials with respect to its (at
// most two) operands; backward(out) sweeps it once in reverse to give d out / d node for every
// node. A var is a value plus its node on a tape; a var with no tape is a constant and records
// nothing, so expressions mixing variables and plain doubles stay small. The tape is the
// caller's: nothing here is global, and one tape per thread is the way to differentiate in
// parallel.
//
// The elementary functions are the ones the laws use, written to give the same values as
// compose.cpp's double code (softplus with torch's threshold of 20, the two-branch logistic).

#ifndef __CVC_PH_AD_H__
#define __CVC_PH_AD_H__

#include <cmath>
#include <cstddef>
#include <vector>

namespace cvc {
namespace ph {
namespace ad {

class tape {
public:
  struct node {
    int a = -1, b = -1;
    double da = 0, db = 0;
  };

  // A new input node (no operands).
  int input() {
    nodes_.push_back(node{});
    return static_cast<int>(nodes_.size()) - 1;
  }
  int unary(int a, double da) {
    nodes_.push_back(node{a, -1, da, 0});
    return static_cast<int>(nodes_.size()) - 1;
  }
  int binary(int a, double da, int b, double db) {
    nodes_.push_back(node{a, b, da, db});
    return static_cast<int>(nodes_.size()) - 1;
  }

  std::size_t size() const { return nodes_.size(); }
  void clear() {
    nodes_.clear();
    adj_.clear();
  }
  void reserve(std::size_t n) { nodes_.reserve(n); }

  // d out / d node for every node (adjoint(i) afterwards).
  void backward(int out) {
    adj_.assign(nodes_.size(), 0.0);
    if (out < 0)
      return;
    adj_[out] = 1.0;
    for (int i = out; i >= 0; --i) {
      const double g = adj_[i];
      if (g == 0)
        continue;
      const node &n = nodes_[i];
      if (n.a >= 0)
        adj_[n.a] += g * n.da;
      if (n.b >= 0)
        adj_[n.b] += g * n.db;
    }
  }
  double adjoint(int i) const {
    return i >= 0 && i < static_cast<int>(adj_.size()) ? adj_[i] : 0.0;
  }

private:
  std::vector<node> nodes_;
  std::vector<double> adj_;
};

struct var {
  double v = 0;
  int i = -1;        // the node, -1 for a constant
  tape *t = nullptr; // the tape, null for a constant
  var() = default;
  var(double c) : v(c) {} // NOLINT(google-explicit-constructor): constants mix freely
  var(double value, int index, tape *tp) : v(value), i(index), t(tp) {}
  bool constant() const { return t == nullptr; }
};

// A fresh variable on t.
inline var variable(tape &t, double value) { return var(value, t.input(), &t); }

inline double value(const var &x) { return x.v; }
inline double value(double x) { return x; }

namespace detail {
inline var unary(const var &a, double v, double da) {
  if (a.constant())
    return var(v);
  return var(v, a.t->unary(a.i, da), a.t);
}
inline var binary(const var &a, const var &b, double v, double da, double db) {
  if (a.constant() && b.constant())
    return var(v);
  if (a.constant())
    return var(v, b.t->unary(b.i, db), b.t);
  if (b.constant())
    return var(v, a.t->unary(a.i, da), a.t);
  return var(v, a.t->binary(a.i, da, b.i, db), a.t);
}
} // namespace detail

inline var operator+(const var &a, const var &b) {
  return detail::binary(a, b, a.v + b.v, 1.0, 1.0);
}
inline var operator-(const var &a, const var &b) {
  return detail::binary(a, b, a.v - b.v, 1.0, -1.0);
}
inline var operator*(const var &a, const var &b) {
  return detail::binary(a, b, a.v * b.v, b.v, a.v);
}
inline var operator/(const var &a, const var &b) {
  const double q = a.v / b.v;
  return detail::binary(a, b, q, 1.0 / b.v, -q / b.v);
}
inline var operator-(const var &a) { return detail::unary(a, -a.v, -1.0); }
inline var &operator+=(var &a, const var &b) { return a = a + b; }
inline var &operator-=(var &a, const var &b) { return a = a - b; }
inline var &operator*=(var &a, const var &b) { return a = a * b; }
inline var &operator/=(var &a, const var &b) { return a = a / b; }

inline var exp(const var &a) {
  const double e = std::exp(a.v);
  return detail::unary(a, e, e);
}
inline var log1p(const var &a) { return detail::unary(a, std::log1p(a.v), 1.0 / (1.0 + a.v)); }
inline var sqrt(const var &a) {
  const double s = std::sqrt(a.v);
  return detail::unary(a, s, 0.5 / s);
}
inline var tanh(const var &a) {
  const double y = std::tanh(a.v);
  return detail::unary(a, y, 1 - y * y);
}

// The logistic and torch's softplus (beta 1, threshold 20), as compose.cpp computes them.
inline double sigmoid(double z) {
  if (z >= 0)
    return 1 / (1 + std::exp(-z));
  const double e = std::exp(z);
  return e / (1 + e);
}
inline double softplus(double z) { return z > 20 ? z : std::log1p(std::exp(z)); }
inline var sigmoid(const var &a) {
  const double s = sigmoid(a.v);
  return detail::unary(a, s, s * (1 - s));
}
inline var softplus(const var &a) {
  return detail::unary(a, softplus(a.v), a.v > 20 ? 1.0 : sigmoid(a.v));
}
inline var silu(const var &a) {
  const double s = sigmoid(a.v);
  return detail::unary(a, a.v * s, s * (1 + a.v * (1 - s)));
}
inline double silu(double z) { return z * sigmoid(z); }

using std::exp;
using std::log1p;
using std::sqrt;
using std::tanh;

} // namespace ad
} // namespace ph
} // namespace cvc

#endif

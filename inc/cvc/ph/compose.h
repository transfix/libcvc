/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph composition: port-Hamiltonian components joined by power-preserving ports. The method is
// C-PHAST, by Shubham Bhardwaj and Chandrajit Bajaj (UT Austin, CVC); the port-Hamiltonian
// interconnection it builds on is classical (van der Schaft and Jeltsema, "Port-Hamiltonian
// Systems Theory: An Introductory Overview", 2014). docs/PH_COMPOSE.md describes the API.
//
// A component (block_law) has a state x, a stored energy H, a lossless structure J = -J^T, a
// dissipation R = R^T >= 0, coupling ports B and external inputs G:
//
// clang-format off
//   dx/dt = (J - R) e + B u_hat + G u,    e = grad H,    y_hat = B^T e
// clang-format on
//
// A system stacks components and pairs their coupling ports by a skew matrix C = -C^T (the
// pairing): u_hat = -C y_hat. Its blocks C_ij may be rectangular and C_ii = 0. The composed field
// is
//
// clang-format off
//   dx/dt = (J_hat - B_hat C B_hat^T - R_hat) grad H_total + G_hat u
// clang-format on
//
// with block-diagonal stacks, H_total the sum of the component energies. The pairing does no net
// work, sum_i u_hat_i . y_hat_i = -y_hat . C y_hat = 0, so
//
//   dH_total/dt = -e . R_hat e + e . G_hat u        (loss, then the external ports' power)
//
// The numerical transition is a split step: per substep d = dt / substeps,
//
//   x <- E_{d/2}(x),  each block x_i <- LocalStep_d(x_i, u_i),  x <- E_{d/2}(x)
//
// where E_tau advances only the coupling field f_c = -B_hat C B_hat^T grad H, by explicit Euler
// (x + tau f_c(x)) or explicit midpoint (x + tau f_c(x + tau/2 f_c(x))), and LocalStep is
// kick tau/2, damp tau/2, conserve tau, damp tau/2, kick tau/2 over each of its own nsub substeps
// (kick: the input field G u; damp: -R grad H; conserve: J grad H by implicit midpoint or
// leapfrog). Euler coupling makes the step first order, midpoint coupling second order. Neither
// conserves energy exactly: each step's books (step_books, the trapezoid rule) carry a residual of
// the integrator's own error, O(dt^2) per step with Euler coupling and O(dt^3) with midpoint.
// step_rk4 and step_implicit_midpoint integrate the whole field instead, as references.
//
// Laws are immutable and shared (std::shared_ptr<const block_law>); a system is a value the
// caller owns (a composite). Nothing here is process-global.

#ifndef __CVC_PH_COMPOSE_H__
#define __CVC_PH_COMPOSE_H__

#include <cvc/nav/detail/dyn_bike.h> // dyn_ph_books: the energy books every libcvc plant keeps
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace ph {

using vecd = std::vector<double>;

// One step's energy books, in the plants' record: dH = supplied - dissipated + residual (floor,
// grade and limiter are zero here). supplied is the external ports' work, dissipated the loss,
// both by the trapezoid rule; residual is what the integrator added or removed.
using books = cvc::nav::diff::dyn_ph_books<double>;

// The implicit solves: Picard iteration from an explicit-Euler start, stopping when every
// coordinate moves by less than tol, or after max_iter iterations (returning the last iterate
// without an error).
struct solve_opts {
  int max_iter = 10;
  double tol = 1e-6;
};

// A component law. Matrices are row-major: J and R are n x n, B is n x ports, G is n x inputs.
class block_law {
public:
  virtual ~block_law() = default;

  virtual std::string kind() const = 0;
  virtual int state_dim() const = 0;
  virtual int port_dim() const = 0;
  virtual int input_dim() const = 0;

  virtual double energy(const double *x) const = 0;
  virtual void effort(const double *x, double *e) const = 0;
  virtual void structure(const double *x, double *J, double *R) const = 0;
  virtual void port_map(const double *x, double *B) const = 0;
  virtual void input_map(const double *x, double *G) const = 0;

  // The local split maps, each advancing x in place over a duration tau.
  // kick: the input field, x += tau G(x) u (u held).
  virtual void kick(double tau, const double *u, double *x) const;
  // damp: the dissipative field -R grad H.
  virtual void damp(double tau, double *x) const = 0;
  // conserve: the lossless field J grad H; returns the Picard iterations used (0 if explicit).
  virtual int conserve(double tau, double *x, const solve_opts &o) const = 0;

  // LocalStep over dt in nsub substeps; returns the largest Picard count.
  int local_step(double dt, int nsub, const double *u, double *x, const solve_opts &o) const;
};

using law_ptr = std::shared_ptr<const block_law>;

// ── linear_law: H = 1/2 x^T Q x with constant J, R, B, G ────────────────────────────────────────
// A transmission line (Q = 1/L, J = 0, R = R_line, B = 1), a linear DGU, or any other
// linear-quadratic component. damp is the trapezoid (Cayley) step of x' = -R Q x and conserve the
// implicit midpoint of x' = J Q x, both exact solves of linear maps.
struct linear_spec {
  int n = 0, ports = 0, inputs = 0;
  vecd Q, J, R, B, G; // Q, J, R: n x n; B: n x ports; G: n x inputs (row-major)
};

class linear_law final : public block_law {
public:
  explicit linear_law(linear_spec s); // throws std::invalid_argument on a malformed spec
  const linear_spec &spec() const { return s_; }

  std::string kind() const override { return "linear"; }
  int state_dim() const override { return s_.n; }
  int port_dim() const override { return s_.ports; }
  int input_dim() const override { return s_.inputs; }
  double energy(const double *x) const override;
  void effort(const double *x, double *e) const override;
  void structure(const double *x, double *J, double *R) const override;
  void port_map(const double *x, double *B) const override;
  void input_map(const double *x, double *G) const override;
  void damp(double tau, double *x) const override;
  int conserve(double tau, double *x, const solve_opts &o) const override;

private:
  linear_spec s_;
};

// ── canonical_law: the canonical mechanical form
// ────────────────────────────────────────────────────── x = (q, p) in R^n x R^n, H = V(q) + 1/2 p
// . M(q)^-1 p, J = [[0, I], [-I, 0]], R = diag(0, D(q)):
//
//   V(q) = 1/2 (q - c) . K (q - c) + v_scale v_net(q)
//   M(q) = diag(mass)        or diag(softplus(m_net(q)) + m_min)
//   D(q) = diag(damping)     or diag(softplus(d_net(q)) + d_min)
//
// An electrical DGU (distributed generation unit) is n = 1 with q the capacitor charge, p the
// inductor flux, V its capacitor energy, M its inductance and D its resistance: the efforts are the
// capacitor voltage dH/dq and the inductor current dH/dp. damp is p_j <- (1 - c D_j/M_j) / (1 + c
// D_j/M_j) p_j with c = tau / 2 and q frozen (the scalar Cayley step); conserve is implicit
// midpoint solved by Picard iteration (the full gradients, so M may depend on q) or, when M is
// constant, leapfrog (kick-drift-kick).

enum class activation { identity, silu, tanh, softplus, sigmoid };

struct dense_layer {
  int in = 0, out = 0;
  vecd W, b; // W: out x in, row-major
  activation act = activation::identity;
};

// A small multilayer perceptron with its input Jacobian.
struct mlp {
  std::vector<dense_layer> layers;
  bool empty() const { return layers.empty(); }
  int in_dim() const { return layers.empty() ? 0 : layers.front().in; }
  int out_dim() const { return layers.empty() ? 0 : layers.back().out; }
  // y = net(x); jac (out x in, row-major) when non-null.
  void eval(const double *x, double *y, double *jac = nullptr) const;
};

enum class conservative_scheme { implicit_midpoint, leapfrog };

struct canonical_spec {
  int n = 1;
  vecd K, center; // n x n stiffness (symmetric) and its centre; empty K is no quadratic term
  double v_scale = 1.0;
  mlp v_net; // R^n -> R; empty is none
  vecd mass; // n constants, used when m_net is empty
  mlp m_net; // R^n -> R^n
  double m_min = 0.0;
  vecd damping; // n constants (empty: no damping), used when d_net is empty
  mlp d_net;    // R^n -> R^n
  double d_min = 0.0;
  int ports = 0, inputs = 0;
  vecd B, G; // 2n x ports, 2n x inputs (row-major; rows q_1..q_n then p_1..p_n)
  conservative_scheme scheme = conservative_scheme::implicit_midpoint;
};

class canonical_law final : public block_law {
public:
  explicit canonical_law(canonical_spec s); // throws std::invalid_argument
  const canonical_spec &spec() const { return s_; }

  std::string kind() const override { return "canonical"; }
  int state_dim() const override { return 2 * s_.n; }
  int port_dim() const override { return s_.ports; }
  int input_dim() const override { return s_.inputs; }
  double energy(const double *x) const override;
  void effort(const double *x, double *e) const override;
  void structure(const double *x, double *J, double *R) const override;
  void port_map(const double *x, double *B) const override;
  void input_map(const double *x, double *G) const override;
  void damp(double tau, double *x) const override;
  int conserve(double tau, double *x, const solve_opts &o) const override;

  // The pieces at q (n each): V, its gradient, the diagonal of M and of D.
  double potential(const double *q, double *grad = nullptr) const;
  void mass(const double *q, double *m,
            double *dm = nullptr) const; // dm: n x n, dm[j][i] = dM_j/dq_i
  void damping(const double *q, double *d) const;

private:
  canonical_spec s_;
};

// ── composite: components and their pairing
// ───────────────────────────────────────────────────────

struct power_terms {
  double stored = 0;   // H_total
  double loss = 0;     // e . R_hat e >= 0
  double external = 0; // e . G_hat u, the power the external inputs supply
  double internal = 0; // u_hat . y_hat = -y_hat . C y_hat: zero for a skew pairing
};

class composite {
public:
  // Adds a component; returns its index. Names are optional but must be unique when given.
  int add_block(law_ptr law, std::string name = {});

  // Pairs port a of block i with port b of block j by coefficient c: C(i.a, j.b) += c and
  // C(j.b, i.a) -= c, so u_hat_i[a] gets -c y_hat_j[b] and u_hat_j[b] gets +c y_hat_i[a].
  void connect(int i, int a, int j, int b, double c = 1.0);
  // An electrical network: buses[k]'s port bus_port pairs with lines[l]'s
  // port line_port through the incidence matrix A (buses x lines, row-major; -1 at a line's sending
  // bus, +1 at its receiving bus), so that dq_bus += A i_line and dphi_line -= A^T v_bus.
  void connect_incidence(const std::vector<int> &buses, int bus_port, const std::vector<int> &lines,
                         int line_port, const vecd &A);
  // A dense rule over repeated components (s x s, row-major): port `port` of blocks[a] pairs with
  // the same port of blocks[b] by (C - C^T)/2, the skew part of the matrix given.
  void connect_skew(const std::vector<int> &blocks, int port, const vecd &C);

  int blocks() const { return static_cast<int>(laws_.size()); }
  const block_law &law(int i) const { return *laws_.at(i); }
  law_ptr law_ptr_at(int i) const { return laws_.at(i); }
  const std::string &name(int i) const { return names_.at(i); }
  int find(const std::string &name) const; // -1 when absent

  int state_dim() const { return nx_; }
  int input_dim() const { return nu_; }
  int port_dim() const { return ny_; }
  int state_offset(int i) const { return xoff_.at(i); }
  int input_offset(int i) const { return uoff_.at(i); }
  int port_offset(int i) const { return yoff_.at(i); }

  // The assembled pairing C (port_dim x port_dim, row-major). Skew by construction.
  const vecd &pairing() const { return C_; }

  double energy(const double *x) const;
  void effort(const double *x, double *e) const;
  void outputs(const double *x, double *y) const; // y_hat = B_hat^T e (port_dim)
  void field(const double *x, const double *u, double *dx) const;
  void coupling_field(const double *x, double *dx) const; // -B_hat C B_hat^T grad H
  power_terms powers(const double *x, const double *u) const;

  // The physical readout: every block's efforts in block order (state_dim values). For a DGU
  // these are its voltage and current, for a line its current.
  int readout_dim() const { return nx_; }
  void readout(const double *x, double *y) const { effort(x, y); }

private:
  std::vector<law_ptr> laws_;
  std::vector<std::string> names_;
  std::vector<int> xoff_, uoff_, yoff_;
  int nx_ = 0, nu_ = 0, ny_ = 0;
  vecd C_; // ny_ x ny_
  void grow_pairing(int new_ny);
};

// ── transitions ─────────────────────────────────────────────────────────────────────────────────

enum class coupling_scheme { euler, midpoint };
enum class step_scheme { split, rk4, implicit_midpoint };

struct transition_opts {
  double dt = 0.01;
  int substeps = 2;       // repetitions of E_{d/2} o L_d o E_{d/2}, d = dt / substeps
  int local_substeps = 1; // LocalStep's nsub inside L_d
  coupling_scheme coupling = coupling_scheme::midpoint;
  step_scheme scheme = step_scheme::split;
  solve_opts solve;
};

struct step_stats {
  int iterations = 0;    // the largest Picard count of the step
  bool converged = true; // false when some solve stopped at max_iter
};

// Advances x (state_dim) by one step of opts.dt with u (input_dim) held.
step_stats step(const composite &s, const transition_opts &opts, const double *u, double *x);
// The coupling half-step E_tau alone.
void coupling_step(const composite &s, coupling_scheme c, double tau, double *x);
// Whole-field references.
void step_rk4(const composite &s, double dt, const double *u, double *x);
step_stats step_implicit_midpoint(const composite &s, double dt, const double *u, double *x,
                                  const solve_opts &o);

// A step's books from its endpoints, u held: supplied = dt/2 (P_ext(x0) + P_ext(x1)), dissipated =
// dt/2 (P_loss(x0) + P_loss(x1)), residual = dH - supplied + dissipated.
books step_books(const composite &s, double dt, const double *x0, const double *x1,
                 const double *u);
// The same books by the midpoint rule (the powers at (x0 + x1)/2). For a quadratic H, implicit
// midpoint balances them exactly: its residual is the solver's tolerance alone.
books step_books_midpoint(const composite &s, double dt, const double *x0, const double *x1,
                          const double *u);

struct rollout_result {
  std::vector<vecd> x; // steps + 1 states, x[0] the start
  std::vector<books> ledger;
  std::vector<step_stats> stats;
};

// Rolls out `steps` steps from x0. u holds one input row (input_dim, held throughout) or `steps`
// rows (row k held over step k).
rollout_result rollout(const composite &s, const transition_opts &opts, const vecd &x0,
                       const vecd &u, int steps);

// ── checks ──────────────────────────────────────────────────────────────────────────────────────

// The power balance at x, u: |e . f(x, u) - (external - loss)| and |internal|, each
// relative to the largest power term (at least 1).
struct balance_check {
  double rate_error = 0;     // dH/dt from the field against -loss + external
  double internal_error = 0; // the pairing's net power
  double skew_error = 0;     // max |C + C^T|
};
balance_check check_balance(const composite &s, const double *x, const double *u);

// check_balance's worst case over `samples` random states and inputs (normal, standard deviation
// `scale`, from `seed`).
struct balance_summary {
  int samples = 0;
  balance_check worst;
};
balance_summary check_balance_random(const composite &s, int samples, unsigned seed,
                                     double scale = 1.0);

// ── reports ─────────────────────────────────────────────────────────────────────────────────────

// The blocks (name, kind, state, ports, inputs, offsets) and the pairing's nonzero entries.
std::string describe(const composite &s);
// A rollout's summary: the start and end energies, the books' totals and worst residual, the
// solves, and the end state by block.
std::string describe(const composite &s, const transition_opts &o, const rollout_result &r);
// One row per state: t, every coordinate (<block>.<i>), H, then the step's books (empty on row 0).
void write_csv(std::ostream &out, const composite &s, const transition_opts &o,
               const rollout_result &r);

// ── files ───────────────────────────────────────────────────────────────────────────────────────
// "cvc-ph-law/1": {"format", "kind": "linear" | "canonical", ...}; "cvc-ph-system/1": {"format",
// "laws": {name: law}, "blocks": [{"name", "law"}], "couplings": [{"type": "pair" | "incidence" |
// "skew", ...}]}. The fields are in docs/PH_COMPOSE.md. Both throw std::runtime_error on a file
// they cannot read.

law_ptr load_law(const std::string &path);
law_ptr parse_law(const std::string &json);
composite load_system(const std::string &path);
composite parse_system(const std::string &json);

const char *to_string(coupling_scheme c);
const char *to_string(step_scheme s);
coupling_scheme coupling_scheme_of(const std::string &s); // throws std::invalid_argument
step_scheme step_scheme_of(const std::string &s);

} // namespace ph
} // namespace cvc

#endif

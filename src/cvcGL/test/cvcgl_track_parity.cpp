// CameraController Track (push-fed) vs the pure-Python ChaseCamera algorithm.
//
// The "unify the cvcGL and grl-snam cameras so they work the same" claim (goal d)
// rests on the C++ Track mode reproducing pymod_gl/camera.py's ChaseCamera. This
// pins it NUMERICALLY: a reference implementation of that exact algorithm (the
// two-stage EMA — light position smoothing feeds a heavier velocity/heading EMA,
// then a critically-damped trailing pose) is driven with the SAME position stream
// as a push-fed Track controller, and their poses must agree to floating point.
// For Z-up with widen_tau = 0 they are line-for-line the same math, so any future
// divergence in Track breaks this test rather than silently drifting the demos.
//
// Headless: no vtkCamera / GL context — getPose() reads trackEye/trackFocal
// directly in Track mode, and resetTracking() with no camera leaves the first
// frame to SNAP, exactly as ChaseCamera does on its first update().
//
// cvcpkg builds Release (NDEBUG), so #undef it before <cassert> or the checks
// pass vacuously.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <string>

using cvc::gl::CameraController;

namespace {

// A faithful C++ transcription of pymod_gl/camera.py's ChaseCamera (Z-up), the
// reference the C++ Track controller must match.
struct RefChase {
  double back, height, lookAhead, lookUp, posTau, velTau, camTau, minSpeed;
  bool hasP = false, hasPrev = false, hasV = false, hasHead = false, hasEye = false;
  double p[3] = {0, 0, 0}, pprev[3] = {0, 0, 0}, v[3] = {0, 0, 0}, head[2] = {1, 0};
  double eye[3] = {0, 0, 0}, tgt[3] = {0, 0, 0};

  static double a(double dt, double tau) { return tau > 0.0 ? 1.0 - std::exp(-dt / tau) : 1.0; }

  void update(const double pos[3], double dt) {
    dt = std::max(dt, 1e-4);
    if (!hasP) {
      p[0] = pos[0];
      p[1] = pos[1];
      p[2] = pos[2];
      hasP = true;
    } else {
      const double f = a(dt, posTau);
      for (int i = 0; i < 3; ++i)
        p[i] += (pos[i] - p[i]) * f;
    }
    if (hasPrev) {
      double rawv[3];
      for (int i = 0; i < 3; ++i)
        rawv[i] = (p[i] - pprev[i]) / dt;
      if (!hasV) {
        for (int i = 0; i < 3; ++i)
          v[i] = rawv[i];
        hasV = true;
      } else {
        const double f = a(dt, velTau);
        for (int i = 0; i < 3; ++i)
          v[i] += (rawv[i] - v[i]) * f;
      }
    }
    for (int i = 0; i < 3; ++i)
      pprev[i] = p[i];
    hasPrev = true;
    if (hasV) {
      const double sp = std::hypot(v[0], v[1]);
      if (sp >= minSpeed) {
        head[0] = v[0] / sp;
        head[1] = v[1] / sp;
        hasHead = true;
      }
    }
    const double hx = hasHead ? head[0] : 1.0, hy = hasHead ? head[1] : 0.0;
    const double teye[3] = {p[0] - hx * back, p[1] - hy * back, p[2] + height};
    const double tlk[3] = {p[0] + hx * lookAhead, p[1] + hy * lookAhead, p[2] + lookUp};
    if (!hasEye) {
      for (int i = 0; i < 3; ++i) {
        eye[i] = teye[i];
        tgt[i] = tlk[i];
      }
      hasEye = true;
    } else {
      const double f = a(dt, camTau);
      for (int i = 0; i < 3; ++i) {
        eye[i] += (teye[i] - eye[i]) * f;
        tgt[i] += (tlk[i] - tgt[i]) * f;
      }
    }
  }
};

} // namespace

int main() {
  cvc::app app;

  // The C++ Track controller's defaults are ChaseCamera's defaults (back 55,
  // height 40, look_ahead 0, look_up 3, pos_tau 0.15, vel_tau 0.40, cam_tau 0.55,
  // min_speed 0.05); drive the reference with the same and widen_tau = 0.
  RefChase ref{55.0, 40.0, 0.0, 3.0, 0.15, 0.40, 0.55, 0.05};

  CameraController cc(app, "parity.cam");
  cc.setMode(CameraController::Mode::Track);
  cc.resetTracking(); // headless -> haveEye=false, first frame snaps like the ref

  // A continuous, varying-speed wander that dips toward a near-stop (exercising
  // the stop-safe heading hold) and never teleports, on irregular tick spacing.
  double phase = 0.0;
  double maxErr = 0.0;
  const int N = 200;
  for (int i = 0; i < N; ++i) {
    const double dt = 0.03 + 0.012 * std::sin(1.3 * i * 0.05);
    const double speed = 0.6 + 0.55 * std::cos(0.08 * i); // varies; dips low, stays >= 0
    phase += speed * dt * 0.25;
    const double pos[3] = {60.0 * std::cos(phase), 40.0 * std::sin(1.3 * phase),
                           3.0 + 2.0 * std::sin(0.5 * phase)};

    cc.feedTrackTarget(pos[0], pos[1], pos[2]);
    cc.update(dt);
    double e[3], f[3], u[3];
    cc.getPose(e, f, u);

    ref.update(pos, dt);

    for (int k = 0; k < 3; ++k) {
      maxErr = std::max(maxErr, std::abs(e[k] - ref.eye[k]));
      maxErr = std::max(maxErr, std::abs(f[k] - ref.tgt[k]));
    }
    // up axis is Z-up on both.
    assert(std::abs(u[0]) < 1e-12 && std::abs(u[1]) < 1e-12 && std::abs(u[2] - 1.0) < 1e-12);
  }

  printf("  max abs pose error over %d frames: %.3e\n", N, maxErr);
  assert(
      maxErr < 1e-9 &&
      "C++ Track diverged from the reference ChaseCamera — the cameras no longer 'work the same'");

  printf("cvcgl_track_parity: OK\n");
  return 0;
}

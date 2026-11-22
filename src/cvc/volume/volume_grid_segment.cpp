/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Segmentation tab's data side. See inc/cvc/volume/volume_grid_segment.h.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/segmentation_params.h>
#include <cvc/volume/volume_grid_segment.h>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <string_view>
#include <utility>

namespace cvc {

namespace {

std::string num(double d) { return param_detail::format_double(d); }

void put(state &s, const std::string &v) {
  if (s.value() != v)
    s.value(v);
}

std::string rgb_text(const std::array<double, 3> &c) {
  return num(c[0]) + "," + num(c[1]) + "," + num(c[2]);
}

// "r,g,b" (spaces allowed) as three numbers in [0, 1].
bool parse_rgb(const std::string &text, std::array<double, 3> &out) {
  std::vector<std::string> parts;
  std::string cur;
  for (char ch : text) {
    if (ch == ',') {
      parts.push_back(cur);
      cur.clear();
    } else if (ch != ' ' && ch != '\t') {
      cur += ch;
    }
  }
  parts.push_back(cur);
  if (parts.size() != 3)
    return false;
  std::array<double, 3> c{};
  for (int k = 0; k < 3; ++k)
    if (!param_detail::parse_double(parts[k], c[k]) || !(c[k] >= 0.0 && c[k] <= 1.0))
      return false;
  out = c;
  return true;
}

// The units of an inner step: small enough that a cancel ends a step soon.
constexpr std::size_t kPiece = 4096;

// The inner surfaces' phases, as this stepper names them (static strings).
const char *surface_phase(const char *inner) {
  const std::string_view p = inner ? inner : "";
  if (p == "checking")
    return "surfaces: checking";
  if (p == "reading")
    return "surfaces: reading";
  if (p == "extracting")
    return "surfaces: extracting";
  if (p == "smoothing")
    return "surfaces: smoothing";
  if (p == "normals")
    return "surfaces: normals";
  return "surfaces";
}

} // namespace

bool volume_grid_segment_session_key(const std::string &path) {
  const auto in = [&path](const std::string &base, const std::vector<std::string> &names) {
    for (const std::string &n : names)
      if (path == base + n)
        return true;
    return false;
  };
  return in("grid.segment.", param_names<seeded_segmentation_params>()) ||
         in("grid.segment.", param_names<volume_grid_segment_view_params>()) ||
         in("grid.segment.mesh.", param_names<class_surface_params>());
}

// ---- the stepper --------------------------------------------------------------------------------

struct volume_grid_segment_stepper::shared {
  std::atomic<bool> cancel{false};
  mutable std::mutex live_m;
  std::shared_ptr<const volume> live;
  std::atomic<std::uint64_t> live_v{0};
};

struct volume_grid_segment_stepper::impl {
  volume in;
  seeded_segmentation_params seg;
  class_surface_params surf;
  bool want_surfaces;
  double live_seconds;
  std::shared_ptr<shared> sh;

  bool finished = false, failed = false;
  std::exception_ptr error;
  double prog = 0.0;

  std::optional<segment_stepper> labels; // the first stage
  std::optional<class_surfaces_stepper> surfaces;
  std::optional<volume_grid_segment_result> result;

  std::chrono::steady_clock::time_point live_at{};
  bool live_started = false;

  impl(const volume &v, point_classes seeds, seeded_segmentation_params s, class_surface_params c,
       bool w, double ls, std::shared_ptr<shared> sh_)
      : in(v), seg(s), surf(std::move(c)), want_surfaces(w), live_seconds(ls), sh(std::move(sh_)) {
    labels.emplace(v, std::move(seeds), s);
  }
  bool cancelled() const { return sh->cancel.load(std::memory_order_acquire); }

  double share() const { return want_surfaces ? 0.75 : 1.0; }

  void take_live() {
    if (live_seconds < 0 || !labels)
      return;
    const auto now = std::chrono::steady_clock::now();
    if (live_started && std::chrono::duration<double>(now - live_at).count() < live_seconds)
      return;
    auto copy = std::make_shared<volume>(in.ctx());
    if (!labels->labels_so_far(*copy))
      return;
    live_at = now;
    live_started = true;
    {
      const std::lock_guard<std::mutex> hold(sh->live_m);
      sh->live = std::move(copy);
    }
    sh->live_v.fetch_add(1, std::memory_order_release);
  }
};

volume_grid_segment_stepper::volume_grid_segment_stepper(const volume &vol, point_classes seeds,
                                                         seeded_segmentation_params seg,
                                                         class_surface_params surf, bool surfaces,
                                                         double live_seconds)
    : _shared(std::make_shared<shared>()),
      _impl(std::make_unique<impl>(vol, std::move(seeds), seg, std::move(surf), surfaces,
                                   live_seconds, _shared)) {}

volume_grid_segment_stepper::~volume_grid_segment_stepper() = default;
volume_grid_segment_stepper::volume_grid_segment_stepper(volume_grid_segment_stepper &&) noexcept =
    default;
volume_grid_segment_stepper &
volume_grid_segment_stepper::operator=(volume_grid_segment_stepper &&) noexcept = default;

bool volume_grid_segment_stepper::step(std::size_t budget) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.failed) {
    if (m.cancelled())
      return true;
    std::rethrow_exception(m.error);
  }
  if (m.finished)
    return true;
  try {
    std::size_t left = budget == 0 ? std::numeric_limits<std::size_t>::max() : budget;
    while (left > 0 && !m.finished) {
      if (m.cancelled())
        break;
      const std::size_t piece = std::min(left, kPiece);
      if (m.labels) {
        const bool done = m.labels->step(piece);
        m.prog = std::max(m.prog, m.share() * m.labels->progress());
        if (done) {
          volume_grid_segment_result &r = m.result.emplace(m.in.ctx());
          r.segmentation = m.labels->take();
          m.labels.reset();
          if (m.want_surfaces)
            m.surfaces.emplace(r.segmentation.labels, m.surf);
          else
            m.finished = true;
        } else {
          m.take_live();
        }
      } else if (m.surfaces) {
        const bool done = m.surfaces->step(piece);
        m.prog = std::max(m.prog, m.share() + (1.0 - m.share()) * m.surfaces->progress());
        if (done) {
          m.result->surfaces = m.surfaces->take();
          m.surfaces.reset();
          m.finished = true;
        }
      }
      if (left != std::numeric_limits<std::size_t>::max())
        left -= piece;
    }
    if (m.cancelled() && !m.finished) {
      m.error = std::make_exception_ptr(segmentation_error("segment: cancelled"));
      m.failed = true;
      m.labels.reset();
      m.surfaces.reset();
      m.result.reset();
      return true;
    }
    if (m.finished)
      m.prog = 1.0;
    return m.finished;
  } catch (...) {
    m.error = std::current_exception();
    m.failed = true;
    m.labels.reset();
    m.surfaces.reset();
    throw;
  }
}

bool volume_grid_segment_stepper::done() const {
  return !_impl || _impl->finished || _impl->failed;
}

const char *volume_grid_segment_stepper::phase() const {
  if (!_impl)
    return "done";
  const impl &m = *_impl;
  if (m.failed)
    return "failed";
  if (m.finished)
    return "done";
  if (m.labels)
    return m.labels->phase();
  if (m.surfaces)
    return surface_phase(m.surfaces->phase());
  return "done";
}

double volume_grid_segment_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string volume_grid_segment_stepper::status() const {
  if (!_impl)
    return "done";
  const impl &m = *_impl;
  if (m.failed) {
    try {
      std::rethrow_exception(m.error);
    } catch (const std::exception &e) {
      return std::string("failed: ") + e.what();
    } catch (...) {
      return "failed";
    }
  }
  if (m.finished && m.result) {
    const seeded_segmentation &s = m.result->segmentation;
    std::size_t labelled = 0;
    for (std::size_t c : s.class_voxels)
      labelled += c;
    std::string out = "done: " + std::to_string(s.class_names.size()) + " classes, " +
                      std::to_string(labelled) + " voxels labelled, " +
                      std::to_string(s.border.size()) + " border voxels";
    if (m.want_surfaces)
      out += ", " + std::to_string(m.result->surfaces.labels.size()) + " surfaces (" +
             std::to_string(m.result->surfaces.triangle_count()) + " triangles)";
    return out;
  }
  if (m.labels)
    return m.labels->status();
  if (m.surfaces)
    return "surfaces: " + m.surfaces->status();
  return "done";
}

void volume_grid_segment_stepper::cancel() {
  if (_shared)
    _shared->cancel.store(true, std::memory_order_release);
}

volume_grid_segment_result volume_grid_segment_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished || !_impl->result)
    throw segmentation_error("segment: take() before the stepper is done");
  volume_grid_segment_result r = std::move(*_impl->result);
  _impl.reset();
  return r;
}

std::shared_ptr<const volume> volume_grid_segment_stepper::live_labels() const {
  if (!_shared)
    return nullptr;
  const std::lock_guard<std::mutex> hold(_shared->live_m);
  return _shared->live;
}

std::uint64_t volume_grid_segment_stepper::live_version() const {
  return _shared ? _shared->live_v.load(std::memory_order_acquire) : 0;
}

std::unique_ptr<volume_grid_segment_stepper>
volume_grid_segment_stepper_of(const volume &vol, point_classes seeds,
                               seeded_segmentation_params seg, class_surface_params surf,
                               bool surfaces, double live_seconds) {
  return std::make_unique<volume_grid_segment_stepper>(vol, std::move(seeds), seg, std::move(surf),
                                                       surfaces, live_seconds);
}

// ---- rows -------------------------------------------------------------------------------------

std::vector<segment_class_row> segment_class_rows(const point_classes &seeds, int variable,
                                                  int timestep, const seeded_segmentation &r,
                                                  const class_surface_result &surfaces) {
  std::vector<const point_class *> shown;
  for (const point_class &c : seeds.classes)
    if (c.variable == variable && c.timestep == timestep)
      shown.push_back(&c);
  const std::size_t n = r.class_names.size();
  // seed_mode::critical_points: its two classes come last, after the point classes'.
  const bool critical = n >= 2 && n == shown.size() + 2 && r.class_names[n - 2] == "feature" &&
                        r.class_names[n - 1] == "background";
  std::vector<segment_class_row> rows(n);
  for (std::size_t i = 0; i < n; ++i) {
    segment_class_row &row = rows[i];
    row.name = r.class_names[i];
    std::vector<double> col;
    if (critical && i == n - 2) {
      col = {1.0, 0.78, 0.2};
    } else if (critical && i == n - 1) {
      col = {0.3, 0.45, 0.9};
      row.visible = false;
    } else if (i < shown.size() && shown[i]->color.size() == 3) {
      col = shown[i]->color;
    } else {
      col = point_class_palette(i);
    }
    for (int k = 0; k < 3; ++k)
      row.color[k] = k < static_cast<int>(col.size()) ? std::clamp(col[k], 0.0, 1.0) : 0.0;
    row.voxels = i < r.class_voxels.size() ? r.class_voxels[i] : 0;
    if (const geometry *g = surfaces.surface_of(static_cast<int>(i)))
      row.triangles = g->tris().size();
  }
  return rows;
}

void write_segment_rows(state &rows, const std::vector<segment_class_row> &r) {
  const std::size_t n = r.size();
  for (std::size_t i = 0; i < n; ++i) {
    state &row = rows(std::to_string(i));
    put(row("name"), r[i].name);
    put(row("color"), rgb_text(r[i].color));
    put(row("visible"), r[i].visible ? "1" : "0");
    put(row("voxels"), std::to_string(r[i].voxels));
    put(row("triangles"), std::to_string(r[i].triangles));
  }
  for (const std::string &child : rows.childNames()) {
    long long i = 0;
    if (param_detail::parse_integer(child, i) && i >= static_cast<long long>(n))
      rows.removeChild(child);
  }
  put(rows, std::to_string(n)); // last: a watcher on the count sees the rows written
}

bool read_segment_rows(state &rows, std::vector<segment_class_row> &r,
                       std::vector<std::string> *problems) {
  bool changed = false;
  for (std::size_t i = 0; i < r.size(); ++i) {
    state *row = rows.findDescendant(std::to_string(i));
    if (!row)
      continue;
    if (state *c = row->findDescendant("color")) {
      std::array<double, 3> col = r[i].color;
      if (parse_rgb(c->value(), col)) {
        if (col != r[i].color) {
          r[i].color = col;
          changed = true;
        }
      } else if (problems) {
        problems->push_back("segment.rows." + std::to_string(i) + ".color: '" + c->value() +
                            "' is not r,g,b in [0, 1]");
      }
    }
    if (state *v = row->findDescendant("visible")) {
      bool b = r[i].visible;
      if (param_detail::parse_bool(v->value(), b)) {
        if (b != r[i].visible) {
          r[i].visible = b;
          changed = true;
        }
      } else if (problems) {
        problems->push_back("segment.rows." + std::to_string(i) + ".visible: '" + v->value() +
                            "' is not 1 or 0");
      }
    }
  }
  return changed;
}

// ---- the colour layer ---------------------------------------------------------------------------

std::vector<std::uint8_t> color_labels(const volume_slice_data &labels,
                                       const std::vector<segment_class_row> &rows, double opacity) {
  const double a = std::isfinite(opacity) ? std::clamp(opacity, 0.0, 1.0) : 0.0;
  const std::uint8_t alpha = static_cast<std::uint8_t>(std::lround(a * 255.0));
  // Each row's RGBA, looked up by label.
  std::vector<std::array<std::uint8_t, 4>> lut(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    for (int k = 0; k < 3; ++k)
      lut[i][k] =
          static_cast<std::uint8_t>(std::lround(std::clamp(rows[i].color[k], 0.0, 1.0) * 255.0));
    lut[i][3] = rows[i].visible ? alpha : 0;
  }
  const std::size_t w = labels.width, h = labels.height;
  std::vector<std::uint8_t> px(w * h * 4, 0);
  for (std::size_t row = 0; row < h; ++row) {
    // Image order: the slice's last row first.
    std::uint8_t *out = px.data() + (h - 1 - row) * w * 4;
    for (std::size_t col = 0; col < w; ++col) {
      const double v = labels.values[row * w + col];
      if (!(v >= 0.0) || v >= double(lut.size()) || v != std::floor(v))
        continue;
      const std::array<std::uint8_t, 4> &c = lut[static_cast<std::size_t>(v)];
      if (c[3] == 0)
        continue;
      std::copy(c.begin(), c.end(), out + col * 4);
    }
  }
  return px;
}

} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// tf_binding.cpp -- see tf_binding.h.
//
// Threading. The binding has no thread of its own. Everything here runs on the render thread
// (tick, the constructor and destructor, wait_overlays) except the overlay tasks' deliver hooks,
// which run on whatever thread pumps app.exec_scheduler() -- the render thread in every Ariadne
// host, but the handoff is mutex-guarded anyway. A hook captures only its task's overlay_mailbox,
// never the binding: a task may outlive the binding (until the destructor's cancel_overlays lets
// go of the hook), so a hook holding the binding could not tell it was gone.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/mesh_nodes.h> // fe_mesh_signature: what fe_lab's last mesh job meshed
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/state/state.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/volren/types.h>
#include <exception>
#include <iterator>
#include <limits>
#include <map>
#include <mutex>
#include <tuple>
#include <utility>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

namespace ari = cvc::ariadne;

// --- the overlay tasks' handoff -----------------------------------------------------------------

// Where one overlay task's deliver hook (cvc::tf::overlay_task_options::deliver) leaves the raw
// overlays for the binding to take in. The hook holds the only other reference, and the task lets
// go of it when its process ends (published, cancelled or killed).
struct overlay_mailbox {
  std::mutex m;
  bool delivered = false; // guarded by m, as is ov
  cvc::tf::overlays ov;
};

// One overlay task the binding started (cvc::tf::schedule_overlays) and has not taken in yet.
struct overlay_task {
  int pid = -1;
  unsigned bits = 0;
  unsigned generation = 0; // the volume it computes from (see impl::generation)
  std::shared_ptr<overlay_mailbox> mailbox;
};

// The scheduler owner tag of a binding's tasks, unique to the binding ("tf#<n>:<model key>"), so
// a kill_owner of it stops exactly this binding's tasks.
std::string task_owner(const std::string &model_key) {
  static std::atomic<unsigned long long> seq{0};
  return "tf#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed)) + ":" + model_key;
}

// --- small numeric helpers ----------------------------------------------------------------------

bool same_range(double a0, double a1, double b0, double b1) {
  const double tol = 1e-12 * std::max({1.0, std::fabs(a0), std::fabs(a1)});
  return std::fabs(a0 - b0) <= tol && std::fabs(a1 - b1) <= tol;
}

// Two isovalues over the domain [lo, hi] that are one isocontour position. An isovalue v becomes
// the position (v - lo) / (hi - lo) and lowers back as lo + (hi - lo) t, which can miss v by ulps
// of the range's ends -- 0.37 over [-1e5, 1e5] comes back 4.7e-12 off -- and a model string
// written with positions to 9 significant digits moves it by up to 5e-10 of the span. Within 1e-9
// of the span (plus that rounding) it is the same isovalue: not a move, and not worth a
// seconds-long remesh.
bool same_isovalue(double a, double b, double lo, double hi) {
  const double tol = 1e-9 * std::fabs(hi - lo) + 8.0 * std::numeric_limits<double>::epsilon() *
                                                     std::max(std::fabs(lo), std::fabs(hi));
  return std::fabs(a - b) <= tol;
}

// A state CSV of exactly `n` numbers.
bool parse_n(const std::string &s, std::size_t n, std::vector<double> &out) {
  return !s.empty() && cvc::tf::parse_csv(s, out) && out.size() == n;
}

// Histogram bins uniform over [a0, a1] re-binned onto [b0, b1] (the same count): each source bin's
// count spread over the target bins it overlaps, so the total over the shared span is kept.
std::vector<double> rebin(const std::vector<double> &bins, double a0, double a1, double b0,
                          double b1) {
  const std::size_t n = bins.size();
  std::vector<double> out(n, 0.0);
  if (n == 0 || !(a1 > a0) || !(b1 > b0))
    return out;
  const double wa = (a1 - a0) / double(n), wb = (b1 - b0) / double(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double s0 = a0 + double(i) * wa, s1 = s0 + wa;
    const double f0 = (s0 - b0) / wb, f1 = (s1 - b0) / wb;
    if (f1 <= 0.0 || f0 >= double(n))
      continue;
    const std::size_t j0 = static_cast<std::size_t>(std::max(0.0, std::floor(f0)));
    const std::size_t j1 =
        std::min(n - 1, static_cast<std::size_t>(std::max(0.0, std::ceil(f1) - 1.0)));
    for (std::size_t j = j0; j <= j1; ++j) {
      const double t0 = b0 + double(j) * wb;
      const double overlap = std::min(s1, t0 + wb) - std::max(s0, t0);
      if (overlap > 0.0)
        out[j] += bins[i] * overlap / wa;
    }
  }
  return out;
}

// Samples uniform over [a0, a1] (sample i at a0 + i (a1 - a0) / (n - 1)) resampled at n points over
// [b0, b1], held at the end values outside the source span.
std::vector<double> resample(const std::vector<double> &v, double a0, double a1, double b0,
                             double b1) {
  const std::size_t n = v.size();
  if (n < 2 || !(a1 > a0))
    return v;
  std::vector<double> out(n);
  for (std::size_t j = 0; j < n; ++j) {
    const double x = b0 + (b1 - b0) * double(j) / double(n - 1);
    const double f = std::clamp((x - a0) / (a1 - a0) * double(n - 1), 0.0, double(n - 1));
    const std::size_t i = std::min(n - 2, static_cast<std::size_t>(f));
    const double t = f - double(i);
    out[j] = v[i] * (1.0 - t) + v[i + 1] * t;
  }
  return out;
}

// Flat tables of `stride` doubles per point: a point at the same scalar as the one before it (a
// step in the model) moves up by a hair, because state_settings' ramp merge and VTK both collapse
// duplicate scalars into one point and would flatten the step.
void separate_steps(std::vector<double> &t, std::size_t stride, double span) {
  const double eps = std::max(std::fabs(span), 1e-300) * 1e-9;
  for (std::size_t i = stride; i < t.size(); i += stride)
    if (t[i] <= t[i - stride])
      t[i] = std::max(t[i - stride] + eps,
                      std::nextafter(t[i - stride], std::numeric_limits<double>::infinity()));
}

// "contour_spectrum,contour_tree" -> "contour spectrum, contour tree".
std::string overlay_names(unsigned bits) {
  std::string s;
  for (char c : cvc::tf::overlay_bits_to_csv(bits)) {
    if (c == ',')
      s += ", ";
    else
      s += c == '_' ? ' ' : c;
  }
  return s;
}

// --- the live-binding registry (find_transfer_function) ------------------------------------------

using registry_key = std::pair<const cvc::app *, std::string>;

std::mutex &registry_mutex() {
  static std::mutex m;
  return m;
}

std::map<registry_key, std::weak_ptr<tf_binding>> &registry() {
  static std::map<registry_key, std::weak_ptr<tf_binding>> r;
  return r;
}

void prune_registry() {
  auto &r = registry();
  for (auto it = r.begin(); it != r.end();)
    it = it->second.expired() ? r.erase(it) : std::next(it);
}

// The seed each model key was last given in this process. A re-realize (a document hot-reload)
// finds the key still holding the previous seed when nobody edited it: the document's new
// transfer function then replaces it, exactly as the renderer's own keys are re-configured.
// A key that differs was edited (or restored from a saved session) and is kept.
std::map<registry_key, std::string> &seed_memory() {
  static std::map<registry_key, std::string> m;
  return m;
}

// Whose <tf>.overlay_status it is, per model key: the owner tag of the newest binding for the key.
// A hot reload builds the new binding (whose histogram task says "computing histogram...") before
// the old one is destroyed, so a binding that stops for good leaves the status idle only while it
// is still its own. Guarded by registry_mutex().
std::map<registry_key, std::string> &status_owners() {
  static std::map<registry_key, std::string> m;
  return m;
}

} // namespace

// --- the binding ---------------------------------------------------------------------------------

struct tf_binding::impl {
  std::weak_ptr<GraphicsNode> node;
  cvc::app *app = nullptr;
  tf_target target = tf_target::volren;
  tf_binding_options opt;
  std::string base;      // the node's state path
  std::string model_key; // base + ".transfer_function"
  std::string seed_str;  // to_string(opt.seed)
  std::string name;      // the node's name, for warnings

  // The model: the string last read, its last good parse, and the string last lowered.
  std::string seen;
  cvc::tf::model model;
  bool model_ok = false;
  bool dirty = false;
  std::string applied;

  // The `auto` domain as last published at <tf>.range.
  bool has_range = false;
  double range_lo = 0.0, range_hi = 1.0;
  std::string range_sig;

  // The volume (a copy-on-write copy; each overlay task keeps a shallow copy of it) and its range.
  cvc::volume vol;
  bool vol_has_range = false;
  double vol_lo = 0.0, vol_hi = 1.0;

  // volslice: the window this binding wrote for an explicit domain (owns_window while the key still
  // holds it), so going back to `auto` clears only its own window. Once anyone else changes the
  // window the binding yields it for good: it never writes over a window a script, a control or
  // a peer set.
  bool owns_window = false;
  bool window_yielded = false;
  double window_lo = 0.0, window_hi = 0.0;

  // volume: the node's volume as last followed, by its shared owner (VolumeNode::getSharedVolume;
  // fe_lab: opt.volume_source) -- never by address: a setVolume can be handed the address of the
  // volume it replaced, and the weak_ptr keeps that block (so its address) from being reused while
  // the binding holds it. has_volume: false while a volume_source has none yet (no range, no
  // overlay task).
  std::weak_ptr<const cvc::volume> volume_seen;
  bool has_volume = true;

  // fe_lab: the ids of the isocontours fe.mesh.isovalue (fe_isocontour) and, in interval mode,
  // fe.mesh.isovalue2 (fe_isocontour2) follow; each key as last read or written; fe.mesh.region as
  // last read (fe_interval: it says interval); and the remesh debounce.
  std::string fe_iso_id, fe_iso2_id;
  std::string iso_seen, iso2_seen, region_seen;
  bool fe_interval = false;
  bool remesh_pending = false;
  std::chrono::steady_clock::time_point iso_changed{};

  // Overlays, computed by tasks on the app's state_exec scheduler (cvc/tf/overlay_tasks.h). `raw`
  // holds everything computed, over the volume's own range (raw.range_*); it is re-binned onto
  // <tf>.range whenever that differs. inflight = the bits the live tasks compute.
  std::string owner;               // the tasks' scheduler owner tag (task_owner)
  std::vector<overlay_task> tasks; // started, not yet taken in
  unsigned generation = 0;         // bumped when the volume is replaced: older results are dropped
  unsigned computed = 0, inflight = 0, requested = 0;
  std::string request_seen;
  cvc::tf::overlays raw;
  // Why layers of the current volume are missing, by the bits of the task that left them so: its
  // final status ("" or "unavailable: ..."), "unavailable: the overlay task was stopped", or why it
  // did not start. One volume's tasks compute disjoint bits, so a later task never erases an
  // earlier one's reason (a layer is not retried: the request only grows); cleared with the volume.
  std::map<unsigned, std::string> notes;
  bool retired = false; // stopped for good (retire)

  explicit impl(const cvc::volume &v) : vol(v) {}

  std::string key(const std::string &rel) const { return base + cvc::state::SEPARATOR + rel; }
  std::string tf_key(const std::string &rel) const {
    return model_key + cvc::state::SEPARATOR + rel;
  }

  void read_volume_range() {
    vol_has_range = false;
    if (!has_volume)
      return;
    // min()/max() fill the volume's cached range here; the overlay tasks never write the cache.
    try {
      vol_lo = vol.min();
      vol_hi = vol.max();
      vol_has_range = std::isfinite(vol_lo) && std::isfinite(vol_hi);
    } catch (...) {
      vol_has_range = false;
    }
  }

  // --- overlays ---

  // Start a task computing `bits` from the current volume. Its results come back through its
  // mailbox (take_results); it writes its own progress notes to <tf>.overlay_status meanwhile.
  void submit(unsigned bits) {
    if (!bits || !app || !has_volume)
      return; // (with no volume yet, follow_volume submits them when one arrives)
    auto mailbox = std::make_shared<overlay_mailbox>();
    cvc::tf::overlay_task_options o;
    o.overlays = opt.overlays;
    o.threading = opt.overlay_threading;
    o.owner = owner;
    o.deliver = [mailbox](cvc::tf::overlays &&ov) {
      std::lock_guard<std::mutex> lock(mailbox->m);
      mailbox->ov = std::move(ov);
      mailbox->delivered = true;
    };
    const int pid = cvc::tf::schedule_overlays(*app, model_key, vol, bits, o);
    if (pid < 0) {
      // It could not start (schedule_overlays wrote why to overlay_status). Marked computed so it
      // is not retried every frame; a replaced volume tries again.
      computed |= bits;
      std::string &note = notes[bits];
      note = ari::read_or<std::string>(*app, tf_key("overlay_status"), "");
      if (note.empty())
        note = "unavailable: the overlay task did not start";
      return;
    }
    tasks.push_back({pid, bits, generation, std::move(mailbox)});
    inflight |= bits;
    // Said once, so the status is right before the task's first step; from then on the tasks'
    // own progress notes replace it.
    ari::write<std::string>(*app, tf_key("overlay_status"),
                            "computing " + overlay_names(inflight) + "...");
  }

  // Stop every live task (the volume was replaced, or the binding is going away): none of them
  // delivers anything more, their processes are reaped and their jobs let go of.
  void cancel_tasks() {
    for (const overlay_task &t : tasks) {
      try {
        cvc::tf::cancel_overlays(*app, t.pid);
      } catch (...) {
      }
    }
    tasks.clear();
    inflight = 0;
  }

  // Stop for good: the binding is going away, or its node is gone. The live tasks are cancelled
  // (nobody would take their results in), and <tf>.overlay_status, while it is still this
  // binding's (no newer binding took the key over) and a task's note was in it, is left idle
  // rather than a progress note nobody will finish. Once; render thread.
  void retire() {
    if (retired || !app)
      return;
    retired = true;
    const bool had_tasks = !tasks.empty();
    cancel_tasks();
    bool mine = false;
    {
      std::lock_guard<std::mutex> lock(registry_mutex());
      auto &owners = status_owners();
      const auto it = owners.find({app, model_key});
      if (it != owners.end() && it->second == owner) {
        owners.erase(it);
        mine = true;
      }
    }
    if (mine && had_tasks) // written outside the lock: the write runs state handlers
      ari::write<std::string>(*app, tf_key("overlay_status"), idle_status());
  }

  // The idle <tf>.overlay_status: every reason in `notes` as one "unavailable: a; b" note
  // (note_unavailable's form), "" when nothing is missing.
  std::string idle_status() const {
    static const std::string head = "unavailable: ";
    std::string s;
    for (const auto &n : notes) {
      if (n.second.empty())
        continue;
      s += s.empty() ? head : "; ";
      s += n.second.compare(0, head.size(), head) == 0 ? n.second.substr(head.size()) : n.second;
    }
    return s;
  }

  // Unpublish every layer beside the model key: they were computed from a volume the binding no
  // longer follows (a replaced one, or a predecessor's), and publish_overlays never clears a layer.
  void clear_layers() {
    try {
      cvc::tf::clear_overlays(cvc::state::instance(*app)(model_key));
    } catch (const std::exception &) {
    }
  }

  // Publish the members `bits` names from `raw`, on the published range.
  void publish(unsigned bits) {
    if (!bits || !app)
      return;
    cvc::tf::overlays out; // has_range stays false: <tf>.range is refresh_range()'s
    // publish_overlays always writes the status: while a task runs, the note there is the task's
    // (left as it is); idle, why any layer is missing.
    out.status = tasks.empty() ? idle_status()
                               : ari::read_or<std::string>(*app, tf_key("overlay_status"), "");
    const bool remap =
        has_range && raw.has_range && !same_range(raw.range_lo, raw.range_hi, range_lo, range_hi);
    if (bits & cvc::tf::overlay_histogram)
      out.histogram = remap ? rebin(raw.histogram, raw.range_lo, raw.range_hi, range_lo, range_hi)
                            : raw.histogram;
    if (bits & cvc::tf::overlay_contour_spectrum) {
      out.spectrum = raw.spectrum;
      if (remap) {
        const auto rs = [&](const std::vector<double> &v) {
          return resample(v, raw.range_lo, raw.range_hi, range_lo, range_hi);
        };
        out.spectrum.area = rs(raw.spectrum.area);
        out.spectrum.min_volume = rs(raw.spectrum.min_volume);
        out.spectrum.max_volume = rs(raw.spectrum.max_volume);
        out.spectrum.gradient = rs(raw.spectrum.gradient);
      }
    }
    if (bits & cvc::tf::overlay_contour_tree)
      out.tree = raw.tree; // vertex values are data units already
    try {
      cvc::tf::publish_overlays(cvc::state::instance(*app)(model_key), out);
    } catch (const std::exception &) {
    }
  }

  // Take in what the tasks delivered, and the tasks that ended without delivering (stopped by a
  // kill or kill_owner of someone else's), then publish the layers that arrived.
  void take_results() {
    unsigned arrived = 0;
    for (auto it = tasks.begin(); it != tasks.end();) {
      // Asked before the mailbox is read: a task that has ended delivered, if at all, before it
      // ended, so `delivered` below is final whenever `running` is false.
      const bool running = cvc::tf::overlays_running(*app, it->pid);
      bool delivered = false;
      cvc::tf::overlays ov;
      {
        std::lock_guard<std::mutex> lock(it->mailbox->m);
        if (it->mailbox->delivered) {
          delivered = true;
          ov = std::move(it->mailbox->ov);
        }
      }
      if (!delivered && running) {
        ++it;
        continue;
      }
      const unsigned bits = it->bits, gen = it->generation;
      it = tasks.erase(it);
      if (gen != generation)
        continue; // computed from a volume the node has since replaced
      inflight &= ~bits;
      // Marked computed even when a layer came back empty (no mesher for the spectrum) or the
      // task was stopped: the status says why, and the request must not recompute it every frame.
      computed |= bits;
      if (!delivered) {
        notes[bits] = "unavailable: the overlay task was stopped";
        continue;
      }
      if (bits & cvc::tf::overlay_histogram)
        raw.histogram = std::move(ov.histogram);
      if (bits & cvc::tf::overlay_contour_spectrum)
        raw.spectrum = std::move(ov.spectrum);
      if (bits & cvc::tf::overlay_contour_tree)
        raw.tree = std::move(ov.tree);
      if (ov.has_range) {
        raw.has_range = true;
        raw.range_lo = ov.range_lo;
        raw.range_hi = ov.range_hi;
      }
      notes[bits] = ov.status;
      arrived |= bits;
    }
    publish(arrived);
  }

  // Idle (no task running), <tf>.overlay_status says why any layer is missing (idle_status). While
  // a task runs it is the task's to write (its progress notes), so it is left alone.
  void write_idle_status() {
    if (tasks.empty())
      ari::write<std::string>(*app, tf_key("overlay_status"), idle_status());
  }

  void service_overlays() {
    take_results();
    const std::string req = ari::read_or<std::string>(*app, tf_key("overlay_request"), "");
    if (req != request_seen) {
      request_seen = req;
      requested = cvc::tf::overlay_bits_from_csv(req);
    }
    submit(requested & ~(computed | inflight));
    write_idle_status();
  }

  // --- the auto domain ---

  // The volume to follow now, by its shared owner: a VolumeNode's (volume), opt.volume_source's
  // (fe_lab), else null (`vol` for good).
  std::shared_ptr<const cvc::volume> current_volume(GraphicsNode &n) const {
    if (target == tf_target::volume) {
      auto *vn = dynamic_cast<VolumeNode *>(&n);
      return vn ? vn->getSharedVolume() : nullptr;
    }
    return opt.volume_source ? opt.volume_source() : nullptr;
  }

  // volume: follow a reload (setVolume); fe_lab: a load (opt.volume_source). Asked every tick,
  // whatever data_min/data_max say -- a new volume with the same range (the next time step)
  // re-publishes the same strings -- and by the shared owner of the volume, which a later volume
  // can never alias. The old volume's layers are unpublished at once and recomputed from the new
  // one.
  void follow_volume(GraphicsNode &n) {
    const std::shared_ptr<const cvc::volume> now = current_volume(n);
    if (!now || (!volume_seen.owner_before(now) && !now.owner_before(volume_seen)))
      return;
    volume_seen = now;
    has_volume = true;
    cancel_tasks(); // computing from the volume the node no longer draws
    vol = *now;
    read_volume_range();
    ++generation;
    inflight = computed = 0;
    raw = cvc::tf::overlays();
    notes.clear();
    clear_layers(); // never drawn on the new range, nor left beside a layer the new one lacks
    submit(cvc::tf::overlay_histogram | requested);
  }

  // The `auto` domain the renderer uses now; publishes <tf>.range (and re-bins the overlays onto
  // it) when it moved. Returns whether it moved.
  bool refresh_range(GraphicsNode &n) {
    bool has = false;
    double lo = 0.0, hi = 1.0;
    std::vector<double> v;
    switch (target) {
    case tf_target::volren:
    case tf_target::fe_lab:
    case tf_target::volslice:
      if (opt.volume_source)
        follow_volume(n); // fe_lab: the volume its last load brought
      // volslice too: its window is the slice LUT's span, not the transfer function's domain.
      // VolSliceNode samples the raw-unit tables over whatever window it has, so the TF stays
      // fixed in data units when the window moves -- the legacy renderer's semantics.
      has = vol_has_range;
      lo = vol_lo;
      hi = vol_hi;
      break;
    case tf_target::volume: {
      follow_volume(n);
      const std::string a = ari::read_or<std::string>(*app, key("data_min"), "");
      const std::string b = ari::read_or<std::string>(*app, key("data_max"), "");
      if (parse_n(a + "," + b, 2, v)) {
        has = true;
        lo = v[0];
        hi = v[1];
      }
      break;
    }
    }
    if (!has)
      return false;
    const std::string s = cvc::tf::range_to_string(lo, hi);
    if (s == range_sig)
      return false;
    range_sig = s;
    has_range = true;
    range_lo = lo;
    range_hi = hi;
    ari::write<std::string>(*app, tf_key("range"), s);
    publish(computed); // every published layer stays uniform over the range beside it
    return true;
  }

  // --- the model ---

  // Read the model key, seeding it when empty; parse it when it changed. A string that does not
  // parse keeps the last good model (and the renderer what it draws).
  bool read_model(std::string *error = nullptr) {
    std::string cur = ari::read_or<std::string>(*app, model_key, "");
    if (cur.empty()) {
      ari::write<std::string>(*app, model_key, seed_str);
      cur = ari::read_or<std::string>(*app, model_key, seed_str);
    }
    if (cur == seen)
      return true;
    seen = cur;
    cvc::tf::model m;
    if (!cvc::tf::from_string(cur, m, error))
      return false;
    model = std::move(m);
    model_ok = true;
    dirty = cur != applied;
    return true;
  }

  void write_tables(const std::string &color_key, const std::string &opacity_key, double lo,
                    double hi) {
    std::vector<double> color = cvc::tf::color_table(model, lo, hi);
    std::vector<double> opacity = cvc::tf::opacity_table(model, lo, hi);
    separate_steps(color, 4, hi - lo);
    separate_steps(opacity, 2, hi - lo);
    ari::write<std::string>(*app, color_key, cvc::tf::format_csv(color));
    ari::write<std::string>(*app, opacity_key, cvc::tf::format_csv(opacity));
  }

  // Lower the model into the renderer's keys -- once per change. volren's state_settings re-reads
  // every key on each write, all-or-nothing, so the order keeps every intermediate snapshot
  // well-formed (the tables, then the domain flag, then the isosurfaces); its raycast picks up the
  // final one on the node's next tick.
  // volslice: the window is the slice LUT's span (VolSliceNode samples the raw-unit tables over
  // it), not the transfer function's domain. An explicit domain is written as the window, so the
  // LUT's 256 entries cover it, while the window is this binding's to write: unset (and never
  // taken from it), or still what it last wrote. A window anyone else set or changed is theirs:
  // the binding yields it for good and never writes it again, and the TF, fixed in data units,
  // is drawn through their window. Going back to `auto` clears the window only while it is ours.
  void lower_window() {
    cvc::app &a = *app;
    const std::string k = key("volslice.window");
    std::vector<double> w;
    const bool set = parse_n(ari::read_or<std::string>(a, k, ""), 2, w) && w[1] > w[0];
    if (owns_window && !(set && same_range(w[0], w[1], window_lo, window_hi))) {
      owns_window = false; // changed (or cleared) by someone else since this binding wrote it
      window_yielded = true;
    }
    if (!model.domain.automatic) {
      if (owns_window || (!set && !window_yielded)) {
        if (!(set && same_range(w[0], w[1], model.domain.min, model.domain.max)))
          ari::write<std::string>(a, k, cvc::tf::format_csv({model.domain.min, model.domain.max}));
        owns_window = true;
        window_lo = model.domain.min;
        window_hi = model.domain.max;
      }
    } else if (owns_window) {
      ari::write<std::string>(a, k, std::string());
      owns_window = false;
    }
  }

  // --- fe_lab ---

  // The lowest isocontour whose id is not `skip` (any, when `skip` is empty), or -1.
  static int lowest_isocontour(const cvc::tf::model &m, const std::string &skip) {
    int i = -1;
    for (std::size_t j = 0; j < m.isocontours.size(); ++j)
      if ((skip.empty() || m.isocontours[j].id != skip) &&
          (i < 0 || m.isocontours[j].at < m.isocontours[std::size_t(i)].at))
        i = int(j);
    return i;
  }

  // fe_lab: the index of the isocontour fe.mesh.isovalue follows, or -1 when the model has none.
  // It is followed by id -- another isocontour passing it never swaps which one drives the mesh --
  // and when that id is gone (deleted) the lowest isocontour is adopted in its place (in interval
  // mode the lowest other than the second bound's, so the two never share one).
  int fe_isocontour(const cvc::tf::model &m) {
    int i = fe_iso_id.empty() ? -1 : m.find_isocontour(fe_iso_id);
    if (i < 0) {
      i = lowest_isocontour(m, fe_interval ? fe_iso2_id : std::string());
      if (i >= 0)
        fe_iso_id = m.isocontours[std::size_t(i)].id;
    }
    return i;
  }

  // fe_lab, interval mode: the index of the isocontour fe.mesh.isovalue2 follows, or -1 (always in
  // solid mode). By id like the first; a missing one -- or one the first bound has taken -- adopts
  // the lowest isocontour other than the first bound's. Call fe_isocontour first.
  int fe_isocontour2(const cvc::tf::model &m) {
    if (!fe_interval)
      return -1;
    int i = fe_iso2_id.empty() || fe_iso2_id == fe_iso_id ? -1 : m.find_isocontour(fe_iso2_id);
    if (i < 0) {
      i = lowest_isocontour(m, fe_iso_id);
      if (i >= 0)
        fe_iso2_id = m.isocontours[std::size_t(i)].id;
    }
    return i;
  }

  // fe_lab, interval mode: give each bound an isocontour of its own. One the model lacks is added
  // (next_isocontour_id) at that bound's current isovalue, so the mesh's interval is kept, and the
  // model is written. Returns whether it added any.
  bool fe_ensure_pair() {
    if (!fe_interval || !model_ok)
      return false;
    double lo = 0.0, hi = 1.0;
    cvc::tf::effective_domain(model, has_range, range_lo, range_hi, lo, hi);
    cvc::tf::model m = model;
    bool added = false;
    const auto add = [&](const char *rel, std::string &id) {
      const double v =
          ari::read_or<double>(*app, key(rel), std::numeric_limits<double>::quiet_NaN());
      cvc::tf::isocontour iso;
      iso.id = m.next_isocontour_id();
      iso.at = std::isfinite(v) ? std::clamp(cvc::tf::to_normalized(v, lo, hi), 0.0, 1.0) : 0.5;
      m.isocontours.push_back(iso);
      id = iso.id;
      added = true;
    };
    if (fe_isocontour(m) < 0)
      add("fe.mesh.isovalue", fe_iso_id);
    if (fe_isocontour2(m) < 0)
      add("fe.mesh.isovalue2", fe_iso2_id);
    if (!added)
      return false;
    m.normalize(); // re-sorts; the ids are fresh, so they are kept
    const std::string ms = cvc::tf::to_string(m);
    ari::write<std::string>(*app, model_key, ms);
    seen = applied = ari::read_or<std::string>(*app, model_key, ms);
    model = std::move(m);
    dirty = false;
    return true;
  }

  // fe_lab: whether isocontour `i`'s bar already stands for the isovalue `v` over [lo, hi]: the
  // same isovalue (within the position <-> data-value round trip), or -- the model holds positions
  // in [0, 1] only -- a bar pinned at an end of the axis for a bound at or past that end. A bound
  // outside the volume's range is legal (`interval: [0.5, 2]` on a 0/1 mask: 0.5 and up), shows
  // as its bar at that end, and keeps its true value: neither side is rewritten, no remesh runs.
  bool bar_holds(int i, double v, double lo, double hi) const {
    const double at = model.isocontours[std::size_t(i)].at;
    return same_isovalue(cvc::tf::isovalues(model, lo, hi)[std::size_t(i)], v, lo, hi) ||
           (at >= 1.0 && v >= hi) || (at <= 0.0 && v <= lo);
  }

  // fe_lab: write the followed isocontour `i`'s isovalue to `rel` when it moved (while its bar
  // still stands for the key's value it has not: a slider's exact value is kept, and no remesh
  // runs for nothing), arming the debounced remesh when `arm`.
  void fe_sync_bound(int i, const char *rel, std::string &seen_str, double lo, double hi,
                     bool arm) {
    if (i < 0)
      return; // no isocontour: the mesh keeps the last isovalue
    if (!has_volume)
      return; // no volume yet, so no axis: the bars are placed from the keys when it arrives
    const double iv = cvc::tf::isovalues(model, lo, hi)[std::size_t(i)];
    const std::string k = key(rel);
    const double cur = ari::read_or<double>(*app, k, std::numeric_limits<double>::quiet_NaN());
    if (!bar_holds(i, cur, lo, hi)) {
      ari::write<double>(*app, k, iv);
      if (arm) {
        remesh_pending = true;
        iso_changed = std::chrono::steady_clock::now();
      }
    }
    seen_str = ari::read_or<std::string>(*app, k, "");
  }

  // fe_lab: the model -> fe.mesh.isovalue (and, in interval mode, fe.mesh.isovalue2).
  void lower_fe(bool arm) {
    fe_ensure_pair();
    double lo = 0.0, hi = 1.0;
    cvc::tf::effective_domain(model, has_range, range_lo, range_hi, lo, hi);
    fe_sync_bound(fe_isocontour(model), "fe.mesh.isovalue", iso_seen, lo, hi, arm);
    fe_sync_bound(fe_isocontour2(model), "fe.mesh.isovalue2", iso2_seen, lo, hi, arm);
    publish_interval();
  }

  // fe_lab: <tf>.interval, the isocontour pair the mesher uses ("<isovalue's id>,<isovalue2's
  // id>"), or "" outside interval mode. Written only when it changes.
  void publish_interval() {
    std::string v;
    if (fe_interval && model_ok) {
      const int a = fe_isocontour(model), b = fe_isocontour2(model);
      if (a >= 0 && b >= 0)
        v = model.isocontours[std::size_t(a)].id + "," + model.isocontours[std::size_t(b)].id;
    }
    const std::string k = tf_key("interval");
    if (ari::read_or<std::string>(*app, k, "") != v)
      ari::write<std::string>(*app, k, v);
  }

  void lower() {
    cvc::app &a = *app;
    if (target == tf_target::volslice)
      lower_window();
    double lo = 0.0, hi = 1.0;
    cvc::tf::effective_domain(model, has_range, range_lo, range_hi, lo, hi);
    switch (target) {
    case tf_target::volren: {
      const std::string v0 = "volren.volumes.0.";
      write_tables(key(v0 + "transfer_function.color"), key(v0 + "transfer_function.opacity"), lo,
                   hi);
      ari::write<int>(a, key(v0 + "tf_auto_domain"), model.domain.automatic ? 1 : 0);
      std::vector<double> iso;
      iso.reserve(model.isocontours.size() * 6);
      for (const cvc::tf::isocontour &ic : model.isocontours) {
        float rgb[3];
        model.isocontour_rgb(ic, rgb);
        const auto sh = opt.shininess.find(ic.id);
        iso.insert(
            iso.end(),
            {cvc::tf::to_data(ic.at, lo, hi), double(model.isocontour_opacity(ic)), double(rgb[0]),
             double(rgb[1]), double(rgb[2]),
             double(sh != opt.shininess.end() ? sh->second : cvc::volren::defaults::shininess)});
      }
      ari::write<std::string>(a, key(v0 + "isosurfaces"), cvc::tf::format_csv(iso));
      break;
    }
    case tf_target::volslice:
      ari::write<int>(a, key("volslice.tf_auto_domain"), model.domain.automatic ? 1 : 0);
      write_tables(key("volslice.transfer_function.color"),
                   key("volslice.transfer_function.opacity"), lo, hi);
      break;
    case tf_target::volume:
      write_tables(key("transfer_function.color"), key("transfer_function.opacity"), lo, hi);
      break;
    case tf_target::fe_lab:
      lower_fe(true); // an edit (an editor's drag) re-meshes once it settles
      break;
    }
    applied = seen;
    dirty = false;
  }

  // fe_lab: a bound's isovalue someone else wrote (fe_controls' slider) moves the isocontour that
  // bound follows (`second`: fe.mesh.isovalue2's, in interval mode only); a value past an end of
  // the axis pins the bar there (bar_holds). No remesh: like any fe_controls edit, it waits for
  // Tet-mesh.
  void follow_bound(const char *rel, std::string &seen_str, bool second) {
    cvc::app &a = *app;
    const std::string s = ari::read_or<std::string>(a, key(rel), "");
    if (s == seen_str)
      return;
    seen_str = s;
    if (!model_ok)
      return;
    const int first = fe_isocontour(model);
    const int i = second ? fe_isocontour2(model) : first;
    double lo = 0.0, hi = 1.0;
    cvc::tf::effective_domain(model, has_range, range_lo, range_hi, lo, hi);
    std::vector<double> v;
    if (i >= 0 && parse_n(s, 1, v) && !bar_holds(i, v[0], lo, hi)) {
      cvc::tf::model m = model;
      m.isocontours[std::size_t(i)].at = std::clamp(cvc::tf::to_normalized(v[0], lo, hi), 0.0, 1.0);
      m.normalize(); // may reorder the isocontours; the followed ones keep their ids
      const std::string ms = cvc::tf::to_string(m);
      ari::write<std::string>(a, model_key, ms);
      // Already what fe_lab holds: record it as lowered so it is not written back.
      seen = applied = ari::read_or<std::string>(a, model_key, ms);
      model = std::move(m);
    }
  }

  // fe_lab: the range moved (a load brought another volume): the bars the bounds follow move to
  // where the keys' values sit on it. The keys hold the bounds, so they are never rewritten for a
  // range and nothing re-meshes; the model, which now agrees with them, is recorded as lowered.
  void place_fe_bars() {
    double lo = 0.0, hi = 1.0;
    cvc::tf::effective_domain(model, has_range, range_lo, range_hi, lo, hi);
    cvc::tf::model m = model;
    bool moved = false;
    const auto place = [&](int i, const char *rel) {
      const double v =
          ari::read_or<double>(*app, key(rel), std::numeric_limits<double>::quiet_NaN());
      if (i < 0 || !std::isfinite(v))
        return;
      const double at = std::clamp(cvc::tf::to_normalized(v, lo, hi), 0.0, 1.0);
      if (m.isocontours[std::size_t(i)].at != at) {
        m.isocontours[std::size_t(i)].at = at;
        moved = true;
      }
    };
    const int first = fe_isocontour(m);
    const int second = fe_isocontour2(m);
    place(first, "fe.mesh.isovalue");
    place(second, "fe.mesh.isovalue2");
    if (!moved)
      return;
    m.normalize(); // may reorder the isocontours; the followed ones keep their ids
    const std::string ms = cvc::tf::to_string(m);
    ari::write<std::string>(*app, model_key, ms);
    seen = applied = ari::read_or<std::string>(*app, model_key, ms);
    model = std::move(m);
  }

  // fe_lab: whether fe.mesh.region switched between solid and interval (fe_interval follows it).
  // To interval the second bound is followed from here on, to solid no longer; follow_fe brings
  // the keys and the model together after the bounds.
  bool follow_region() {
    const std::string r = ari::read_or<std::string>(*app, key("fe.mesh.region"), "");
    if (r == region_seen)
      return false;
    region_seen = r;
    const bool interval = r == "interval";
    if (interval == fe_interval)
      return false;
    fe_interval = interval;
    return true;
  }

  // fe_lab: follow fe.mesh.region and the isovalues someone else wrote (fe_controls), publish
  // <tf>.interval, and run the debounced remesh. The remesh is a seconds-long job fe_lab cannot
  // interrupt, so it waits for the edits to settle and for the running job, and never replaces a
  // request already waiting.
  void follow_fe() {
    cvc::app &a = *app;
    // The region first (it decides whether isovalue2 is followed), then the bounds someone else
    // wrote move their bars, and only then does a switch bring the keys to the model: switching to
    // interval, the second bound adopts an isocontour (or one is added at fe.mesh.isovalue2) and a
    // key nobody wrote takes its bar's value. So a bound written in the same tick as the region (a
    // script, an `on:` action, a peer setting both) keeps its value. A switch never re-meshes:
    // like fe_controls' other keys, it waits for Tet-mesh.
    const bool switched = follow_region();
    follow_bound("fe.mesh.isovalue", iso_seen, false);
    follow_bound("fe.mesh.isovalue2", iso2_seen, true);
    if (switched && model_ok)
      lower_fe(false);
    publish_interval();
    if (!opt.auto_remesh) {
      remesh_pending = false;
      return;
    }
    if (!remesh_pending)
      return;
    // Owed no longer once a mesh job -- this binding's own request, fe_controls' Tet-mesh, a
    // script's -- has started with the region and bounds the keys hold now (fe_lab publishes each
    // job's at fe.last_mesh): running it again would mesh the same, and clear the field shown.
    if (ari::read_or<std::string>(a, key("fe.last_mesh"), "") == fe_mesh_signature(a, key("fe"))) {
      remesh_pending = false;
      return;
    }
    const double waited =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - iso_changed).count();
    if (waited < opt.remesh_debounce)
      return;
    if (ari::read_or<int>(a, key("fe.busy"), 0) != 0 ||
        !ari::read_or<std::string>(a, key("fe.request"), "").empty())
      return;
    ari::write<std::string>(a, key("fe.request"), "mesh");
    remesh_pending = false;
  }
};

tf_binding::tf_binding(std::shared_ptr<GraphicsNode> node, tf_target target, const cvc::volume &vol,
                       tf_binding_options opt, std::vector<std::string> *warnings)
    : _impl(std::make_unique<impl>(vol)), _target(target) {
  impl &d = *_impl;
  d.target = target;
  d.opt = std::move(opt);
  if (!node)
    return;
  d.node = node;
  d.app = &node->app();
  d.base = node->stateName();
  d.name = node->getName();
  _model_path = d.model_key = node->stateName("transfer_function");
  d.owner = task_owner(d.model_key);
  {
    // The newest binding for a key owns its overlay_status (a predecessor being replaced by a hot
    // reload leaves it alone).
    std::lock_guard<std::mutex> lock(registry_mutex());
    status_owners()[{d.app, d.model_key}] = d.owner;
  }
  d.opt.seed.normalize();
  d.seed_str = cvc::tf::to_string(d.opt.seed);
  if (target == tf_target::volume) {
    if (auto *vn = dynamic_cast<VolumeNode *>(node.get()))
      d.volume_seen = vn->getSharedVolume();
  } else if (d.opt.volume_source) {
    // fe_lab: the volume as its loads bring it -- none yet while an SDF is being computed.
    const std::shared_ptr<const cvc::volume> v = d.opt.volume_source();
    d.has_volume = v != nullptr;
    if (v) {
      d.volume_seen = v;
      d.vol = *v;
    }
  }
  d.read_volume_range();
  cvc::app &a = *d.app;

  // volslice: the realizer wrote an explicit seed domain as the node's window -- it is ours.
  if (target == tf_target::volslice && !d.opt.seed.domain.automatic) {
    std::vector<double> w;
    if (parse_n(ari::read_or<std::string>(a, d.key("volslice.window"), ""), 2, w) &&
        same_range(w[0], w[1], d.opt.seed.domain.min, d.opt.seed.domain.max)) {
      d.owns_window = true;
      d.window_lo = w[0];
      d.window_hi = w[1];
    }
  }
  if (target == tf_target::fe_lab) {
    d.iso_seen = ari::read_or<std::string>(a, d.key("fe.mesh.isovalue"), "");
    d.iso2_seen = ari::read_or<std::string>(a, d.key("fe.mesh.isovalue2"), "");
    d.region_seen = ari::read_or<std::string>(a, d.key("fe.mesh.region"), "");
    d.fe_interval = d.region_seen == "interval";
    d.fe_iso_id = d.opt.fe_isocontour;
    d.fe_iso2_id = d.opt.fe_isocontour2;
  }
  d.refresh_range(*node);
  std::string before = ari::read_or<std::string>(a, d.model_key, "");
  bool reseed = before.empty();
  {
    std::lock_guard<std::mutex> lock(registry_mutex());
    std::string &prev = seed_memory()[{d.app, d.model_key}];
    reseed = reseed || (!prev.empty() && before == prev);
    prev = d.seed_str;
  }
  if (reseed) { // written outside the lock: the write runs the node's state handlers
    ari::write<std::string>(a, d.model_key, d.seed_str);
    before = ari::read_or<std::string>(a, d.model_key, d.seed_str);
  }
  if (d.opt.seed_is_applied && before == d.seed_str) {
    // The renderer already draws the seed: record it as lowered.
    d.seen = d.applied = before;
    d.model = d.opt.seed;
    d.model_ok = true;
  } else {
    std::string err;
    if (!d.read_model(&err) && warnings)
      warnings->push_back("ari: scene node '" + d.name + "': the transfer-function model at '" +
                          d.model_key + "' does not parse (" + err +
                          "); the renderer keeps the document's transfer function");
  }
  if (target == tf_target::fe_lab && d.model_ok) {
    // fe.mesh.isovalue (and, for an interval, isovalue2) follow these isocontours, by id, from
    // here on.
    d.fe_isocontour(d.model);
    d.fe_isocontour2(d.model);
    // The keys follow the model (an interval needs both bounds in it, even under an applied
    // seed), but attaching never re-meshes: auto_mesh or Tet-mesh meshes what the keys hold. A
    // bar fe_lab's seed pinned at an end leaves a bound past that end as it is (bar_holds).
    if (d.dirty || d.fe_interval) {
      d.lower_fe(false);
      d.applied = d.seen;
      d.dirty = false;
    }
  } else if (d.model_ok && d.dirty) {
    d.lower();
  }
  if (target == tf_target::fe_lab)
    d.publish_interval();

  // Layers already beside the key are a predecessor's (a hot reload, perhaps of another file): not
  // known to describe this volume, so none is shown until this binding computes its own.
  d.clear_layers();
  d.submit(cvc::tf::overlay_histogram);
}

tf_binding::~tf_binding() {
  // Stop the live tasks: no join and no thread -- each cancel kills and reaps the task's process
  // and lets go of its job (a pool chunk still running it stops at its next check), so nothing is
  // published for this binding afterwards; a progress note of theirs still in overlay_status goes
  // too unless a newer binding owns the key (retire). Never throws.
  try {
    if (_impl)
      _impl->retire();
  } catch (...) {
  }
}

void tf_binding::tick() {
  impl &d = *_impl;
  std::shared_ptr<GraphicsNode> node = d.node.lock();
  if (!node || !d.app) {
    d.retire(); // the node is gone: stop computing for it, for good
    return;
  }
  const bool range_moved = d.refresh_range(*node);
  d.read_model();
  const bool fe_lab = d.target == tf_target::fe_lab;
  const bool rescaled = range_moved && d.model_ok && d.model.domain.automatic;
  if (fe_lab && rescaled)
    d.place_fe_bars(); // fe_lab's bounds are its keys' values, wherever the range puts them
  if (d.model_ok && (d.dirty || (rescaled && !fe_lab)))
    d.lower();
  if (fe_lab)
    d.follow_fe();
  d.service_overlays();
}

bool tf_binding::wait_overlays(double timeout_s) {
  impl &d = *_impl;
  if (!d.app)
    return true;
  if (!d.node.lock()) {
    d.retire(); // as tick() would: nothing is computed for a node that is gone
    return true;
  }
  using clock = std::chrono::steady_clock;
  const clock::time_point start = clock::now();
  std::vector<int> pids;
  for (const overlay_task &t : d.tasks)
    pids.push_back(t.pid);
  bool finished = true;
  for (const int pid : pids) {
    const double left = timeout_s - std::chrono::duration<double>(clock::now() - start).count();
    if (!cvc::tf::wait_overlays(*d.app, pid, std::max(0.0, left)))
      finished = false;
  }
  // Publish what arrived now, as the next tick() would (or retire, as it would once the node is
  // gone).
  if (d.node.lock()) {
    d.take_results();
    d.write_idle_status();
  } else {
    d.retire();
  }
  return finished;
}

std::shared_ptr<tf_binding> attach_transfer_function(RealizedScene &out,
                                                     std::shared_ptr<GraphicsNode> node,
                                                     tf_target target, const cvc::volume &vol,
                                                     tf_binding_options opt,
                                                     std::vector<std::string> *warnings) {
  if (!node)
    return nullptr;
  std::shared_ptr<tf_binding> b;
  try {
    b = std::make_shared<tf_binding>(node, target, vol, std::move(opt), warnings);
  } catch (const std::exception &e) {
    if (warnings)
      warnings->push_back("ari: scene node '" + node->getName() +
                          "': transfer-function binding failed: " + e.what());
    return nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(registry_mutex());
    prune_registry();
    registry()[{&node->app(), b->model_path()}] = b;
  }
  out.custom_ticks.push_back([b](vtkRenderer *) { b->tick(); });
  return b;
}

std::shared_ptr<tf_binding> find_transfer_function(cvc::app &app, const std::string &model_path) {
  std::lock_guard<std::mutex> lock(registry_mutex());
  prune_registry();
  auto it = registry().find({&app, model_path});
  return it == registry().end() ? nullptr : it->second.lock();
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

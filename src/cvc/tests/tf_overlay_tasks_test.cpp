/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::tf::schedule_overlays (cvc/tf/overlay_tasks.h): an overlay_job driven by a
// state_exec process on the app's scheduler, pumped here frame by frame exactly as an Ariadne
// Runtime pumps it (drain_ingress, wake_awaiting, sync_run(200000, 0.1)), in both modes.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/tf/overlays.h>
#include <cvc/volume/volume.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace cvc::tf;
namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

namespace {

double ms_since(steady::time_point t0) {
  return std::chrono::duration<double, std::milli>(steady::now() - t0).count();
}

cvc::volume make_volume(cvc::app &ctx, cvc::uint64 nx, cvc::uint64 ny, cvc::uint64 nz,
                        const std::function<double(double, double, double)> &f) {
  cvc::volume vol(ctx, cvc::dimension(nx, ny, nz), cvc::Float);
  for (cvc::uint64 k = 0; k < nz; ++k)
    for (cvc::uint64 j = 0; j < ny; ++j)
      for (cvc::uint64 i = 0; i < nx; ++i)
        vol(i, j, k, f(double(i), double(j), double(k)));
  vol.unsetMinMax();
  return vol;
}

double hash01(double x, double y, double z) {
  const double s = std::sin(x * 12.9898 + y * 78.233 + z * 37.719) * 43758.5453;
  return s - std::floor(s);
}

// Two blobs (scaled to a 64-voxel box) under a little noise.
double blobs(double x, double y, double z) {
  auto g = [](double dx, double dy, double dz) {
    return std::exp(-(dx * dx + dy * dy + dz * dz) / 120.0);
  };
  return g(x - 20, y - 32, z - 32) + 0.9 * g(x - 44, y - 30, z - 34) + 0.02 * hash01(x, y, z);
}

double ripples(double x, double y, double z) {
  return std::sin(x * 0.3) * std::cos(y * 0.25) + 0.1 * z + 0.02 * hash01(z, y, x);
}

// The state node at `path`, or nullptr (never creates one).
cvc::state *find(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app).findDescendant(path);
}

std::string value_at(cvc::app &app, const std::string &path) {
  cvc::state *s = find(app, path);
  return s ? s->value() : std::string();
}

// The published layers are byte-for-byte what publish_overlays(compute_overlays(...)) writes.
void expect_published(cvc::app &app, const std::string &tf, const overlays &ref) {
  ASSERT_TRUE(ref.has_range);
  EXPECT_EQ(value_at(app, tf + ".range"), range_to_string(ref.range_lo, ref.range_hi));
  if (!ref.histogram.empty())
    EXPECT_EQ(value_at(app, tf + ".histogram"), histogram_to_string(ref.histogram));
  if (!ref.spectrum.empty())
    EXPECT_EQ(value_at(app, tf + ".contour_spectrum"), spectrum_to_string(ref.spectrum));
  if (!ref.tree.empty())
    EXPECT_EQ(value_at(app, tf + ".contour_tree"), tree_to_string(ref.tree));
  EXPECT_EQ(value_at(app, tf + ".overlay_status"), ref.status);
}

void expect_nothing_published(cvc::app &app, const std::string &tf) {
  for (const char *key : {".range", ".histogram", ".contour_spectrum", ".contour_tree"})
    EXPECT_EQ(find(app, tf + key), nullptr) << tf << key << " was published";
}

void expect_same(const overlays &a, const overlays &b) {
  EXPECT_EQ(a.has_range, b.has_range);
  EXPECT_EQ(a.range_lo, b.range_lo);
  EXPECT_EQ(a.range_hi, b.range_hi);
  EXPECT_EQ(a.histogram, b.histogram);
  EXPECT_EQ(spectrum_to_string(a.spectrum), spectrum_to_string(b.spectrum));
  EXPECT_EQ(tree_to_string(a.tree), tree_to_string(b.tree));
  EXPECT_EQ(a.status, b.status);
}

class TfOverlayTasks : public ::testing::Test {
protected:
  cvc::app app;

  se::async_scheduler &sched() { return app.exec_scheduler(); }

  // One frame of an Ariadne Runtime's pump (Runtime::Impl::pump_and_sweep_actions); its time in
  // ms.
  double pump() {
    const steady::time_point t0 = steady::now();
    sched().drain_ingress();
    sched().wake_awaiting();
    sched().sync_run(200000, 0.1);
    return ms_since(t0);
  }

  std::string name_of(int pid) {
    const auto info = sched().get_process_info(pid);
    return info ? info->name : std::string();
  }

  // Wait (pumping, a frame per ~2 ms) until the job count drops to `n` or `timeout_ms` passes.
  bool jobs_drop_to(std::size_t n, double timeout_ms) {
    const steady::time_point t0 = steady::now();
    while (overlay_job::instances() > n) {
      if (ms_since(t0) > timeout_ms)
        return false;
      pump();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
  }
};

struct frame_log {
  int frames = 0;
  double max_ms = 0.0, total_ms = 0.0;
  std::vector<std::string> notes; // overlay_status, each time it changed while running
};

} // namespace

TEST(TfOverlayTasksBuild, ThreadsAreAvailableNatively) { EXPECT_TRUE(threads_available()); }

TEST_F(TfOverlayTasks, CooperativeTaskSlicesAcrossFramesAndMatchesCompute) {
  // 96^3: the progress notes come at most ~10 Hz, so the task must run well past 300 ms for notes
  // from both slow layers. A 64^3 one took ~200 ms on a CI runner and showed only two.
  const cvc::volume vol = make_volume(app, 96, 96, 96, blobs);
  overlay_task_options opt;
  opt.threading = overlay_threading::cooperative;
  opt.slice_ms = 4.0;
  const overlays ref = compute_overlays(vol, overlay_all, opt.overlays);
  ASSERT_EQ(ref.status, "");
  const std::string tf = "tasks_test.coop.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  EXPECT_NE(name_of(pid).find("cooperative"), std::string::npos) << name_of(pid);
  EXPECT_TRUE(overlays_running(app, pid));
  EXPECT_EQ(overlay_job::instances(), 1u);

  frame_log log;
  std::vector<std::pair<double, std::string>> frames; // each pump's ms, and the note before it
  const steady::time_point start = steady::now();
  while (overlays_running(app, pid)) {
    const std::string before = value_at(app, tf + ".overlay_status");
    const double ms = pump();
    frames.emplace_back(ms, before);
    ++log.frames;
    log.max_ms = std::max(log.max_ms, ms);
    log.total_ms += ms;
    const std::string note = value_at(app, tf + ".overlay_status");
    if (overlays_running(app, pid) && !note.empty() &&
        (log.notes.empty() || log.notes.back() != note))
      log.notes.push_back(note);
    ASSERT_LT(ms_since(start), 120000.0) << "the task never finished";
  }
  const double wall = ms_since(start);
  std::printf("[cooperative] 96^3, slice %.1f ms: %d frames, max pump %.2f ms, mean pump %.2f ms, "
              "%.0f ms wall, %zu progress notes (last \"%s\")\n",
              opt.slice_ms, log.frames, log.max_ms, log.total_ms / log.frames, wall,
              log.notes.size(), log.notes.empty() ? "" : log.notes.back().c_str());
  std::vector<std::size_t> slowest(frames.size());
  for (std::size_t i = 0; i < slowest.size(); ++i)
    slowest[i] = i;
  std::sort(slowest.begin(), slowest.end(),
            [&](std::size_t x, std::size_t y) { return frames[x].first > frames[y].first; });
  for (std::size_t k = 0; k < std::min<std::size_t>(4, slowest.size()); ++k)
    std::printf("  slow frame %zu/%zu: %.2f ms (after \"%s\")\n", slowest[k], frames.size(),
                frames[slowest[k]].first, frames[slowest[k]].second.c_str());

  expect_published(app, tf, ref);
  EXPECT_GT(log.frames, 20) << "a 96^3 task should take many 4 ms frames";
  EXPECT_LT(log.max_ms, 20.0 * opt.slice_ms) << "a frame ran far past its slice";
  // Progress was visible while it ran, from more than one layer, at most ~10 Hz.
  ASSERT_GE(log.notes.size(), 3u);
  bool saw_spectrum = false, saw_tree = false;
  for (const std::string &n : log.notes) {
    EXPECT_NE(n.find('%'), std::string::npos) << n;
    saw_spectrum = saw_spectrum || n.find("contour spectrum") != std::string::npos;
    saw_tree = saw_tree || n.find("contour tree") != std::string::npos;
  }
  EXPECT_TRUE(saw_spectrum);
  EXPECT_TRUE(saw_tree);
  EXPECT_LE(static_cast<double>(log.notes.size()), wall / 100.0 + 2.0);
  // Finished: reaped, and the job (volume copy, builders) let go.
  EXPECT_FALSE(sched().get_process_info(pid).has_value());
  EXPECT_EQ(overlay_job::instances(), 0u);
}

TEST_F(TfOverlayTasks, ThreadedTaskRunsOnThePoolWithoutBlockingThePump) {
  const cvc::volume vol = make_volume(app, 64, 64, 64, blobs);
  overlay_task_options opt;
  opt.threading = overlay_threading::threaded;
  opt.chunk_ms = 100.0;
  const overlays ref = compute_overlays(vol, overlay_all, opt.overlays);
  const std::string tf = "tasks_test.threaded.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  EXPECT_NE(name_of(pid).find("threaded"), std::string::npos) << name_of(pid);

  frame_log log;
  const steady::time_point start = steady::now();
  while (overlays_running(app, pid)) {
    const double ms = pump();
    ++log.frames;
    log.max_ms = std::max(log.max_ms, ms);
    log.total_ms += ms;
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); // the rest of a frame
    ASSERT_LT(ms_since(start), 120000.0) << "the task never finished";
  }
  const double wall = ms_since(start);
  std::printf("[threaded] 64^3, chunk %.0f ms: %d frames, max pump %.2f ms, total in pumps %.1f "
              "ms of %.0f ms wall\n",
              opt.chunk_ms, log.frames, log.max_ms, log.total_ms, wall);
  expect_published(app, tf, ref);
  // The scheduler thread never waited for a chunk: every pump stayed far under one, and the work
  // happened between pumps.
  EXPECT_LT(log.max_ms, opt.chunk_ms / 2.0);
  EXPECT_LT(log.total_ms, 0.5 * wall);
  EXPECT_FALSE(sched().get_process_info(pid).has_value());
  EXPECT_TRUE(jobs_drop_to(0, 5000.0));
}

TEST_F(TfOverlayTasks, AutomaticPicksThreadedNativelyAndWaitOverlaysFinishesIt) {
  const cvc::volume vol = make_volume(app, 40, 36, 32, ripples);
  overlay_task_options opt; // automatic
  const overlays ref = compute_overlays(vol, overlay_all, opt.overlays);
  const std::string tf = "tasks_test.automatic.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  EXPECT_NE(name_of(pid).find("threaded"), std::string::npos) << name_of(pid);
  EXPECT_FALSE(wait_overlays(app, pid, 0.0)) << "a zero timeout returns before it finishes";
  EXPECT_TRUE(overlays_running(app, pid));
  EXPECT_TRUE(wait_overlays(app, pid, 60.0));
  EXPECT_FALSE(overlays_running(app, pid));
  EXPECT_TRUE(wait_overlays(app, pid, 1.0)) << "a finished task waits for nothing";
  expect_published(app, tf, ref);
  EXPECT_TRUE(jobs_drop_to(0, 5000.0));
}

TEST_F(TfOverlayTasks, CancelledCooperativeTaskPublishesNothingAndLetsGo) {
  const cvc::volume vol = make_volume(app, 48, 48, 48, blobs);
  overlay_task_options opt;
  opt.threading = overlay_threading::cooperative;
  const std::string tf = "tasks_test.cancel_coop.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  for (int i = 0; i < 5; ++i)
    pump();
  ASSERT_TRUE(overlays_running(app, pid));
  const std::string note = value_at(app, tf + ".overlay_status");
  EXPECT_NE(note.find('%'), std::string::npos) << note;

  cancel_overlays(app, pid);
  EXPECT_FALSE(sched().get_process_info(pid).has_value()) << "not reaped";
  EXPECT_FALSE(overlays_running(app, pid));
  EXPECT_EQ(overlay_job::instances(), 0u) << "the job outlived its cancel";
  for (int i = 0; i < 50; ++i)
    pump();
  expect_nothing_published(app, tf);
  EXPECT_EQ(value_at(app, tf + ".overlay_status"), note) << "a cancelled task wrote again";
  cancel_overlays(app, pid); // a finished or unknown pid is fine
  cancel_overlays(app, 987654);
  EXPECT_FALSE(overlays_running(app, 987654));
}

TEST_F(TfOverlayTasks, CancelledThreadedTaskStopsItsChunkAndPublishesNothing) {
  const cvc::volume vol = make_volume(app, 64, 64, 64, blobs);
  overlay_task_options opt;
  opt.threading = overlay_threading::threaded;
  opt.chunk_ms = 2000.0; // a chunk far longer than the test waits: the cancel must stop it
  const std::string tf = "tasks_test.cancel_threaded.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  pump(); // launches the first chunk
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(overlays_running(app, pid));

  const steady::time_point t0 = steady::now();
  cancel_overlays(app, pid);
  EXPECT_FALSE(sched().get_process_info(pid).has_value()) << "not reaped";
  EXPECT_FALSE(overlays_running(app, pid));
  ASSERT_TRUE(jobs_drop_to(0, 5000.0)) << "the pool chunk kept the job";
  const double stop_ms = ms_since(t0);
  std::printf("[threaded cancel] the running chunk let go of the job %.1f ms after the cancel\n",
              stop_ms);
  EXPECT_LT(stop_ms, 500.0) << "the chunk ran on past the cancel";
  for (int i = 0; i < 20; ++i) {
    pump();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  expect_nothing_published(app, tf);
  // The abandoned chunk's reply is not left queued in the scheduler.
  EXPECT_FALSE(overlays_running(app, pid)); // sweeps
  EXPECT_EQ(sched().total_pending_messages(), 0u);
}

TEST_F(TfOverlayTasks, AChunkStoppedWhileItRunsLeavesNoReplyBehind) {
  const cvc::volume vol = make_volume(app, 64, 64, 64, blobs);
  overlay_task_options opt;
  opt.threading = overlay_threading::threaded;
  opt.chunk_ms = 2000.0; // still running when it is cancelled
  const std::string tf = "tasks_test.stopped_chunk.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  pump(); // launches the first chunk
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(overlays_running(app, pid));

  cancel_overlays(app, pid);
  EXPECT_EQ(overlay_orphaned_replies(app), 0u) << "the stopped chunk is still owed a collection";
  ASSERT_TRUE(jobs_drop_to(0, 5000.0)) << "the pool chunk kept the job";
  // The chunk has stopped; give it time to post a reply, and pump it in, but sweep nothing (no
  // schedule_overlays, cancel_overlays or overlays_running after the cancel's own sweep).
  const steady::time_point t0 = steady::now();
  while (ms_since(t0) < 300.0 && sched().total_pending_messages() == 0) {
    pump();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(sched().total_pending_messages(), 0u) << "the stopped chunk left a reply in the queue";
}

TEST_F(TfOverlayTasks, AChunkReplyAlreadyTakenLeavesNothingToCollect) {
  // The chunk's reply was delivered (the process is ready, its await answered) but the process has
  // not stepped again when the task is stopped, by cancel_overlays or by kill_owner.
  const cvc::volume vol = make_volume(app, 48, 48, 48, blobs);
  for (const bool by_owner : {false, true}) {
    const std::string what = by_owner ? "kill_owner" : "cancel_overlays";
    overlay_task_options opt;
    opt.threading = overlay_threading::threaded;
    opt.chunk_ms = 1.0;
    opt.owner = "tf_overlay_tasks_test#taken";
    const std::string tf =
        "tasks_test.taken_" + std::to_string(int(by_owner)) + ".transfer_function";
    const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
    ASSERT_GT(pid, 0) << what;
    pump(); // the first step launches a chunk and parks on its future
    const steady::time_point t0 = steady::now();
    for (;;) {
      sched().drain_ingress(); // delivers the reply, if posted; steps nothing
      const std::optional<se::process_info> info = sched().get_process_info(pid);
      ASSERT_TRUE(info.has_value()) << what;
      ASSERT_FALSE(se::is_finished(info->status)) << what;
      if (info->status == se::process_status::ready)
        break;
      ASSERT_LT(ms_since(t0), 10000.0) << what << ": the chunk's reply never arrived";
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (by_owner)
      EXPECT_EQ(sched().kill_owner(opt.owner), 1) << what;
    else
      cancel_overlays(app, pid);
    EXPECT_FALSE(overlays_running(app, pid)) << what; // sweeps
    EXPECT_EQ(overlay_orphaned_replies(app), 0u) << what << ": a taken reply was left to collect";
    EXPECT_EQ(sched().total_pending_messages(), 0u) << what;
    EXPECT_FALSE(sched().get_process_info(pid).has_value()) << what << ": not reaped";
    EXPECT_TRUE(jobs_drop_to(0, 5000.0)) << what;
    expect_nothing_published(app, tf);
  }
}

TEST_F(TfOverlayTasks, DeliverHookReplacesPublishing) {
  const cvc::volume vol = make_volume(app, 32, 32, 32, blobs);
  for (const overlay_threading mode :
       {overlay_threading::cooperative, overlay_threading::threaded}) {
    overlay_task_options opt;
    opt.threading = mode;
    int calls = 0;
    std::thread::id where;
    overlays got;
    opt.deliver = [&](overlays &&ov) {
      ++calls;
      where = std::this_thread::get_id();
      got = std::move(ov);
    };
    const std::string tf = std::string("tasks_test.deliver_") +
                           (mode == overlay_threading::threaded ? "threaded" : "coop") +
                           ".transfer_function";
    const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
    ASSERT_GT(pid, 0);
    ASSERT_TRUE(wait_overlays(app, pid, 60.0));
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(where, std::this_thread::get_id()) << "deliver ran off the scheduler thread";
    expect_same(got, compute_overlays(vol, overlay_all, opt.overlays));
    expect_nothing_published(app, tf);
  }
  EXPECT_TRUE(jobs_drop_to(0, 5000.0));
}

TEST_F(TfOverlayTasks, TwoTasksOnDifferentKeysRunSideBySide) {
  // Both big enough to take many 4 ms frames (the spectrum is the slow layer), even on a fast
  // runner: 48x40x36 finished in 10 frames on one.
  const cvc::volume a = make_volume(app, 64, 64, 64, blobs);
  const cvc::volume b = make_volume(app, 64, 56, 48, ripples);
  overlay_task_options opt;
  opt.threading = overlay_threading::cooperative;
  const std::string ta = "tasks_test.pair.a", tb = "tasks_test.pair.b";
  const unsigned bits_b = overlay_contour_spectrum | overlay_histogram;
  const int pa = schedule_overlays(app, ta, a, overlay_all, opt);
  const int pb = schedule_overlays(app, tb, b, bits_b, opt);
  ASSERT_GT(pa, 0);
  ASSERT_GT(pb, 0);
  EXPECT_NE(pa, pb);
  EXPECT_EQ(overlay_job::instances(), 2u);
  int frames = 0, both_running = 0;
  while (overlays_running(app, pa) || overlays_running(app, pb)) {
    pump();
    ++frames;
    if (overlays_running(app, pa) && overlays_running(app, pb))
      ++both_running;
    ASSERT_LT(frames, 200000);
  }
  std::printf("[two tasks] %d frames, both running in %d\n", frames, both_running);
  EXPECT_GT(both_running, 10) << "the tasks did not interleave";
  expect_published(app, ta, compute_overlays(a, overlay_all, opt.overlays));
  expect_published(app, tb, compute_overlays(b, bits_b, opt.overlays));
  EXPECT_EQ(find(app, tb + ".contour_tree"), nullptr) << "an unrequested layer was published";
  EXPECT_EQ(overlay_job::instances(), 0u);
}

TEST_F(TfOverlayTasks, KillOwnerStopsAnOwnedTaskAndItIsReaped) {
  const cvc::volume vol = make_volume(app, 48, 48, 48, blobs);
  overlay_task_options opt;
  opt.threading = overlay_threading::cooperative;
  opt.owner = "tf_overlay_tasks_test#doc";
  const std::string tf = "tasks_test.owned.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  for (int i = 0; i < 3; ++i)
    pump();
  ASSERT_TRUE(overlays_running(app, pid));
  EXPECT_EQ(sched().kill_owner(opt.owner), 1);
  EXPECT_EQ(overlay_job::instances(), 0u) << "the killed process kept its job";
  EXPECT_FALSE(overlays_running(app, pid));
  EXPECT_FALSE(sched().get_process_info(pid).has_value()) << "not reaped";
  for (int i = 0; i < 20; ++i)
    pump();
  expect_nothing_published(app, tf);

  // Threaded: the running chunk stops too.
  opt.threading = overlay_threading::threaded;
  opt.chunk_ms = 2000.0;
  const int tpid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(tpid, 0);
  pump();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(sched().kill_owner(opt.owner), 1);
  EXPECT_FALSE(overlays_running(app, tpid));
  EXPECT_TRUE(jobs_drop_to(0, 5000.0));
  expect_nothing_published(app, tf);
}

TEST_F(TfOverlayTasks, DegenerateVolumesEndWithComputeOverlaysStatus) {
  // A NaN voxel: the histogram skips it, the tree refuses it.
  cvc::volume holes =
      make_volume(app, 12, 12, 12, [](double x, double y, double z) { return x + 2 * y + 4 * z; });
  holes(1, 2, 3, std::numeric_limits<double>::quiet_NaN());
  holes.unsetMinMax();
  // A constant volume: one histogram bin, no tree.
  const cvc::volume flat = make_volume(app, 10, 10, 10, [](double, double, double) { return 3.0; });
  // A NaN at voxel 0: calcMinMax's range is NaN, so nothing but the status.
  cvc::volume nan0 =
      make_volume(app, 8, 8, 8, [](double x, double y, double z) { return x - y + z; });
  nan0(0, 0, 0, std::numeric_limits<double>::quiet_NaN());
  nan0.unsetMinMax();

  int i = 0;
  const std::vector<const cvc::volume *> volumes = {&holes, &flat, &nan0};
  for (const cvc::volume *vol : volumes)
    for (const overlay_threading mode :
         {overlay_threading::cooperative, overlay_threading::threaded}) {
      const overlays ref = compute_overlays(*vol, overlay_all);
      ASSERT_FALSE(ref.status.empty());
      overlay_task_options opt;
      opt.threading = mode;
      const std::string tf = "tasks_test.degenerate" + std::to_string(i++) + ".transfer_function";
      const int pid = schedule_overlays(app, tf, *vol, overlay_all, opt);
      ASSERT_GT(pid, 0);
      ASSERT_TRUE(wait_overlays(app, pid, 60.0));
      EXPECT_EQ(value_at(app, tf + ".overlay_status"), ref.status) << tf;
      expect_same(read_overlays(cvc::state::instance(app)(tf)), ref);
    }
  EXPECT_TRUE(jobs_drop_to(0, 5000.0));
}

TEST_F(TfOverlayTasks, ATaskThatCannotStartReturnsMinusOne) {
  const cvc::volume vol = make_volume(app, 4, 4, 4, [](double x, double, double) { return x; });
  EXPECT_EQ(schedule_overlays(app, "", vol, overlay_all), -1);
  EXPECT_EQ(overlay_job::instances(), 0u);
  // An empty volume starts and ends with compute_overlays's note.
  const cvc::volume none(app, cvc::dimension(0, 0, 0), cvc::Float);
  const int pid = schedule_overlays(app, "tasks_test.empty.transfer_function", none, overlay_all);
  ASSERT_GT(pid, 0);
  ASSERT_TRUE(wait_overlays(app, pid, 10.0));
  EXPECT_EQ(value_at(app, "tasks_test.empty.transfer_function.overlay_status"),
            compute_overlays(none).status);
}

namespace {

// An app whose background pool can create no thread, as when thread resources are exhausted (a
// throw from the pool's test seam counts as the creation failing).
struct no_thread_app : cvc::app {
  std::atomic<int> attempts{0};
  no_thread_app() {
    _poolWorkerSpawnHook = [this] {
      ++attempts;
      throw boost::thread_resource_error();
    };
  }
};

} // namespace

// A threaded task carries on cooperatively when no worker can be started for its chunk (the app's
// background pool has none and can't create one, so compute_async throws), and publishes what
// compute_overlays gives. (The chunk used to wait in the pool's queue for a worker that never came,
// and the task with it.)
TEST(TfOverlayTasksNoThreads, ThreadedTaskCarriesOnCooperativelyWhenNoWorkerCanStart) {
  no_thread_app app;
  const cvc::volume vol = make_volume(app, 32, 32, 32, ripples);
  overlay_task_options opt;
  opt.threading = overlay_threading::threaded;
  const overlays ref = compute_overlays(vol, overlay_all, opt.overlays);
  const std::string tf = "tasks_test.no_threads.transfer_function";
  const int pid = schedule_overlays(app, tf, vol, overlay_all, opt);
  ASSERT_GT(pid, 0);
  EXPECT_TRUE(wait_overlays(app, pid, 60.0)) << "the task waited for a chunk no worker will run";
  expect_published(app, tf, ref);
  EXPECT_GE(app.attempts.load(), 1) << "no chunk was launched";
  EXPECT_EQ(app.getPendingThreadCount(), 0u);
  EXPECT_EQ(app.getActiveThreadCount(), 0u);
}

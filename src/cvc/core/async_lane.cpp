// cvc::async_lane: a persistent named FIFO worker lane, plus the state_exec glue that posts lane
// results to channels a (chrooted) DSL process can msg-recv on. See inc/cvc/core/async_lane.h for
// the contract, the no-thread fallback and the per-frame pump.

#include <chrono>
#include <cvc/core/async_lane.h>
#include <cvc/core/state_exec/async_scheduler.h> // post_message + the pump calls
#include <cvc/core/state_exec/intrinsics.h> // resolve_channel_key, make_future, park_on_channel
#include <exception>
#include <memory>
#include <utility>

#if (defined(__linux__) || defined(__APPLE__)) && !defined(__EMSCRIPTEN__)
#include <pthread.h>
#define CVC_ASYNC_LANE_NAMES_THREADS 1
#endif

namespace cvc {

namespace se = state_exec;

namespace {

// The lane whose job is running on this thread (the worker thread for its whole life; the pump
// thread while it runs a deferred lane's jobs). stop() and run_deferred() use it to detect a call
// made from inside one of the lane's own jobs, which must neither join itself nor re-enter the
// drain.
thread_local const async_lane *t_running_lane = nullptr;

struct running_lane_scope {
  const async_lane *prev;
  explicit running_lane_scope(const async_lane *l) : prev(t_running_lane) { t_running_lane = l; }
  ~running_lane_scope() { t_running_lane = prev; }
  running_lane_scope(const running_lane_scope &) = delete;
  running_lane_scope &operator=(const running_lane_scope &) = delete;
};

void name_this_thread(const std::string &lane_name) {
#if defined(CVC_ASYNC_LANE_NAMES_THREADS)
  // Shown by top/perf/gdb. Linux caps a thread name at 15 bytes plus the NUL.
  std::string n = "lane:" + lane_name;
  if (n.size() > 15)
    n.resize(15);
#if defined(__APPLE__)
  pthread_setname_np(n.c_str());
#else
  pthread_setname_np(pthread_self(), n.c_str());
#endif
#else
  (void)lane_name;
#endif
}

} // namespace

// ---------------------------------------------------------------------------
// async_lane
// ---------------------------------------------------------------------------

async_lane::async_lane(std::string name, lane_mode mode)
    : name_(std::move(name)), mode_(async_lane_has_threads ? mode : lane_mode::deferred) {
  // Every member is initialized before the worker starts reading them.
  if (mode_ == lane_mode::threaded)
    worker_ = std::thread([this] { worker_loop(); });
}

async_lane::~async_lane() { stop(); }

bool async_lane::submit(job j) {
  if (!j)
    return false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (stopping_)
      return false;
    queue_.push_back(std::move(j));
    in_flight_.fetch_add(1, std::memory_order_acq_rel);
  }
  if (mode_ == lane_mode::threaded)
    cv_.notify_one();
  return true;
}

bool async_lane::pop_front(job &out) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (queue_.empty())
    return false;
  out = std::move(queue_.front());
  queue_.pop_front();
  return true;
}

void async_lane::worker_loop() {
  name_this_thread(name_);
  running_lane_scope scope(this);
  for (;;) {
    job j;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait(lk, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty())
        return; // stopping, and every job queued before stop() has run
      j = std::move(queue_.front());
      queue_.pop_front();
    }
    run_job(j);
  }
}

void async_lane::run_job(job &j) {
  const auto t0 = std::chrono::steady_clock::now();
  // A throw must never escape: on the worker it would call std::terminate, and on a deferred
  // lane it would unwind through the caller's pump.
  try {
    j();
  } catch (const std::exception &e) {
    failed_.fetch_add(1, std::memory_order_acq_rel);
    report_error(e.what());
  } catch (...) {
    failed_.fetch_add(1, std::memory_order_acq_rel);
    report_error("unknown error");
  }
  // Release the job's captures HERE, on the thread that ran it, before it counts as done, so an
  // owner that sees in_flight() == 0 knows the lane holds none of its objects any more.
  j = nullptr;
  last_job_ms_.store(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(),
      std::memory_order_release);
  completed_.fetch_add(1, std::memory_order_acq_rel);
  in_flight_.fetch_sub(1, std::memory_order_acq_rel);
}

void async_lane::report_error(const std::string &msg) {
  error_handler h;
  {
    std::lock_guard<std::mutex> lk(err_mtx_);
    last_error_ = msg;
    h = on_error_;
  }
  if (!h)
    return;
  try {
    h(msg);
  } catch (...) {
    // A throwing handler must not take the lane down with it; the error is already recorded.
  }
}

void async_lane::stop() {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stopping_ = true;
  }
  // From inside one of this lane's own jobs: a thread cannot join itself, and a deferred drain is
  // already running further up this stack. The lane is marked stopping; the join (or the rest of
  // the drain) happens in the destructor or in a stop() from another thread.
  if (t_running_lane == this)
    return;
  if (mode_ == lane_mode::threaded) {
    cv_.notify_all();
    std::lock_guard<std::mutex> jl(join_mtx_);
    if (worker_.joinable())
      worker_.join();
    return;
  }
  // Deferred: run everything still queued on the caller. Wait out a drain another thread (the
  // pump) is running, so jobs stay strictly one at a time.
  bool expected = false;
  while (!draining_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    expected = false;
    std::this_thread::yield();
  }
  {
    running_lane_scope scope(this);
    job j;
    while (pop_front(j))
      run_job(j);
  }
  draining_.store(false, std::memory_order_release);
}

std::size_t async_lane::run_deferred(std::size_t max_jobs) {
  if (mode_ != lane_mode::deferred || t_running_lane == this)
    return 0;
  bool expected = false;
  if (!draining_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    return 0; // another thread is draining this lane right now
  std::size_t budget;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    budget = queue_.size(); // only the jobs queued at entry: one call == one hop
  }
  if (max_jobs < budget)
    budget = max_jobs;
  std::size_t ran = 0;
  {
    running_lane_scope scope(this);
    job j;
    while (ran < budget && pop_front(j)) {
      run_job(j);
      ++ran;
    }
  }
  draining_.store(false, std::memory_order_release);
  return ran;
}

void async_lane::set_on_error(error_handler h) {
  std::lock_guard<std::mutex> lk(err_mtx_);
  on_error_ = std::move(h);
}

bool async_lane::stopped() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return stopping_;
}

std::size_t async_lane::pending() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return queue_.size();
}

std::string async_lane::last_error() const {
  std::lock_guard<std::mutex> lk(err_mtx_);
  return last_error_;
}

// ---------------------------------------------------------------------------
// state_exec integration
// ---------------------------------------------------------------------------

namespace {

// The value to deliver for a failed task: on_error(msg), or `fallback` when there is no on_error or
// it throws. Exactly one value is always posted, so a waiter is never left parked.
se::value_t lane_error_value(const lane_task_on_error &on_error, const std::string &msg,
                             const std::string &fallback) {
  if (on_error) {
    try {
      return on_error(msg);
    } catch (...) {
      // fall through to the fallback string
    }
  }
  return se::value_t(fallback);
}

} // namespace

std::string post_lane_task(async_lane &lane, se::async_scheduler &sched, const std::string &root,
                           const std::string &channel, lane_task_work work,
                           lane_task_on_error on_error) {
  // Resolved HERE, as a pure string (thread-safe, no tree walk), to the key a process chrooted to
  // `root` uses for (msg-recv channel): a '#' channel is verbatim, anything else is scoped.
  const std::string key = se::resolve_channel_key(root, channel);
  // Kept for the stopped-lane path: submit() destroys the job (and the moved-in on_error) when it
  // refuses it.
  lane_task_on_error refused_on_error = on_error;
  const bool queued = lane.submit([&sched, key, lane_name = lane.name(), work = std::move(work),
                                   on_error = std::move(on_error)] {
    se::value_t payload;
    try {
      payload = work();
    } catch (const std::exception &e) {
      payload = lane_error_value(on_error, e.what(), "async_lane '" + lane_name + "': " + e.what());
    } catch (...) {
      payload = lane_error_value(on_error, "unknown error",
                                 "async_lane '" + lane_name + "': unknown error");
    }
    // The thread-safe ingress of the scheduler the caller drives; it is delivered on that
    // scheduler's own thread at its next drain_ingress().
    sched.post_message(key, payload);
  });
  if (!queued) {
    const std::string msg = "async_lane '" + lane.name() + "' is stopped";
    sched.post_message(key, lane_error_value(refused_on_error, msg, msg));
  }
  return key;
}

std::string launch_lane_task(async_lane &lane, se::async_scheduler &sched, const std::string &root,
                             const std::string &chan_prefix, lane_task_work work,
                             lane_task_on_error on_error) {
  // A UNIQUE reply channel per call. The "<lane>." part keeps it disjoint from launch_pool_task's
  // "<prefix>#<n>" keys, so a pool task and a lane task with the same prefix never collide; the
  // '#' keeps it verbatim under chroot scoping and exempt from channel policy.
  static std::atomic<std::uint64_t> seq{0};
  const std::string chan = chan_prefix + "#" + lane.name() + "." +
                           std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
  return post_lane_task(lane, sched, root, chan, std::move(work), std::move(on_error));
}

se::value_t park_on_lane_task(async_lane &lane, se::async_scheduler &sched, const std::string &root,
                              const std::string &chan_prefix, lane_task_work work,
                              lane_task_on_error on_error) {
  const std::string done =
      launch_lane_task(lane, sched, root, chan_prefix, std::move(work), std::move(on_error));
  return se::park_on_channel(&sched, sched.current_process().get(), sched.current_pid(), done);
}

se::value_t future_lane_task(async_lane &lane, se::async_scheduler &sched, const std::string &root,
                             const std::string &chan_prefix, lane_task_work work,
                             lane_task_on_error on_error) {
  return se::make_future(
      launch_lane_task(lane, sched, root, chan_prefix, std::move(work), std::move(on_error)));
}

std::size_t pump_exec_frame(se::async_scheduler &sched, std::initializer_list<async_lane *> lanes,
                            std::uint64_t max_steps, double max_time) {
  std::size_t ran = 0;
  for (async_lane *lane : lanes)
    if (lane)
      ran += lane->run_deferred(); // no-thread builds: the lane's work runs here, on the pump
  sched.drain_ingress();           // deliver lane posts (sync_run alone never drains when idle)
  sched.wake_awaiting();           // (await ...) frame-yielders: once per frame
  sched.sync_step();               // unconditional: wakes due sleepers when nothing is runnable
  sched.sync_run(max_steps, max_time);
  return ran;
}

} // namespace cvc

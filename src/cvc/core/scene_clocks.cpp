// The scenes' own clocks: see inc/cvc/core/scene_clocks.h.

#include <atomic>
#include <cvc/core/app.h>
#include <cvc/core/scene_clocks.h>
#include <cvc/core/world_clock.h>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace cvc {

namespace {

// The app's scene clocks, by state prefix. `any` lets scene_clock() skip the lock while there are
// none, which is every app that mounts no scene.
struct scene_clock_slot {
  static constexpr const char *app_slot = "cvc.core.scene_clocks";
  std::mutex m;
  std::map<std::string, std::unique_ptr<world_clock>> clocks;
  std::atomic<bool> any{false};
};

} // namespace

world_clock &add_scene_clock(app &a, const std::string &prefix, double fixed_dt) {
  if (prefix.empty())
    throw std::invalid_argument("add_scene_clock: an empty prefix is the app's own clock");
  scene_clock_slot &s = a.slot<scene_clock_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  std::unique_ptr<world_clock> &c = s.clocks[prefix];
  if (!c) {
    world_clock::config cfg;
    cfg.fixed_dt = fixed_dt;
    c = std::make_unique<world_clock>(cfg);
  }
  s.any.store(true, std::memory_order_release);
  return *c;
}

void remove_scene_clock(app &a, const std::string &prefix) {
  scene_clock_slot &s = a.slot<scene_clock_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  s.clocks.erase(prefix);
  s.any.store(!s.clocks.empty(), std::memory_order_release);
}

world_clock &scene_clock(app &a, const std::string &prefix) {
  scene_clock_slot &s = a.slot<scene_clock_slot>();
  if (s.any.load(std::memory_order_acquire) && !prefix.empty()) {
    std::lock_guard<std::mutex> lock(s.m);
    // The nearest prefix at or above `prefix`: `prefix` itself, then each parent (a.b.c, a.b, a).
    std::string p = prefix;
    for (;;) {
      const auto it = s.clocks.find(p);
      if (it != s.clocks.end())
        return *it->second;
      const std::size_t dot = p.rfind('.');
      if (dot == std::string::npos)
        break;
      p.resize(dot);
    }
  }
  return a.world_clock();
}

bool has_scene_clock(app &a, const std::string &prefix) {
  scene_clock_slot &s = a.slot<scene_clock_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  return s.clocks.count(prefix) != 0;
}

} // namespace cvc

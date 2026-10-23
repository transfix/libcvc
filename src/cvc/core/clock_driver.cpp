/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/core/clock_driver.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock_binding.h>

namespace cvc {

clock_driver::clock_driver(world_clock &clock) : _clock(clock) {}

clock_driver::handle clock_driver::add(std::vector<entry> &list, bool &dirty, int order,
                                       observer_fn fn) {
  const std::uint64_t id = _next_id++;
  entry e;
  e.id = id;
  e.order = order;
  e.seq = _next_seq++;
  e.fn = std::move(fn);
  e.want_step = (&list == &_step_obs);
  if (_in_advance) {
    // Added from inside a fan-out: hold it until the current advance finishes so
    // it neither fires this frame nor perturbs the list being iterated.
    _pending.push_back(std::move(e));
  } else {
    list.push_back(std::move(e));
    dirty = true;
  }
  return handle{id};
}

clock_driver::handle clock_driver::add_step_observer(int order, observer_fn fn) {
  return add(_step_obs, _step_dirty, order, std::move(fn));
}

clock_driver::handle clock_driver::add_frame_observer(int order, observer_fn fn) {
  return add(_frame_obs, _frame_dirty, order, std::move(fn));
}

bool clock_driver::remove(handle h) {
  if (!h)
    return false;
  for (auto *list : {&_step_obs, &_frame_obs, &_pending}) {
    for (auto &e : *list) {
      if (e.id == h.id && !e.dead) {
        if (_in_advance) {
          // Mark it; fire() skips dead entries and the post-advance compaction
          // erases them. This lets an observer remove itself or a sibling.
          e.dead = true;
        } else {
          e.dead = true; // compacted below (cheap; keeps one code path)
          list->erase(
              std::remove_if(list->begin(), list->end(), [](const entry &x) { return x.dead; }),
              list->end());
        }
        return true;
      }
    }
  }
  return false;
}

void clock_driver::sort_if_dirty(std::vector<entry> &list, bool &dirty) {
  if (!dirty)
    return;
  std::stable_sort(list.begin(), list.end(), [](const entry &a, const entry &b) {
    if (a.order != b.order)
      return a.order < b.order;
    return a.seq < b.seq;
  });
  dirty = false;
}

void clock_driver::fire(std::vector<entry> &list) {
  // Index, not iterators: remove() only marks `dead` while firing and adds go to
  // _pending, so `list` is not resized here and the walk stays valid.
  for (std::size_t i = 0; i < list.size(); ++i) {
    if (!list[i].dead && list[i].fn)
      list[i].fn(_frame);
  }
}

void clock_driver::flush_pending() {
  auto drop_dead = [](std::vector<entry> &l) {
    l.erase(std::remove_if(l.begin(), l.end(), [](const entry &x) { return x.dead; }), l.end());
  };
  drop_dead(_step_obs);
  drop_dead(_frame_obs);
  if (!_pending.empty()) {
    for (auto &e : _pending) {
      if (e.dead)
        continue;
      if (e.want_step) {
        _step_obs.push_back(std::move(e));
        _step_dirty = true;
      } else {
        _frame_obs.push_back(std::move(e));
        _frame_dirty = true;
      }
    }
    _pending.clear();
  }
}

const clock_driver::clock_frame &clock_driver::advance(double wall_dt) {
  if (_in_advance)
    throw cvc::unsupported_exception("clock_driver::advance re-entered from an observer");

  double dt;
  if (wall_dt < 0.0) {
    const auto now = std::chrono::steady_clock::now();
    dt = _started ? std::chrono::duration<double>(now - _last_tp).count() : 0.0;
    _last_tp = now;
  } else {
    dt = wall_dt;
    _last_tp = std::chrono::steady_clock::now();
  }
  const bool first = !_started;
  _started = true;

  const world_clock::step_result r = _binding ? _binding->drive(dt) : _clock.advance(dt);

  _frame.step = r;
  _frame.tick = _clock.tick();
  _frame.t = _clock.t();
  _frame.alpha = r.alpha;
  _frame.fixed_dt = _clock.fixed_dt();
  _frame.wall_dt = dt;
  _frame.first = first;
  _frame.index = _index;

  auto finish = [this]() {
    _in_advance = false;
    flush_pending();
    ++_index;
  };

  _in_advance = true;
  try {
    sort_if_dirty(_step_obs, _step_dirty);
    fire(_step_obs);
    sort_if_dirty(_frame_obs, _frame_dirty);
    fire(_frame_obs);
  } catch (...) {
    finish();
    throw;
  }
  finish();
  return _frame;
}

} // namespace cvc

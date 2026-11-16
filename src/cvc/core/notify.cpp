/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/notify.h>
#include <cvc/state/state.h>
#include <deque>
#include <exception>
#include <mutex>
#include <string>
#include <utility>

namespace cvc {

namespace {

// The feed: the app's own (app::slot), never a process-wide one.
struct notify_feed {
  static constexpr const char *app_slot = "cvc.notify.feed";
  std::mutex m;
  std::deque<notification_entry> entries; // oldest first
  std::uint64_t next_id = 1;
  std::uint64_t rev = 0;
};

std::string key(std::string_view k) {
  return std::string(notify_root) + state::SEPARATOR + std::string(k);
}
std::string item_key(std::uint64_t id, std::string_view k = {}) {
  std::string p = key("items") + state::SEPARATOR + std::to_string(id);
  if (!k.empty())
    p += state::SEPARATOR + std::string(k);
  return p;
}

template <class T> void put(state &root, const std::string &path, const T &v) {
  try {
    root.childThroughLinks(path).value(v);
  } catch (const std::exception &) {
  }
}

void mirror_entry(state &root, const notification_entry &e) {
  put(root, item_key(e.id, "text"), e.text);
  put(root, item_key(e.id, "level"), std::string(notify_level_name(e.level)));
  put(root, item_key(e.id, "source"), e.source);
  put(root, item_key(e.id, "time"), e.time);
  put(root, item_key(e.id, "read"), e.read ? 1 : 0);
  put(root, item_key(e.id, "repeat"), e.repeat);
}

void drop_entry(state &root, std::uint64_t id) {
  try {
    root.childThroughLinks(key("items")).removeChild(std::to_string(id));
  } catch (const std::exception &) {
  }
}

notify_summary summarize(const notify_feed &f) {
  notify_summary s;
  s.count = f.entries.size();
  s.rev = f.rev;
  for (const notification_entry &e : f.entries)
    if (!e.read) {
      ++s.unread;
      s.worst = std::max(s.worst, e.level);
    }
  return s;
}

// The totals, after a change (the caller holds the lock).
void changed(app &a, notify_feed &f) {
  ++f.rev;
  state &root = state::instance(a);
  const notify_summary s = summarize(f);
  put(root, key("count"), static_cast<int>(s.count));
  put(root, key("unread"), static_cast<int>(s.unread));
  put(root, key("worst"), std::string(s.worst < 0 ? "" : notify_level_name(s.worst)));
  put(root, key("rev"), static_cast<int>(s.rev));
}

double now_seconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

} // namespace

std::uint64_t notify(app &a, notification n) {
  if (n.text.empty())
    return 0;
  n.level = std::clamp(n.level, 0, 2);
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  state &root = state::instance(a);
  if (!f.entries.empty()) {
    notification_entry &last = f.entries.back();
    if (!last.read && last.text == n.text && last.level == n.level && last.source == n.source) {
      ++last.repeat;
      last.time = now_seconds();
      mirror_entry(root, last);
      changed(a, f);
      return last.id;
    }
  }
  notification_entry e;
  static_cast<notification &>(e) = std::move(n);
  e.id = f.next_id++;
  e.time = now_seconds();
  f.entries.push_back(std::move(e));
  mirror_entry(root, f.entries.back());
  while (f.entries.size() > notify_capacity) {
    drop_entry(root, f.entries.front().id);
    f.entries.pop_front();
  }
  changed(a, f);
  return f.entries.back().id;
}

bool notify_read(app &a, std::uint64_t id) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  for (notification_entry &e : f.entries)
    if (e.id == id) {
      if (!e.read) {
        e.read = true;
        put(state::instance(a), item_key(id, "read"), 1);
        changed(a, f);
      }
      return true;
    }
  return false;
}

void notify_read_all(app &a) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  bool any = false;
  for (notification_entry &e : f.entries)
    if (!e.read) {
      e.read = true;
      put(state::instance(a), item_key(e.id, "read"), 1);
      any = true;
    }
  if (any)
    changed(a, f);
}

bool notify_clear(app &a, std::uint64_t id) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  const auto it = std::find_if(f.entries.begin(), f.entries.end(),
                               [id](const notification_entry &e) { return e.id == id; });
  if (it == f.entries.end())
    return false;
  drop_entry(state::instance(a), id);
  f.entries.erase(it);
  changed(a, f);
  return true;
}

std::size_t notify_clear_under(app &a, const std::string &path) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  const auto under = [&](const notification_entry &e) {
    if (path.empty())
      return true;
    return e.path == path ||
           (e.path.size() > path.size() && e.path.compare(0, path.size(), path) == 0 &&
            e.path[path.size()] == state::SEPARATOR[0]);
  };
  std::size_t n = 0;
  for (auto it = f.entries.begin(); it != f.entries.end();) {
    if (under(*it)) {
      drop_entry(state::instance(a), it->id);
      it = f.entries.erase(it);
      ++n;
    } else {
      ++it;
    }
  }
  if (n > 0)
    changed(a, f);
  return n;
}

void notify_clear_all(app &a) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  if (f.entries.empty())
    return;
  for (const notification_entry &e : f.entries)
    drop_entry(state::instance(a), e.id);
  f.entries.clear();
  changed(a, f);
}

std::vector<notification_entry> notifications(app &a) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  return {f.entries.begin(), f.entries.end()};
}

notify_summary notify_status(app &a) {
  notify_feed &f = a.slot<notify_feed>();
  std::lock_guard<std::mutex> lock(f.m);
  return summarize(f);
}

int notify_level(std::string_view name) {
  if (name == "warn" || name == "warning")
    return 1;
  if (name == "error")
    return 2;
  return 0;
}

const char *notify_level_name(int level) {
  return level >= 2 ? "error" : level == 1 ? "warn" : "info";
}

} // namespace cvc

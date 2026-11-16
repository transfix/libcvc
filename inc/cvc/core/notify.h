/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The host's notification feed (docs/ARIADNE.md §5.24): one per cvc::app, so two apps in a process
// never share one. Everything that runs in the app posts to the same feed: the outermost document,
// every document it loads, the host, and the jobs they start. Ariadne's menubar shows it as the
// tray's bell. Nothing in it replicates: it lives outside the conference's shared tree (conf.*).
//
//   cvc::notification n;
//   n.text = "Meshing finished";
//   n.source = "fe_lab";
//   cvc::notify(app, n);
//
// The feed keeps the newest notify_capacity entries, dropping the oldest. A post identical to the
// newest entry while that is unread (its text, level and source) counts as a repeat of it rather
// than a new entry. It mirrors itself into the state tree, for a document to read, under
// notify_root:
//
//   notify.count     how many entries
//   notify.unread    how many of them are unread
//   notify.worst     the worst unread level: "info", "warn", "error", or "" when none is unread
//   notify.rev       bumped by every change
//   notify.items.<id>.{text, level, source, time, read, repeat}
//
// An Ariadne widget binds them app-root-absolute (`bind: /notify.unread`); a program, confined to
// its document's state, reads the feed with (notify-status) and (notify-list). Every function is
// safe to call from any thread; the state writes happen on the caller's.

#ifndef CVC_CORE_NOTIFY_H
#define CVC_CORE_NOTIFY_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
class app;

struct notification {
  std::string text;
  int level = 0;      // 0 info, 1 warn, 2 error
  std::string source; // who posted it, as the feed names it ("fe_lab"); "" names no one
  std::string path;   // the state path of what posted it (a document's scope, a job's keys)
  std::string on;     // a program to run when the entry is clicked; "" none
  std::string scope;  // the state prefix `on` runs in
};

struct notification_entry : notification {
  std::uint64_t id = 0;
  double time = 0.0; // when it was posted (its latest repeat), seconds since the epoch
  bool read = false;
  int repeat = 1; // how many identical posts it stands for
};

inline constexpr std::size_t notify_capacity = 100;
inline constexpr const char *notify_root = "notify";

// Post `n`; returns its entry's id (the repeated entry's, for a repeat). An empty text posts
// nothing and returns 0.
std::uint64_t notify(app &a, notification n);

// Mark entry `id` read; false when there is none.
bool notify_read(app &a, std::uint64_t id);
void notify_read_all(app &a);
// Drop entry `id`; false when there is none.
bool notify_clear(app &a, std::uint64_t id);
// Drop the entries whose path is `path` or under it (a document's, when it restarts); how many.
std::size_t notify_clear_under(app &a, const std::string &path);
void notify_clear_all(app &a);

// The entries, oldest first.
std::vector<notification_entry> notifications(app &a);

struct notify_summary {
  std::size_t count = 0;
  std::size_t unread = 0;
  int worst = -1;        // the worst unread level, -1 when none is unread
  std::uint64_t rev = 0; // bumped by every change
};
notify_summary notify_status(app &a);

// "warn" / "warning" -> 1, "error" -> 2, anything else -> 0; and back ("info", "warn", "error").
int notify_level(std::string_view name);
const char *notify_level_name(int level);

} // namespace cvc

#endif

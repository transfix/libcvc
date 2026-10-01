/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cvc/gl/SceneEventSink.h>
#include <utility>

namespace cvc {
namespace gl {

SceneEventSink::SceneEventSink() : m_owner(std::this_thread::get_id()) {}

bool SceneEventSink::post(std::function<void()> callback) {
  // On refusal `callback` dies with the parameter, after the lock is released.
  return tryPost(callback);
}

bool SceneEventSink::tryPost(std::function<void()> &callback) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_closed)
    return false; // `callback` is left to the caller
  m_queue.push(std::move(callback));
  m_renderNeeded = true;
  return true;
}

bool SceneEventSink::postCoalesced(const void *key, std::function<void()> callback) {
  // The callback this one replaces is destroyed AFTER the lock is released: its
  // captures may take other locks on the way out (a Python callable's deleter
  // takes the GIL, which the owner thread may hold while it waits for us).
  std::function<void()> replaced;
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_closed)
    return false;
  m_renderNeeded = true;
  auto it = m_coalesced.find(key);
  if (it != m_coalesced.end()) {
    replaced = std::move(it->second); // a slot is already queued: latest wins
    it->second = std::move(callback);
    return true;
  }
  m_coalesced.emplace(key, std::move(callback));
  // One slot per key, at the position of the first post since the last drain.
  // It runs whatever callback is latest when the slot is reached.
  m_queue.push([this, key]() { runCoalesced(key); });
  return true;
}

void SceneEventSink::runCoalesced(const void *key) {
  std::function<void()> fn;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_coalesced.find(key);
    if (it == m_coalesced.end())
      return;
    fn = std::move(it->second);
    m_coalesced.erase(it); // a post from here on queues a new slot (next drain)
  }
  if (fn)
    fn();
}

void SceneEventSink::processEvents() {
  // Take the queue under the lock, run it without: callbacks post more work.
  std::queue<std::function<void()>> events;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::swap(events, m_queue);
  }
  while (!events.empty()) {
    std::function<void()> callback = std::move(events.front());
    events.pop();
    if (!callback)
      continue;
    try {
      callback();
    } catch (...) {
      // Dropping the rest would strand every coalesced key among them (its map
      // entry stays, so later posts for it would never queue a slot again).
      // Put them back ahead of whatever was posted meanwhile, then rethrow.
      std::queue<std::function<void()>> dropped; // only if the sink was closed
      {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_closed) {
          std::swap(dropped, events);
        } else {
          while (!m_queue.empty()) {
            events.push(std::move(m_queue.front()));
            m_queue.pop();
          }
          std::swap(events, m_queue);
        }
      }
      throw;
    }
  }
}

void SceneEventSink::requestRender() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_renderNeeded = true;
}

bool SceneEventSink::checkAndResetRenderNeeded() {
  std::lock_guard<std::mutex> lock(m_mutex);
  const bool needed = m_renderNeeded;
  m_renderNeeded = false;
  return needed;
}

void SceneEventSink::close() {
  std::queue<std::function<void()>> queue;
  std::unordered_map<const void *, std::function<void()>> coalesced;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_closed = true;
    std::swap(queue, m_queue);
    std::swap(coalesced, m_coalesced);
  }
  // Destroyed here, outside the lock (see postCoalesced).
}

bool SceneEventSink::closed() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_closed;
}

} // namespace gl
} // namespace cvc

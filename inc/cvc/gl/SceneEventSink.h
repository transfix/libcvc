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

#ifndef CVC_GL_SCENE_EVENT_SINK_H
#define CVC_GL_SCENE_EVENT_SINK_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

namespace cvc {
namespace gl {

// ---------------------------------------------------------------------------
// SceneEventSink -- a SceneGraph's event queue (postEvent / postEventCoalesced
// / processEvents), its render-needed flag and its owner thread, in an object
// of its own that the scene holds by std::shared_ptr.
//
// It exists for producer threads. A node asks for its scene, gets a raw
// pointer, and posts through it -- while the owner thread may be destroying
// that scene. Holding the SINK instead (SceneNode::sceneEvents() hands out a
// locked shared_ptr) keeps the queue, its mutex and its flags valid for as
// long as the producer uses them; ~SceneGraph closes the sink, after which
// every post is refused (returns false) instead of touching freed memory. A
// refused post leaves the producer's own state as it was: a streaming node's
// staged writes stay pending and are applied when it is attached again.
// ~SceneGraph resets the scene's alive token BEFORE closing the sink, so by the
// time a post is refused SceneNode::getSceneGraph() is already nullptr.
class SceneEventSink {
public:
  SceneEventSink(); // owned by the constructing thread

  // Queue `callback` for the next processEvents(). Any thread. False (the
  // callback is dropped) once the sink is closed.
  bool post(std::function<void()> callback);
  // post() that takes `callback` only if it accepts it: on refusal (closed)
  // the caller still owns it, untouched -- for a fallback without a copy.
  bool tryPost(std::function<void()> &callback);
  // Coalesced by `key` -- see SceneGraph::postEventCoalesced. Any thread. False
  // once the sink is closed.
  bool postCoalesced(const void *key, std::function<void()> callback);

  // Run everything queued so far, in order. Owner thread. If a callback throws,
  // the callbacks not yet run go back to the FRONT of the queue (so a coalesced
  // key keeps its slot and no later post for it is stranded) and the exception
  // propagates; the next processEvents() picks up where this one stopped.
  void processEvents();

  void requestRender();
  bool checkAndResetRenderNeeded();

  bool onOwnerThread() const { return std::this_thread::get_id() == m_owner.load(); }
  void adoptOwnerThread() { m_owner.store(std::this_thread::get_id()); }

  // Refuse every later post and drop whatever is still queued (~SceneGraph,
  // after its last drain). Idempotent.
  void close();
  bool closed() const;

private:
  void runCoalesced(const void *key);

  mutable std::mutex m_mutex;
  std::queue<std::function<void()>> m_queue;
  // The latest callback per key; a key is present exactly while its single
  // queue slot is waiting to run.
  std::unordered_map<const void *, std::function<void()>> m_coalesced;
  bool m_renderNeeded = false;
  bool m_closed = false;
  std::atomic<std::thread::id> m_owner;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_SCENE_EVENT_SINK_H

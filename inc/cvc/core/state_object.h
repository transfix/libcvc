/*
  Copyright 2012 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

/* $Id: StateObject.h 5883 2012-07-20 19:52:38Z transfix $ */

#ifndef __CVC_STATE_OBJECT_H__
#define __CVC_STATE_OBJECT_H__

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <set>
#include <string>
#include <vector>

namespace cvc {
// Forward declaration
template <class This> class state_object;

namespace detail {

// Tracks the handler threads a state_object has in flight, so the object's
// destructor can wait for them.
//
// Why this exists rather than joining app::threads(): the handler lambda wraps
// itself in a cvc::app::thread_feedback whose destructor calls
// finishThreadProgress(), and that ERASES the thread's own entry from the app's
// _threads map -- from inside the still-running thread. Erasing drops the last
// shared_ptr to the boost::thread, whose destructor DETACHES it. So by the time
// ~state_object() takes its snapshot of the map, a live handler thread can be
// invisible: never joined, still running, and free to touch this object after
// it is gone. That is a use-after-free, and it corrupts the heap
// ("malloc_consolidate(): unaligned fastbin chunk") well away from the cause.
//
// The gate is owned by a shared_ptr the lambda captures BY VALUE, so it stays
// alive even if everything else does not, and it counts handlers rather than
// identifying threads -- no name prefixes, no map lookups, nothing to miss.
struct handler_gate {
  boost::mutex mutex;
  boost::condition_variable done;
  int inflight;
  bool shutting_down;
  handler_gate() : inflight(0), shutting_down(false) {}
};

// RAII: decrement on EVERY exit path, including boost::thread_interrupted --
// startThread() interrupts a thread whose key it is about to reuse, and a
// handler killed that way must still be accounted for or the destructor hangs.
class handler_scope {
public:
  explicit handler_scope(const boost::shared_ptr<handler_gate> &gate) : _gate(gate) {
    boost::mutex::scoped_lock lock(_gate->mutex);
    _cancelled = _gate->shutting_down;
  }
  ~handler_scope() {
    boost::mutex::scoped_lock lock(_gate->mutex);
    if (--_gate->inflight <= 0) {
      _gate->inflight = 0;
      _gate->done.notify_all();
    }
  }
  // The object began destruction before this handler got to run. Skip the
  // virtual call: the derived subobject may already be gone.
  bool cancelled() const { return _cancelled; }

private:
  boost::shared_ptr<handler_gate> _gate;
  bool _cancelled;
};

} // namespace detail

// -------------------------
// cvc::state_lock_scope
// -------------------------
// Purpose:
//   RAII wrapper for exclusively locking a state_object.
//   While this lock is held, other threads attempting to acquire
//   a lock on the same state_object will block.
//
//   This provides a coordination mechanism for exclusive access to
//   state modifications. The lock itself doesn't prevent direct state
//   access - it's a synchronization primitive for coordinating between
//   threads that respect the locking protocol.
//
//   Usage:
//     {
//       state_lock_scope<MyObject> lock(myObject);
//       // Exclusive access - other lock holders will block
//       myObject.getState("width").value(1920);
//       myObject.getState("height").value(1080);
//     } // Lock released, blocked threads can now acquire
//
// ---- Change History ----
// 12/23/2025 -- Joe R. -- Creation.
template <class This> class state_lock_scope {
public:
  explicit state_lock_scope(state_object<This> &obj) : _obj(obj), _released(false) {
    _obj.lockState();
  }

  ~state_lock_scope() {
    if (!_released) {
      _obj.unlockState();
    }
  }

  // Manually release lock before scope ends
  void unlock() {
    if (!_released) {
      _obj.unlockState();
      _released = true;
    }
  }

  // Non-copyable, non-movable
  state_lock_scope(const state_lock_scope &) = delete;
  state_lock_scope &operator=(const state_lock_scope &) = delete;

private:
  state_object<This> &_obj;
  bool _released;
};

// -----------------------------
// cvc::state_change_batch_scope
// -----------------------------
// Purpose:
//   RAII wrapper for batching state changes. While this object is alive,
//   state change handlers are queued instead of immediately spawned.
//   When destroyed (or flush() is called), all unique pending handlers run.
//
//   Usage:
//     {
//       state_change_batch_scope batch(myObject);
//       myObject.getState("width").value(1920);   // queued
//       myObject.getState("height").value(1080);  // queued
//       myObject.getState("width").value(2560);   // replaces first width change
//     } // handlers run here (only once per changed state)
//
// ---- Change History ----
// 12/23/2025 -- Joe R. -- Creation.
template <class This> class state_change_batch_scope {
public:
  explicit state_change_batch_scope(state_object<This> &obj) : _obj(obj), _flushed(false) {
    _obj.beginBatch();
  }

  ~state_change_batch_scope() {
    if (!_flushed) {
      _obj.endBatch();
    }
  }

  // Manually flush pending changes before scope ends
  void flush() {
    if (!_flushed) {
      _obj.endBatch();
      _flushed = true;
    }
  }

  // Non-copyable
  state_change_batch_scope(const state_change_batch_scope &) = delete;
  state_change_batch_scope &operator=(const state_change_batch_scope &) = delete;

private:
  state_object<This> &_obj;
  bool _flushed;
};

// ----------------------
// cvc::state_init_scope
// ----------------------
// Purpose:
//   RAII wrapper for *establishing* initial state on a state_object
//   without firing change handlers. Use this in the body of a derived
//   class's constructor (and only there) so that seed values do not
//   spawn handler threads that would race with subsequent caller-
//   initiated changes.
//
//   Without this, an object like:
//
//     class Cfg : public state_object<Cfg> {
//       Cfg() {
//         getState("width").value(1920);   // spawns thread T1
//         getState("height").value(1080);  // spawns thread T2
//       }
//     };
//
//   has a window between construction and the caller's first action
//   in which T1/T2 have been spawned but not yet finished. Their
//   completion is unordered with respect to the caller's batch and
//   waitForHandlers() observations, producing flaky handler-count
//   accounting (initial vs final snapshot races).
//
//   Usage:
//     Cfg() : ... {
//       state_init_scope<Cfg> init(*this);
//       getState("width").value(1920);   // change signal suppressed
//       getState("height").value(1080);  // change signal suppressed
//     } // scope ends; queued changes discarded, no handlers spawned.
//
// ---- Change History ----
// 04/2026 -- Joe R. -- Creation. Added to fix StateObjectBatchedChanges /
//                      StateObjectNestedBatching race on macOS Release.
template <class This> class state_init_scope {
public:
  explicit state_init_scope(state_object<This> &obj) : _obj(obj) { _obj.beginInit(); }

  ~state_init_scope() { _obj.endInit(); }

  // Non-copyable
  state_init_scope(const state_init_scope &) = delete;
  state_init_scope &operator=(const state_init_scope &) = delete;

private:
  state_object<This> &_obj;
};

// -----------------
// cvc::state_object
// -----------------
// Purpose:
//   This class is for making it more convenient to use the cvc::state heirarchy
//   to store class member data for your object.
//
//   Use it like this:
//
//      class new_object : public cvc::state_object<new_object>
//      {
//        ...
//      protected:
//        virtual void handleStateChanged(const std::string& childState);
//      };
//
//   Then later, you can do stuff like this:
//
//      new_object *p = ...
//      p->getState("member_variable").value<int>(1234);
//
//   The advantage of this being that you can easily monitor changes to an object's
//   state as long as it is using the state graph to store it's data.  You can also
//   change an object's state easily and it should respond to each change in a meaningful
//   way.
//
// ---- Change History ----
// 05/27/2012 -- Joe R. -- Creation.
// 12/23/2025 -- Joe R. -- Added batching support to avoid thread floods.
template <class This> // This should be the type of the inheriting class

class state_object {
public:
  // Constructor with explicit app context
  state_object(app &ctx, const std::string state_path = std::string())
      : _ctx(ctx), _batchDepth(0), _initDepth(0), _hasInstanceThreading(false),
        _instanceThreading(false), _handlerGate(boost::make_shared<detail::handler_gate>()),
        _state_path(state_path.empty() ? _ctx.dataTypeName<This>() + cvc::state::SEPARATOR +
                                             boost::lexical_cast<std::string>(this)
                                       : state_path) {
    // Register the data type - avoid the macro by using template syntax
    _ctx.template registerDataType<This>(_ctx.dataTypeName<This>());

    // watch this object's state
    _stateConnection = getState().childChanged.connect(map_change_signal::slot_type(
        &state_object<This>::stateChanged, this, boost::placeholders::_1));
  }

  // NOTE: a handler thread that has been spawned but has not dispatched yet
  // will still run while this destructor is joining it, and by then the
  // derived subobject is gone -- so virtual dispatch lands on the base
  // handleStateChanged() below rather than the override.  That is benign (the
  // base only logs, and the base subobject is alive for the whole join), but
  // it is inherent: a base destructor runs after the derived one, so it has no
  // hook early enough to turn such a handler away.  A derived class that needs
  // its override never called during teardown must call disconnectState() and
  // waitForHandlers() from its own destructor, before its members go.
  ~state_object() {
    // A destructor is implicitly noexcept and almost everything below can
    // throw: app::threads() and app::removeThread() both open with an
    // interruption_point(), and join()/timed_join() throw if this thread has
    // been interrupted.  Letting any of that escape calls std::terminate().
    try {
      _stateConnection.disconnect();

      // Quiesce in-flight handlers FIRST. This is the authoritative wait: it
      // counts handlers this object actually spawned, so it cannot miss one
      // that has already removed itself from the app's thread map (see
      // detail::handler_gate). Bounded by the same 5s budget the map scan
      // below uses, so a wedged handler degrades to the old interrupt path
      // instead of hanging a process forever.
      {
        boost::mutex::scoped_lock lock(_handlerGate->mutex);
        _handlerGate->shutting_down = true;
        boost::system_time deadline =
            boost::get_system_time() + boost::posix_time::milliseconds(5000);
        while (_handlerGate->inflight > 0) {
          if (!_handlerGate->done.timed_wait(lock, deadline)) {
            _ctx.log(5, str(boost::format("state_object::~state_object(): %d handler(s) still "
                                          "running after 5s; falling back to interrupt") %
                            _handlerGate->inflight));
            break;
          }
        }
      }

      // Wait for handler threads to complete to avoid dangling references
      // Handler threads are named as stateName(childState) + "_stateChanged"
      std::string threadPrefix = stateName();
      std::vector<std::string> joined;
      thread_map threads = _ctx.threads();
      BOOST_FOREACH (thread_map::value_type &val, threads) {
        // Check if this thread belongs to this state_object instance
        if (val.first.find(threadPrefix) == 0 &&
            val.first.find("_stateChanged") != std::string::npos) {
          // Wait for this handler thread to complete
          if (val.second && val.second->joinable()) {
            if (!val.second->timed_join(boost::posix_time::milliseconds(5000))) {
              _ctx.log(5, str(boost::format("state_object::~state_object(): thread %s did not "
                                            "finish in time, interrupting") %
                              val.first));
              val.second->interrupt();
              val.second->join();
            }
          }
          if (val.second)
            joined.push_back(val.first);
        }
      }

      // Reap what we joined.  app::finishThreadProgress() no longer evicts a
      // thread that is still running, so the entry outlives the handler and it
      // falls to the owning object to drop it -- otherwise the app's thread map
      // keeps one entry per handled child state for the life of the app.  Safe
      // here and only here: the signal is disconnected, so nothing can have
      // registered a *new* thread under one of these keys.
      threads.clear();
      BOOST_FOREACH (const std::string &key, joined)
        _ctx.removeThread(key);
    } catch (...) {
      // Nothing useful to do while unwinding a destructor.
    }
  }

  // Use this to easily get the name of either this state object or it's children.
  std::string stateName(const std::string &childState = std::string()) const {
    return !childState.empty() ? _state_path + cvc::state::SEPARATOR + childState : _state_path;
  }

  // Shortcut for accessing the state corresponding to an instance of this
  // class or it's children. Uses this object's bound app context rather
  // than the global cvcstate singleton.
  state &getState(const std::string &s = std::string()) const {
    return cvc::state::instance(_ctx)(stateName(s));
  }

  // Begin batching state changes - handlers are queued instead of spawned
  void beginBatch() {
    boost::mutex::scoped_lock lock(_batchMutex);
    _batchDepth++;
  }

  // Begin an *initialization* region. While _initDepth > 0, change
  // signals received by stateChanged() are dropped instead of being
  // queued or spawned. Used by state_init_scope to set seed values in
  // a constructor without spawning racy handler threads.
  void beginInit() {
    boost::mutex::scoped_lock lock(_batchMutex);
    _initDepth++;
  }

  // End an initialization region. Pairs with beginInit().
  void endInit() {
    boost::mutex::scoped_lock lock(_batchMutex);
    if (_initDepth > 0) {
      _initDepth--;
    }
  }

  // End batching and flush all pending handlers (spawn threads for unique changes)
  void endBatch() {
    std::set<std::string> pendingCopy;

    {
      boost::mutex::scoped_lock lock(_batchMutex);
      if (_batchDepth > 0) {
        _batchDepth--;
      }

      // Only flush if we're at depth 0 (supports nested batching)
      if (_batchDepth == 0 && !_pendingChanges.empty()) {
        pendingCopy = _pendingChanges;
        _pendingChanges.clear();
      }
    }

    // Spawn threads outside the lock
    bool useThreading = _hasInstanceThreading ? _instanceThreading : _useThreading;
    if (useThreading) {
      // Threading enabled - spawn threads for each change
      boost::shared_ptr<detail::handler_gate> gate = _handlerGate;
      cvc::app *ctxp = &_ctx;
      BOOST_FOREACH (const std::string &childState, pendingCopy) {
        std::string threadKey = stateName(childState) + "_stateChanged";
        {
          boost::mutex::scoped_lock g(gate->mutex);
          if (gate->shutting_down)
            break;
          ++gate->inflight;
        }
        // Was a bare boost::bind on handleStateChanged with no gate and no
        // thread_feedback -- the batched path had the same use-after-free as
        // the single-change path above.
        _ctx.startThread(threadKey, [this, ctxp, childState, threadKey, gate]() {
          detail::handler_scope scope(gate);
          cvc::app::thread_feedback feedback(*ctxp, threadKey);
          if (scope.cancelled())
            return;
          this->handleStateChanged(childState);
        });
      }
    } else {
      // Threading disabled - call synchronously
      BOOST_FOREACH (const std::string &childState, pendingCopy) {
        handleStateChanged(childState);
      }
    }
  }

  // Wait for all handler threads to complete
  void waitForHandlers() {
    // Wait on the gate first -- same reason the destructor does: a handler that
    // has already erased its own app::threads() entry is invisible to the scan
    // below, so the scan alone can report "done" while a handler still runs.
    {
      boost::mutex::scoped_lock lock(_handlerGate->mutex);
      while (_handlerGate->inflight > 0)
        _handlerGate->done.wait(lock);
    }

    std::string threadPrefix = stateName();
    thread_map threads = _ctx.threads();
    BOOST_FOREACH (thread_map::value_type &val, threads) {
      if (val.first.find(threadPrefix) == 0 &&
          val.first.find("_stateChanged") != std::string::npos) {
        if (val.second && val.second->joinable()) {
          val.second->join();
        }
      }
    }
  }

  // Lock state - blocks other threads from modifying this object or children
  void lockState() { _stateLockMutex.lock(); }

  // Unlock state - allows blocked threads to proceed
  void unlockState() { _stateLockMutex.unlock(); }

  // Control whether handleStateChanged runs in threads (default true)
  // Set to false in tests to avoid threading issues during destruction
  static void setUseThreading(bool useThreading) { _useThreading = useThreading; }

  static bool getUseThreading() { return _useThreading; }

  // Per-instance threading control (overrides static if set)
  void setInstanceThreading(bool useThreading) {
    _instanceThreading = useThreading;
    _hasInstanceThreading = true;
  }

  bool getInstanceThreading() const {
    if (_hasInstanceThreading) {
      return _instanceThreading;
    }
    return _useThreading;
  }

  void clearInstanceThreading() { _hasInstanceThreading = false; }

protected:
  app &_ctx;
  std::string _state_path;

  boost::signals2::connection _stateConnection;

  // Handler-thread lifetime. See detail::handler_gate: the destructor waits on
  // this rather than trusting app::threads(), which a finishing handler removes
  // itself from while still running.
  boost::shared_ptr<detail::handler_gate> _handlerGate;

  // Batching support
  mutable boost::mutex _batchMutex;
  int _batchDepth;
  int _initDepth;
  std::set<std::string> _pendingChanges;

  // State locking support
  mutable boost::mutex _stateLockMutex;

  // Per-instance threading control
  bool _hasInstanceThreading;
  bool _instanceThreading;

  // Threading control - can be disabled for tests
  static bool _useThreading;

  // Disconnect from state tree to prevent NEW callbacks during destruction
  // Call this in derived class destructors to avoid pure virtual method calls
  // Note: Cannot safely wait for in-flight callbacks as they may be calling methods on 'this'
  void disconnectState() { _stateConnection.disconnect(); }

  // Classes that are state_objects should implement this function for themselves.
  // Note: each call happens in its own thread (unless threading is disabled).
  virtual void handleStateChanged(const std::string &childState) {
    _ctx.log(2,
             str(boost::format("%s :: state changed: %s\n") % BOOST_CURRENT_FUNCTION % childState));
  }

private:
  // Responding to state changes.  Every change will launch a new thread and will
  // immediately return, therefore this call is non-blocking.
  // If batching is enabled, changes are queued instead.
  void stateChanged(const std::string &childState) {
    boost::mutex::scoped_lock lock(_batchMutex);

    // Initialization region: drop the change entirely. Used by
    // state_init_scope to seed default state from a constructor
    // without spawning handler threads that would race with the
    // caller's first observable interactions.
    if (_initDepth > 0) {
      return;
    }

    if (_batchDepth > 0) {
      // Batching enabled - queue this change (set automatically deduplicates)
      _pendingChanges.insert(childState);
    } else {
      bool useThreading = _hasInstanceThreading ? _instanceThreading : _useThreading;
      if (useThreading) {
        // Threading enabled - spawn thread (unlock first to avoid holding lock)
        lock.unlock();
        std::string threadKey = stateName(childState) + "_stateChanged";
        // Register with the gate BEFORE spawning, so the destructor can never
        // observe inflight==0 while a handler is on its way in.
        boost::shared_ptr<detail::handler_gate> gate = _handlerGate;
        {
          boost::mutex::scoped_lock g(gate->mutex);
          if (gate->shutting_down)
            return;
          ++gate->inflight;
        }
        // Capture the app by pointer, not through `this`: if the object is
        // already going away, the handler must be able to build its
        // thread_feedback and unwind without dereferencing a dead object.
        cvc::app *ctxp = &_ctx;
        _ctx.startThread(threadKey, [this, ctxp, childState, threadKey, gate]() {
          detail::handler_scope scope(gate);
          cvc::app::thread_feedback feedback(*ctxp, threadKey);
          if (scope.cancelled())
            return;
          this->handleStateChanged(childState);
        });
      } else {
        // Threading disabled - call synchronously (unlock first)
        lock.unlock();
        handleStateChanged(childState);
      }
    }
  }
};

// Initialize static member - threading enabled by default
template <class This> bool state_object<This>::_useThreading = true;
} // namespace cvc

#endif // __CVC_STATE_OBJECT_H__

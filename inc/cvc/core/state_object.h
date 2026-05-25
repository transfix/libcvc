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

namespace cvc {
// Forward declaration
template <class This> class state_object;

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
        _instanceThreading(false),
        _state_path(state_path.empty()
                        ? _ctx.dataTypeName<This>() + cvc::state::SEPARATOR +
                              boost::lexical_cast<std::string>(this)
                        : state_path) {
    // Register the data type - avoid the macro by using template syntax
    _ctx.template registerDataType<This>(_ctx.dataTypeName<This>());

    // watch this object's state
    _stateConnection = getState().childChanged.connect(map_change_signal::slot_type(
        &state_object<This>::stateChanged, this, boost::placeholders::_1));
  }

  ~state_object() {
    _stateConnection.disconnect();

    // Wait for handler threads to complete to avoid dangling references
    // Handler threads are named as stateName(childState) + "_stateChanged"
    std::string threadPrefix = stateName();
    thread_map threads = _ctx.threads();
    BOOST_FOREACH (thread_map::value_type &val, threads) {
      // Check if this thread belongs to this state_object instance
      if (val.first.find(threadPrefix) == 0 &&
          val.first.find("_stateChanged") != std::string::npos) {
        // Wait for this handler thread to complete
        if (val.second && val.second->joinable()) {
          if (!val.second->timed_join(boost::posix_time::milliseconds(5000))) {
            _ctx.log(5, str(boost::format("state_object::~state_object(): thread %s did not finish "
                                          "in time, interrupting") %
                            val.first));
            val.second->interrupt();
            val.second->join();
          }
        }
      }
    }
  }

  // Use this to easily get the name of either this state object or it's children.
  std::string stateName(const std::string &childState = std::string()) const {
    return !childState.empty() ? _state_path + cvc::state::SEPARATOR + childState
                               : _state_path;
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
      BOOST_FOREACH (const std::string &childState, pendingCopy) {
        _ctx.startThread(
            stateName(childState) + "_stateChanged",
            boost::bind(&state_object<This>::handleStateChanged, boost::ref(*this), childState));
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
        _ctx.startThread(threadKey, [this, childState, threadKey]() {
          cvc::app::thread_feedback feedback(_ctx, threadKey);
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

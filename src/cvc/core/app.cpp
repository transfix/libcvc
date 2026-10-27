/*
  Copyright 2011 The University of Texas at Austin

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

/* $Id: App.cpp 5881 2012-07-20 19:34:04Z edwardsj $ */

#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/core/types.h>
#include <cvc/core/world_clock.h>
#include <cvc/core/world_units.h>
#include <cvc/geometry/geometry.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/utility/utility.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/io_handlers.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/voxels.h>

#ifdef USING_LOG4CPLUS_DEFAULT
#include <log4cplus/configurator.h>
#include <log4cplus/fileappender.h>
#include <log4cplus/logger.h>
#include <log4cplus/loglevel.h>
#endif

#ifndef CVC_APP_XML_PROPERTY_MAP
#include <boost/property_tree/info_parser.hpp>
#else
#include <boost/property_tree/xml_parser.hpp>
#endif

#include <algorithm>
#include <atomic>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread/xtime.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <queue>
#include <set>
#include <utility>

namespace cvc {
// ---------------------
// app::wait_for_threads
// ---------------------
// Purpose:
//   Waits until all threads are finished before exiting the main
//   thread.  This is needed because child threads might refence objects
//   that are destroyed by the C++ runtime during exit().
// ---- Change History ----
// 09/09/2011 -- Joe R. -- Creation.
// 02/24/2012 -- Joe R. -- Moved to class App so we can support checking whether the main
//                         thread has quit.
// 12/12/2025 -- Joe R. -- Two-phase shutdown: interrupt all, then join with timeout.
// 10/27/2026 -- Joe R. -- Waits with interruption held off, so an interrupt sent to the caller
//                         stays pending.
void app::wait_for_threads() {
  using namespace cvc;

  // Act on THIS app's own tracked threads. (Previously keyed off a process-wide
  // app singleton, now removed — cvc::app carries no singleton, so callers wait
  // on the instance they hold.)
  app &a = *this;

  // Get all the threads
  thread_map map = a.threads();

  // Phase 1: Interrupt all threads to signal them to exit
  // Many algorithms have interruption checkpoints that allow clean exit
  BOOST_FOREACH (thread_map::value_type val, map) {
    try {
      if (val.second)
        val.second->interrupt();
    } catch (...) {
    }
  }

  // The background pool's anonymous tasks aren't in the map: interrupt the ones running on its
  // workers too, as when every pooled task had a thread in the map. (Under the pool's mutex, so
  // that a worker taking its next task doesn't take the interrupt for that task.)
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    interruptPoolWorkersLocked();
  }

  // Phase 2: Attempt to join all threads with a timeout. Not interruptible, as thread_info and
  // thread_feedback keep their books: an interrupt sent to this thread meanwhile stays pending
  // until we return, where it used to be thrown in a join or the pool wait and dropped by the
  // catch there. (The 5 s bounds still end the wait.)
  boost::this_thread::disable_interruption keep_interrupt_pending;
  std::vector<std::string> failed_threads;
  BOOST_FOREACH (thread_map::value_type val, map) {
    try {
      using namespace boost;
      if (val.second && val.second->joinable()) {
        a.log(3, str(boost::format("%s :: waiting for thread %s\n") % BOOST_CURRENT_FUNCTION %
                     val.first));
        // Try to join with a 5 second timeout per thread
        if (!val.second->try_join_for(boost::chrono::seconds(5))) {
          // Thread didn't join in time, track it
          failed_threads.push_back(val.first);
        }
      }
    } catch (...) {
    }
  }

  // ... and wait for the background pool to finish what is queued or running.
  try {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    if (!_threadPoolIdle.timed_wait(lock, boost::posix_time::seconds(5), [this] {
          return _poolStopping || (_activeWorkers == 0 && _pendingTasks.empty());
        }))
      failed_threads.push_back(str(boost::format("background pool (%d running, %d queued)") %
                                   _activeWorkers % _pendingTasks.size()));
  } catch (...) {
  }

  // Phase 3: Report any threads that failed to join
  if (!failed_threads.empty()) {
    using namespace boost;
    std::string msg = str(boost::format("%s :: WARNING: %d thread(s) failed to join within timeout "
                                        "and may cause cleanup issues:\n") %
                          BOOST_CURRENT_FUNCTION % failed_threads.size());
    BOOST_FOREACH (const std::string &thread_name, failed_threads) {
      msg += str(boost::format("  - %s\n") % thread_name);
    }
    a.log(0, msg);    // Error level
    std::cerr << msg; // Also print to stderr to ensure visibility
  }
}

app::app()
    : _maxPoolSize(boost::thread::hardware_concurrency() > 0 ? boost::thread::hardware_concurrency()
                                                             : 4),
      _activeWorkers(0) {
  registerDefaultTypes();
  registerDefaultHandlers();
}

app::app(no_init_t)
    : _maxPoolSize(boost::thread::hardware_concurrency() > 0 ? boost::thread::hardware_concurrency()
                                                             : 4),
      _activeWorkers(0) {}

void app::registerDefaultTypes() {
  registerDataType(char);
  registerDataType<char>(Char);
  registerDataType(unsigned char);
  registerDataType<unsigned char>(UChar);
  registerDataType(unsigned short);
  registerDataType<unsigned short>(UShort);
  registerDataType(int);
  registerDataType<int>(Int);
  registerDataType(unsigned int);
  registerDataType<unsigned int>(UInt);
  registerDataType(float);
  registerDataType<float>(Float);
  registerDataType(double);
  registerDataType<double>(Double);
  registerDataType(int64);
  registerDataType<int64>(Int64);
  registerDataType(uint64);
  registerDataType<uint64>(UInt64);
  registerDataType(std::string);
  registerDataType(bounding_box);
  registerDataType(dimension);
  registerDataType(bool);
  registerDataType(boost::shared_array<unsigned char>);
  registerDataType(boost::shared_array<float>);
  registerDataType(boost::shared_array<double>);
  registerDataType(state);

  registerDataType<cvc::geometry>("cvc::geometry");
  registerDataType<cvc::voxels>("cvc::voxels");
  registerDataType<cvc::volume>("cvc::volume");
  registerDataType<cvc::bounding_box>("cvc::bounding_box");
  registerDataType<cvc::dimension>("cvc::dimension");

  registerDataType<boost::shared_ptr<cvc::geometry>>("boost::shared_ptr<cvc::geometry>");
  registerDataType<boost::shared_ptr<cvc::voxels>>("boost::shared_ptr<cvc::voxels>");
  registerDataType<boost::shared_ptr<cvc::volume>>("boost::shared_ptr<cvc::volume>");
}

void app::registerDefaultHandlers() {
  register_rawiv_io(*this);
  register_rawv_io(*this);
  register_mrc_io(*this);
  register_vtk_io(*this);
  register_spider_io(*this);
  register_null_io(*this);
#ifdef CVC_USING_HDF5
  register_hdf5_io(*this);
#endif
  register_bunny_io(*this);
  register_off_io(*this);
  register_cvcraw_io(*this);
}

// ---------
// app::~app
// ---------
// Purpose:
//   Nothing may still be running against this app's mutexes and maps once
//   this returns.  Threads registered in _threads hold an app& (see
//   thread_feedback / thread_info, whose destructors call back into the app
//   as the worker winds down), so simply letting the map's shared_ptrs go
//   would detach any live thread onto freed memory.  Best effort, and
//   deliberately silent -- log() must not be called here (see app::log).
// ---- Change History ----
// 09/02/2026 -- Joe R. -- Creation; the body used to be empty, so a
//                         fixture-owned app could be destroyed out from
//                         under a running handler thread.
// 10/25/2026 -- Joe R. -- Close the background-task pool first, and sweep
//                         again for threads started while we were joining.
// 10/27/2026 -- Joe R. -- The pool is closed by closeThreadPool(), which also
//                         interrupts its persistent workers' tasks; they are
//                         joined once the map's threads are interrupted too,
//                         and the map is swept again after that join. A pool
//                         with no worker is closed before the async host's
//                         shutdown, which would wait for its queue for good.
//                         Interruption is held off until the last member is
//                         destroyed, not only through this body.
app::~app() {
  // Not interruptible: an interrupt pending on the thread destroying the app would otherwise be
  // thrown in a join below and dropped by its catch, and that thread left running into a freed
  // app. Nor while the members go, after this body: one of their destructors reaching an
  // interruption point (the state tree in _data joining a handler thread, a user object there or
  // in a slot that joins one) would have it thrown out of a destructor, which is std::terminate.
  // The hold is a member destroyed last (app.h), so the interrupt is still pending for the caller
  // once the app is gone.
  _teardownHold.emplace();

  // Async calls first: their kernels run on the background pool and post to the exec scheduler,
  // so cancel them and wait for their kernels while both are still here (the pool is still open).
  // A pool with no worker (none could be created) would never run a kernel still queued, and the
  // wait would be for good: it is closed first then, which drops those kernels and ends their
  // calls. (With no worker, it has no running task to interrupt either.)
  std::vector<thread_ptr> poolWorkers;
  bool poolClosed = false;
  if (_execAsyncHost) {
    bool noWorker = false;
    {
      boost::mutex::scoped_lock lock(_threadPoolMutex);
      noWorker = poolHasNoWorkerLocked(lock);
    }
    if (noWorker) {
      poolWorkers = closeThreadPool();
      poolClosed = true;
    }
    _execAsyncHost->shutdown();
  }

  // The background pool next: nothing queued starts any more (a pooled thread winding down would
  // otherwise start the next task after the snapshot below, and that task would run on into a
  // destroyed app), a task submitted from here on is dropped unrun, and running anonymous tasks
  // are interrupted. The dropped tasks are destroyed outside the pool's lock, in case their
  // captures call back in. The workers are joined below, once the threads in _threads have been
  // interrupted as well. (Keyed pool tasks have threads of their own in _threads.)
  if (!poolClosed)
    poolWorkers = closeThreadPool();

  // Take the map and clear it under the lock, then join without it: a worker
  // winding down takes _threadsMutex itself, so joining while holding it
  // deadlocks.  Repeat while joining turned up more -- a keyed pool thread
  // that had already taken its successor when we stopped the pool registers
  // that one after our snapshot, as may any app thread that starts another --
  // bounded, so a thread that keeps spawning cannot hold the app hostage.
  for (int sweep = 0; sweep < 8; ++sweep) {
    thread_map remaining;
    {
      boost::mutex::scoped_lock lock(_threadsMutex);
      remaining.swap(_threads);
    }

    // Interrupt everything first, then join -- many algorithms have
    // interruption checkpoints, so this is far quicker than serial joins.
    BOOST_FOREACH (thread_map::value_type &val, remaining) {
      if (!val.second)
        continue;
      try {
        val.second->interrupt();
      } catch (...) {
      }
    }

    // The pool's workers are joined only now, with the first sweep's threads interrupted beside
    // their tasks: a pool task waiting, uninterruptibly, on something one of those threads holds
    // gets it once that thread stops, instead of holding ~app for its 5 s and then outliving it.
    // A worker may register a thread while we join it (a keyed task it took before the pool
    // closed, or one its task starts), after this sweep's snapshot: the next sweep takes the map
    // again even when this one was empty.
    const bool joinedPool = !poolWorkers.empty();
    if (joinedPool) {
      joinPoolWorkers(poolWorkers);
      poolWorkers.clear();
    }
    if (remaining.empty()) {
      if (joinedPool)
        continue;
      break;
    }

    BOOST_FOREACH (thread_map::value_type &val, remaining) {
      if (!val.second)
        continue;
      try {
        if (val.second->joinable())
          val.second->try_join_for(boost::chrono::seconds(5));
      } catch (...) {
      }
    }
  }
}

data_map app::data() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataMutex);
  // Return a copy of the datamap.  Objects in the datamap
  // should be copyable without incuring much overhead, like
  // for instance objects that use a copy-on-write pattern for
  // large arrays internal to an object.
  return _data;
}

boost::any app::data(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataMutex);
  if (_data.find(key) != _data.end())
    return _data[key];
  return boost::any();
}

void app::data(const std::string &key, const boost::any &value) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_dataMutex);
    if (value.empty())
      _data.erase(key);    // remove if empty
    else if (!key.empty()) // don't allow empty keys
      _data[key] = value;
  }
  dataChanged(key);
}

void app::data(const data_map &map) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_dataMutex);
    _data = map;
    // erase empty data
    std::list<std::string> emptyProps;
    BOOST_FOREACH (data_map::value_type val, _data)
      if (val.first.empty() || val.second.empty())
        emptyProps.push_back(val.first);
    BOOST_FOREACH (std::string key, emptyProps)
      _data.erase(key);
  }
  dataChanged("all");
}

std::string app::dataTypeName(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataMutex);
  std::string retval;
  if (_data.find(key) != _data.end()) {
    std::string rawname = _data[key].type().name();
    if (_dataTypeNames.find(rawname) != _dataTypeNames.end())
      retval = _dataTypeNames[rawname];
    else
      retval = rawname;
  }

  return retval;
}

std::string app::dataTypeName(const boost::any &d) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataMutex);
  std::string retval;
  std::string rawname = d.type().name();
  if (_dataTypeNames.find(rawname) != _dataTypeNames.end())
    retval = _dataTypeNames[rawname];
  else
    retval = rawname;
  return retval;
}

data_type app::dataType(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataMutex);
  data_type retval = Undefined;
  if (_data.find(key) != _data.end()) {
    std::string rawname = _data[key].type().name();
    if (_dataTypeEnum.find(rawname) != _dataTypeEnum.end())
      retval = _dataTypeEnum[rawname];
  }
  return retval;
}

std::vector<std::string> app::listify(const std::string &keylist) {
  using namespace std;
  using namespace boost;
  using namespace boost::algorithm;
  string separators = properties("system.list_separators");
  if (separators.empty())
    separators = ",";
  vector<string> keys;
  split(keys, keylist, is_any_of(separators));
  BOOST_FOREACH (string &key, keys)
    trim(key);
  return keys;
}

std::string app::listify(const std::vector<std::string> &keys) {
  using namespace std;
  using namespace boost;
  using namespace boost::algorithm;
  string separators = properties("system.list_separators");
  if (separators.empty())
    separators = ",";
  separators.resize(1); // just use the first char
  vector<string> localKeys = keys;
  BOOST_FOREACH (string &key, localKeys)
    trim(key);
  string listkey = join(localKeys, separators);
  return listkey;
}

data_reader_collection app::dataReaders() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataReadersMutex);
  return _dataReaders;
}

void app::dataReaders(const data_reader_collection &dlc) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataReadersMutex);
  _dataReaders = dlc;
}

data_reader app::dataReader(data_reader_collection::size_type idx) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataReadersMutex);
  return _dataReaders[idx];
}

void app::dataReader(const data_reader &dl) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataReadersMutex);
  _dataReaders.push_back(dl);
}

bool app::readData(const std::string &path) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_dataReadersMutex);
  BOOST_FOREACH (data_reader dl, _dataReaders)
    if (dl(path))
      return true;
  return false;
}

property_map app::properties() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_propertiesMutex);
  return _properties;
}

std::string app::properties(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_propertiesMutex);
  if (_properties.find(key) != _properties.end())
    return _properties[key];
  return std::string();
}

void app::properties(const std::string &key, const std::string &val) {
  boost::this_thread::interruption_point();
  bool propChanged = true;
  {
    boost::mutex::scoped_lock lock(_propertiesMutex);
    if (val.empty()) // remove if empty
      _properties.erase(key);
    else if (_properties[key] == val) // if no change, no signal
      propChanged = false;
    else if (!key.empty()) // don't allow empty keys
      _properties[key] = val;
  }
  if (propChanged)
    propertiesChanged(key);
}

void app::properties(const property_map &map) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_propertiesMutex);
    _properties = map;
    // erase empty properties
    std::list<std::string> emptyProps;
    BOOST_FOREACH (property_map::value_type val, _properties)
      if (val.first.empty() || val.second.empty())
        emptyProps.push_back(val.first);
    BOOST_FOREACH (std::string key, emptyProps)
      _properties.erase(key);
  }
  propertiesChanged("all");
}

void app::addProperties(const property_map &map) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_propertiesMutex);
    BOOST_FOREACH (property_map::value_type val, map)
      if (!val.first.empty() && !val.second.empty())
        _properties[val.first] = val.second;
  }
  propertiesChanged("all");
}

bool app::hasProperty(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_propertiesMutex);
  if (_properties.find(key) != _properties.end())
    return true;
  return false;
}

std::vector<std::string> app::listProperty(const std::string &key, bool uniqueElements) {
  using namespace std;
  using namespace boost;
  using namespace boost::algorithm;

  vector<string> vals;

  // arand, 4-21-2011
  // fix for handling the case where the list is empty
  if (!hasProperty(key)) {
    return vals;
  }

  string valstring = properties(key);
  string separators = properties("system.list_separators");
  if (separators.empty())
    separators = ",";
  split(vals, valstring, is_any_of(separators));
  BOOST_FOREACH (string &val, vals)
    trim(val);
  if (uniqueElements) {
    set<string> vals_set;
    copy(vals.begin(), vals.end(), inserter(vals_set, vals_set.begin()));
    vals.resize(vals_set.size());
    copy(vals_set.begin(), vals_set.end(), vals.begin());
  }
  return vals;
}

// arand, 5-3-2011: two new functions for managing property lists below
void app::listPropertyAppend(const std::string &key, const std::string &val) {
  std::vector<std::string> listProp = listProperty(key, true);
  // string thumbGeoNames1;

  for (int i = 0; i < listProp.size(); i++) {
    if (val.compare(listProp[i]) == 0) {
      // its already in the list... just return
      return;
    }
  }
  listProp.push_back(val);
  properties(key, listify(listProp));
}

void app::listPropertyRemove(const std::string &key, const std::string &val) {
  std::vector<std::string> listProp = listProperty(key, true);
  // string thumbGeoNames1;

  bool found = false;
  for (int i = 0; i < listProp.size(); i++) {
    if (val.compare(listProp[i]) == 0) {
      listProp.erase(listProp.begin() + i);
      // only erase one copy of this value...
      found = true;
      break;
    }
  }
  if (found) {
    properties(key, listify(listProp));
  }
}

// 01/12/2014 -- transfix -- Creation.
// 01/17/2014 -- transfix -- using TIME_UTC_
void app::sleep(double ms) {
  thread_info ti(*this, BOOST_CURRENT_FUNCTION);
  boost::xtime xt;
  boost::xtime_get(&xt, boost::TIME_UTC_);
  xt.nsec += uint64(ms * std::pow(10.0, 6.0));
  boost::thread::sleep(xt);
}

// 12/16/2011 -- transfix -- Creation.
void app::propertyTreeTraverse(const boost::property_tree::ptree &pt,
                               const std::string &parentkey) {
  using namespace boost;
  using boost::property_tree::ptree;

  std::vector<std::string> filesToRead;

  BOOST_FOREACH (const ptree::value_type &v, pt) {
    std::string childkey = v.first;
    std::string fullkey = parentkey.empty() ? childkey : parentkey + "." + childkey;

    properties(fullkey, v.second.get_value<std::string>());
    propertyTreeTraverse(v.second, fullkey);
  }
}

// 12/16/2011 -- transfix -- Creation.
void app::readPropertyMap(const std::string &path) {
  using namespace boost;
  using boost::property_tree::ptree;
  ptree pt;
#ifndef CVC_APP_XML_PROPERTY_MAP
  read_info(path, pt);
#else
  read_xml(path, pt);
#endif
  propertyTreeTraverse(pt);
}

// 12/16/2011 -- transfix -- Creation.
void app::writePropertyMap(const std::string &path) {
  using boost::property_tree::ptree;
  ptree pt;
  property_map localmap = properties();
  BOOST_FOREACH (const property_map::value_type &v, localmap)
    pt.put(v.first, v.second);
#ifndef CVC_APP_XML_PROPERTY_MAP
  write_info(path, pt);
#else
  write_xml(path, pt);
#endif
}

thread_map app::threads() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_threadsMutex);
  return _threads;
}

thread_ptr app::threads(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_threadsMutex);
  if (_threads.find(key) != _threads.end())
    return _threads[key];
  return thread_ptr();
}

// 01/13/2014 -- Joe R. -- interrupting the currently running thread with this key
void app::threads(const std::string &key, const thread_ptr &val) {
  boost::this_thread::interruption_point();

  if (hasThread(key)) {
    thread_ptr tptr = threads(key);
    // Check if tptr is valid - it may have been removed by another thread
    // between hasThread() and threads() calls (race condition)
    if (tptr) {
      tptr->interrupt();
      // JOIN, not just interrupt. Replacing (or erasing) the entry below drops
      // the last shared_ptr to this boost::thread, and ~thread() DETACHES a
      // joinable thread rather than stopping it. The detached thread keeps
      // running while no longer appearing in _threads -- and a state_object
      // handler thread holds a pointer to its object, so ~state_object() can no
      // longer find it to join and the object is freed underneath it. That is a
      // use-after-free, and it surfaces later as heap corruption far from here.
      // interrupt() alone does not close the window: it only raises at the next
      // interruption point, which may be many instructions away or never.
      //
      // Never self-join -- a thread reassigning its own key would deadlock --
      // and bound the wait so a wedged handler degrades to the old behaviour
      // instead of hanging the process.
      if (tptr->joinable() && tptr->get_id() != boost::this_thread::get_id()) {
        if (!tptr->timed_join(boost::posix_time::milliseconds(5000)))
          log(5, str(boost::format("app::threads(): thread %s did not stop in 5s; "
                                   "replacing its entry while it still runs") %
                     key));
      }
    }
  }

  {
    boost::mutex::scoped_lock lock(_threadsMutex);
    if (!val)
      _threads.erase(key);
    else
      _threads[key] = val;
    updateThreadKeys();
  }

  threadsChanged(key);
}

// 01/13/2014 -- Joe R. -- interrupting all the threads in the map
void app::threads(const thread_map &map) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_threadsMutex);

    BOOST_FOREACH (thread_map::value_type t, _threads)
      if (t.second)
        t.second->interrupt();

    _threads = map;
    _threadProgress.clear();
    updateThreadKeys();
  }
  threadsChanged("all");
}

bool app::hasThread(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_threadsMutex);
  if (_threads.find(key) != _threads.end())
    return true;
  return false;
}

double app::threadProgress(const std::string &key) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_threadsMutex);

  // For keyed threads, check the persistent progress map first
  // This allows querying progress even after the thread has exited
  if (!key.empty()) {
    if (_threadProgressByKey.find(key) != _threadProgressByKey.end())
      return _threadProgressByKey[key];
  }

  // Fall back to thread ID lookup for current thread or legacy behavior
  boost::thread::id tid;
  if (key.empty())
    tid = boost::this_thread::get_id();
  else if (_threads.find(key) != _threads.end() && _threads[key])
    tid = _threads[key]->get_id();
  else
    return 0.0;

  return _threadProgress[tid];
}

void app::threadProgress(double progress) { threadProgress(std::string(), progress); }

void app::threadProgress(const std::string &key, double progress) {
  boost::this_thread::interruption_point();

  // clamp progress to [0.0,1.0]
  progress = progress < 0.0 ? 0.0 : progress > 1.0 ? 1.0 : progress;

  bool changed = false;
  {
    boost::mutex::scoped_lock lock(_threadsMutex);
    boost::thread::id tid;

    if (key.empty()) {
      tid = boost::this_thread::get_id();
      changed = true;
    } else if (_threads.find(key) != _threads.end() && _threads[key]) {
      tid = _threads[key]->get_id();
      changed = true;
      // Also store in the persistent key-based map
      _threadProgressByKey[key] = progress;
    }

    if (changed)
      _threadProgress[tid] = progress;
  }

  if (changed)
    threadsChanged(key);
}

void app::finishThreadProgress(const std::string &key) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_threadsMutex);

    boost::thread::id tid;
    std::string threadKey = key;

    if (key.empty()) {
      // No key provided - use current thread ID to find the key
      tid = boost::this_thread::get_id();

      // Look up the key from thread ID
      if (_threadKeys.find(tid) != _threadKeys.end())
        threadKey = _threadKeys[tid];
    } else if (_threads.find(key) != _threads.end() && _threads[key]) {
      tid = _threads[key]->get_id();
    } else {
      // Thread not found - nothing to clean up
      return;
    }

    // Store 100% in the persistent key-based map
    if (!threadKey.empty())
      _threadProgressByKey[threadKey] = 1.0;

    // NOTE: do *not* erase _threads[threadKey] here.  This runs from
    // ~thread_feedback, i.e. from inside the worker, which has not exited
    // yet -- its own ~thread_info and boost's thread epilogue still come
    // after us, and both dereference this app.  Erasing drops the last
    // shared_ptr to a live boost::thread, and ~boost::thread() detaches it,
    // after which nothing can ever join it.  Every consumer that discovers
    // work by scanning _threads (app::wait_for_threads(),
    // state_object::waitForHandlers(), ~state_object()) would then skip the
    // thread and let the app die underneath it.  Reaping belongs to the
    // thread's owner: startThread() joins and replaces a finished same-key
    // entry, ~state_object() removes the handler keys it has joined, and
    // ~app() sweeps whatever is left.
    // 09/02/2026 -- Joe R. -- was erasing a still-running thread here; root
    //                         cause of the intermittent SEGFAULT in
    //                         StateObjectFixture.StateObjectMixedInstanceThreading.

    // Clean up by thread ID.  These are keyed by thread::id and get rebuilt
    // from _threads by updateThreadKeys(), so they can go now.
    _threadKeys.erase(tid);
    _threadProgress.erase(tid);
    _threadInfo.erase(tid);
  }
  threadsChanged(key);
}

std::string app::threadKey() {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_threadsMutex);
    if (_threadKeys.find(boost::this_thread::get_id()) != _threadKeys.end())
      return _threadKeys[boost::this_thread::get_id()];
    else
      return std::string("unknown");
  }
}

void app::removeThread(const std::string &key) { threads(key, thread_ptr()); }

// The first of hint, hint0, hint1, ... that names no tracked thread.  (Queued
// pooled jobs need no key held for them: an anonymous one has no key, and a
// keyed one runs under its caller's.)
// 10/25/2026 -- Joe R. -- One lock for the whole search.
std::string app::uniqueThreadKey(const std::string &hint) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_threadsMutex);
  const std::string h = hint.empty() ? "thread" : hint;
  std::string key = h;
  unsigned int i = 0;
  while (_threads.count(key))
    key = h + boost::lexical_cast<std::string>(i++);
  return key;
}

void app::threadInfo(const std::string &key, const std::string &infostr) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_threadsMutex);

    boost::thread::id tid;
    if (key.empty())
      tid = boost::this_thread::get_id();
    else if (_threads.find(key) != _threads.end() && _threads[key]) {
      tid = _threads[key]->get_id();
      // Also store in persistent map by key
      _threadInfoByKey[key] = infostr;
    } else {
      // Thread may have already exited, just update persistent map
      if (!key.empty())
        _threadInfoByKey[key] = infostr;
      return;
    }

    _threadInfo[tid] = infostr;
  }
  threadsChanged(key);
}

std::string app::threadInfo(const std::string &key) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_threadsMutex);

    // For keyed threads, check the persistent info map first
    // This allows querying status even after the thread has exited
    if (!key.empty()) {
      if (_threadInfoByKey.find(key) != _threadInfoByKey.end())
        return _threadInfoByKey[key];
    }

    // Fall back to thread ID lookup for current thread or legacy behavior
    boost::thread::id tid;
    if (key.empty())
      tid = boost::this_thread::get_id();
    else if (_threads.find(key) != _threads.end() && _threads[key])
      tid = _threads[key]->get_id();
    else
      return std::string();

    return _threadInfo[tid];
  }
}

// TODO: write a version of log that returns a stream to use with
// stream operators.
//  09/09/2011 -- Joe R. -- Removing references to cvcapp because it will crash
//                          if you try to use them in ~App.
void app::log(unsigned int verbosity_level, const std::string &buf, bool append_newline) {
#ifdef USING_LOG4CPLUS_DEFAULT
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("cvc.app.log");
  std::string msg = buf;
  // Remove trailing newline if present and append_newline is false
  if (!append_newline && !msg.empty() && msg[msg.length() - 1] == '\n') {
    msg = msg.substr(0, msg.length() - 1);
  }
  if (verbosity_level == 0) { // || verbosity_level == 1) {
    LOG4CPLUS_ERROR(logger, msg);
  } else if (verbosity_level == 1) {
    LOG4CPLUS_WARN(logger, msg);
  } else if (verbosity_level == 2) {
    LOG4CPLUS_INFO(logger, msg);
  } else if (verbosity_level == 3) {
    LOG4CPLUS_DEBUG(logger, msg);
  } else {
    LOG4CPLUS_TRACE(logger, msg);
  }
#else
  using namespace std;
  using namespace boost;
  using namespace boost::algorithm;
  boost::mutex::scoped_lock lock(_logMutex);
  unsigned int log_verbosity =
      hasProperty("system.log_verbosity") ? properties<unsigned int>("system.log_verbosity") : 6;
  if (verbosity_level < log_verbosity) {
    string output_locs = properties("system.log_output");
    string log_prefix = properties("system.log_prefix");
    string log_postfix = properties("system.log_postfix");
    string output_string = log_prefix + buf + log_postfix;
    if (append_newline &&
        (output_string.empty() || output_string[output_string.length() - 1] != '\n')) {
      output_string += "\n";
    }

    vector<string> key_idents;
    split(key_idents, output_locs, is_any_of(","));
    BOOST_FOREACH (string loc, key_idents) {
      trim(loc);
      if (loc == "stdout") {
        cout << output_string;
        continue;
      } else if (loc == "stderr" || loc.empty()) {
        cerr << output_string;
        continue;
      }
      // check if loc is an std::string on the datamap
      bool isDataKey = false;
      std::vector<std::string> dataKeys = data<std::string>();
      BOOST_FOREACH (std::string key, dataKeys)
        if (loc == key) // if so, append buf to the log string
        {
          data(key, data<std::string>(key) + output_string);
          isDataKey = true;
          break;
        }

      // if it was a data key, we're done
      if (isDataKey)
        continue;

      // if nothing else, interpret as a filename
      ofstream outfile(loc.c_str(), ios_base::out | ios_base::app);
      outfile << output_string;
    }
  }
#endif
}

// This should only be called after we lock the threads mutex
//  07/29/2011 - Joe R. - handling thread info here too.
void app::updateThreadKeys() {
  using namespace std;

  _threadKeys.clear();

  set<boost::thread::id> infoIds;
  BOOST_FOREACH (thread_info_map::value_type val, _threadInfo)
    infoIds.insert(val.first);

  set<boost::thread::id> currentIds;
  BOOST_FOREACH (thread_map::value_type val, _threads) {
    thread_ptr ptr = val.second;
    if (ptr) {
      _threadKeys[ptr->get_id()] = val.first;

      // set the thread info to a default state if not already set
      if (_threadInfo[ptr->get_id()].empty())
        _threadInfo[ptr->get_id()] = "running";

      // Inside the null check: this used to sit outside it and would
      // dereference a null thread_ptr if one ever reached _threads --
      // threads(const thread_map&) copies its argument wholesale.
      currentIds.insert(ptr->get_id());
    }
  }

  // compute thread ids that need to be removed from the threadInfo map
  set<boost::thread::id> infoIdsToRemove;
  set_difference(infoIds.begin(), infoIds.end(), currentIds.begin(), currentIds.end(),
                 inserter(infoIdsToRemove, infoIdsToRemove.begin()));

  BOOST_FOREACH (boost::thread::id tid, infoIdsToRemove)
    _threadInfo.erase(tid);
}

mutex_ptr app::mutex(const std::string &name) {
  bool changed = false;
  mutex_ptr ptr;
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_mutexMapMutex);
    if (!_mutexMap[name].get<0>()) {
      _mutexMap[name].get<0>().reset(new boost::mutex);
      changed = true;
    }
    ptr = _mutexMap[name].get<0>();
  }

  if (changed)
    mutexesChanged(name);

  return ptr;
}

void app::mutexInfo(const std::string &name, const std::string &in) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_mutexMapMutex);
    _mutexMap[name].get<1>() = in;
  }
  mutexesChanged(name);
}

std::string app::mutexInfo(const std::string &name) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutexMapMutex);
  return _mutexMap[name].get<1>();
}

// -------------------------
// Thread Pool Implementation
// -------------------------
// 12/25/2025 -- Joe R. -- Creation.

cvc::thread_pool &app::computePool() {
  boost::mutex::scoped_lock lock(_computePoolMutex);
  if (!_computePool)
    _computePool.reset(new cvc::thread_pool()); // hardware_concurrency()-1 workers
  return *_computePool;
}

namespace {

// app::poolDropReason() on this thread: set by a pool_drop_scope while the pool destroys tasks
// unrun here, null otherwise.
thread_local const char *t_poolDropReason = nullptr;

// Holds this thread's drop reason while the pool destroys tasks unrun, and puts back the one
// before (a dropped task's destructor may lead to other drops: the call it ends, a callback).
class pool_drop_scope {
  const char *_previous;

public:
  explicit pool_drop_scope(const char *why) noexcept : _previous(t_poolDropReason) {
    t_poolDropReason = why;
  }
  ~pool_drop_scope() { t_poolDropReason = _previous; }
  pool_drop_scope(const pool_drop_scope &) = delete;
  pool_drop_scope &operator=(const pool_drop_scope &) = delete;
};

} // namespace

const char *app::poolDropReason() noexcept {
  return t_poolDropReason ? t_poolDropReason : kPoolStopped;
}

namespace {

// compute_async's pool task: the std::function's target itself, holding the job by value, so it
// costs one heap block on every standard library (a shared_ptr capture costs a block of its own
// under libstdc++, which keeps only a trivially copyable callable in std::function's buffer). If
// the pool destroys it unrun (it was submitted, or still queued, once ~app had stopped the pool;
// or it was left queued with no worker), on_error hears of it, with app::poolDropReason().
struct compute_job {
  app &owner;
  int n;
  std::function<void(int)> fn;
  std::function<void()> on_done;
  std::function<void(const std::string &)> on_error;
  bool armed = true; // still owes the job its run or its drop (a move or a copy takes that over)

  compute_job(app &a, int count, std::function<void(int)> f, std::function<void()> done,
              std::function<void(const std::string &)> err)
      : owner(a), n(count), fn(std::move(f)), on_done(std::move(done)), on_error(std::move(err)) {}
  compute_job(compute_job &&o) noexcept
      : owner(o.owner), n(o.n), fn(std::move(o.fn)), on_done(std::move(o.on_done)),
        on_error(std::move(o.on_error)), armed(std::exchange(o.armed, false)) {}
  // std::function needs a copy constructor. The pool never copies a task (it moves or swaps the
  // std::function, which hands over a target this large by pointer); a copy takes the job over as
  // a move does, so it runs, or reports its drop, once.
  compute_job(const compute_job &o) noexcept
      : compute_job(std::move(const_cast<compute_job &>(o))) {}
  compute_job &operator=(const compute_job &) = delete;
  compute_job &operator=(compute_job &&) = delete;

  ~compute_job() {
    if (!armed || !on_error)
      return;
    // Not interruptible, as thread_info and thread_feedback keep their books: this runs on the
    // thread that submitted the job, the one destroying the app or the one whose worker creation
    // failed, and an interrupt pending there (a keyed thread ~app has interrupted, say) must stay
    // pending, not be thrown inside on_error and dropped by the catch below.
    boost::this_thread::disable_interruption keep_interrupt_pending;
    try {
      on_error(app::poolDropReason());
    } catch (...) {
    }
  }

  void operator()() {
    if (!std::exchange(armed, false))
      return;
    // Without on_error, a throw goes to the pool (which logs it) and on_done is skipped.
    try {
      owner.computePool().parallel_for(n, fn);
    } catch (boost::thread_interrupted &) {
      if (on_error)
        on_error("interrupted");
      throw;
    } catch (const std::exception &e) {
      if (!on_error)
        throw;
      on_error(e.what());
      return;
    } catch (...) {
      if (!on_error)
        throw;
      on_error("unknown error");
      return;
    }
    if (on_done)
      on_done();
  }
};

} // namespace

void app::compute_async(int n, std::function<void(int)> fn, std::function<void()> on_done,
                        thread_priority priority) {
  compute_async(n, std::move(fn), std::move(on_done), nullptr, priority);
}

void app::compute_async(int n, std::function<void(int)> fn, std::function<void()> on_done,
                        std::function<void(const std::string &)> on_error,
                        thread_priority priority) {
  // A background pool worker owns the blocking parallel_for; the caller returns immediately. The
  // job holds fn/on_done by value, so a result accumulator they close over outlives this call.
  // Only a job with on_error hears of a drop: one without waits in the queue, should the pool be
  // left with no worker, rather than be dropped without a word (its caller may wait for on_done:
  // a threaded tf overlay task did).
  const bool reports_drop = static_cast<bool>(on_error);
  std::function<void()> job(
      compute_job(*this, n, std::move(fn), std::move(on_done), std::move(on_error)));
  // Queued, or dropped (the pool has stopped: on_error, if any, has heard of it).
  if (submitPooledHandingBack(job, priority, /*dropIfNoWorker=*/reports_drop) !=
      PoolSubmit::no_worker)
    return;
  // No worker could be started for it, and the pool has none: as when compute_async started a
  // thread per job and that failed, the caller hears of it by an exception (a threaded tf
  // overlay task then carries on cooperatively) rather than the job waiting for a later
  // submission to start a worker. The callbacks are released without being called, with
  // interruption held off, as the pool destroys every task.
  boost::this_thread::disable_interruption keep_interrupt_pending;
  if (compute_job *handed_back = job.target<compute_job>())
    handed_back->armed = false;
  job = nullptr;
  boost::throw_exception(boost::thread_resource_error(
      static_cast<int>(boost::system::errc::resource_unavailable_try_again),
      "cvc::app::compute_async: no background worker could be started"));
}

cvc::world_clock &app::world_clock() {
  boost::mutex::scoped_lock lock(_worldBasesMutex);
  if (!_worldClock)
    _worldClock.reset(new cvc::world_clock()); // default 120 Hz, live mode
  return *_worldClock;
}

cvc::world_units &app::world_units() {
  boost::mutex::scoped_lock lock(_worldBasesMutex);
  if (!_worldUnits)
    _worldUnits.reset(new cvc::world_units()); // default SI, 1 metre per world unit
  return *_worldUnits;
}

cvc::state_exec::async_scheduler &app::exec_scheduler() {
  boost::mutex::scoped_lock lock(_execSchedulerMutex);
  if (!_execScheduler) {
    _execScheduler.reset(new cvc::state_exec::async_scheduler());
    // Wire it to this app's state root so state-watches (state-watch/on_change:)
    // resolve against the app tree. Processes are chrooted per-owner on submit.
    _execScheduler->set_watch_root(&root());
    // Read submitted scripts inside their processes, like pycvc's Exec (see host_parser_defaults
    // for why 1024 tokens a step). state_exec.defaults.parser.* on this tree overrides it.
    _execScheduler->set_parser_defaults(cvc::state_exec::async_scheduler::host_parser_defaults());
    // Deep-data work a slice per step too (host_data_defaults); state_exec.defaults.data.*
    // overrides it.
    _execScheduler->set_data_defaults(cvc::state_exec::async_scheduler::host_data_defaults());
  }
  return *_execScheduler;
}

cvc::state_exec::async_host &app::exec_async_host() {
  cvc::state_exec::async_scheduler &sched = exec_scheduler();
  boost::mutex::scoped_lock lock(_execSchedulerMutex);
  if (!_execAsyncHost)
    _execAsyncHost.reset(new cvc::state_exec::async_host(sched, this));
  return *_execAsyncHost;
}

void app::setThreadPoolSize(unsigned int size) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    _maxPoolSize = (size > 0) ? size : 1;
    // A larger pool may start queued tasks now: the worker that takes one wakes the next.
    tryStartWorker();
  }
  startRequestedWorkers();
}

unsigned int app::getThreadPoolSize() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_threadPoolMutex));
  return _maxPoolSize;
}

unsigned int app::getActiveThreadCount() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_threadPoolMutex));
  return _activeWorkers;
}

unsigned int app::getPendingThreadCount() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_threadPoolMutex));
  return static_cast<unsigned int>(_pendingTasks.size());
}

// ---------------------------------------------------------------------------
// The background task pool (startThreadPooled / submitPooled)
// ---------------------------------------------------------------------------
// One queue, ordered by priority and then by submission (FIFO within a priority), and at most
// _maxPoolSize tasks running at once (_activeWorkers). Anonymous tasks (submitPooled,
// startThreadPooled with wait=false) run on persistent worker threads (_poolWorkers, started on
// demand up to the pool size and kept until ~app). A keyed task (startThreadPooled with
// wait=true) still gets a thread of its own, registered under its key in threads() before it
// runs, which counts itself out of the pool when it ends.
//
// Locking: _threadPoolMutex guards the queue, the counts, the workers and the stop flag. It is
// never held while a task runs, while a thread is registered in threads() (_threadsMutex), or
// while anything logs, and it is only ever released by the RAII lock that took it. Nothing that
// can throw boost::thread_interrupted runs with interruption enabled while the pool's
// bookkeeping is half done: workers and registrations disable interruption, and only a task
// itself is interruptible. Workers are interrupted only under the mutex (see poolWorkerLoop),
// and are woken (a std::condition_variable_any) and created after it is released; a new worker
// takes no task until its creator has put it in _poolWorkers, where the interrupts go. Once ~app
// has stopped the pool, a task is dropped unrun: submitPooled returns false, and the queued
// ones are destroyed in closeThreadPool. A trySubmitPooled task left queued with no worker,
// running or being created, is dropped as well (takeUnstartableLocked). A dropped task reports
// the drop from its destructor, with poolDropReason() saying why.
//
// 10/26/2026 -- Joe R. -- Persistent workers instead of a boost::thread per task; FIFO within a
//                         priority; anonymous tasks out of the thread map. Fixes the stall and
//                         hang past the pool size (a double unlock of _threadPoolMutex after an
//                         interruption_point() throw in tryStartWorker).
// 10/27/2026 -- Joe R. -- A burst starts a worker per task up to the pool size at once; a
//                         keyed task with room starts on the caller even behind queued anonymous
//                         ones; interrupts ordered by the pool mutex; SRW-backed worker wakes.
// 10/27/2026 -- Joe R. -- A new worker waits at a gate until it is registered, so an interrupt
//                         reaches the first task it takes; submitPooled reports a drop.
// 10/27/2026 -- Joe R. -- The hand-back to compute_async waits for the worker creations under
//                         way: the job may rely on another call's, which can still fail.
// 10/27/2026 -- Joe R. -- startThreadPooled(wait=true) throws when its task's own thread can't
//                         be created; a keyed task started later and failing so is logged.
// 10/27/2026 -- Joe R. -- That drop is logged before its slot is freed: an idle pool has logged it.
// 10/27/2026 -- Joe R. -- A trySubmitPooled task left queued with no worker (the creation asked
//                         for as a keyed thread ends fails) is dropped, with kPoolNoWorker as
//                         poolDropReason(), instead of waiting for a later submission.
// 10/27/2026 -- Joe R. -- Not compute_async's job without on_error, which can't report it: it
//                         waits.

bool app::poolCanStartLocked() const {
  return !_poolStopping && !_pendingTasks.empty() && _activeWorkers < _maxPoolSize;
}

app::ThreadPoolTask app::takePoolTaskLocked() {
  // Swap the parts out of the top rather than moving: a moved-from std::function may keep its
  // callable (libc++ copies a small one), and this one must hold nothing once it is popped.
  ThreadPoolTask &top = const_cast<ThreadPoolTask &>(_pendingTasks.top());
  ThreadPoolTask task;
  task.key.swap(top.key);
  task.task.swap(top.task);
  task.priority = top.priority;
  task.keyed = top.keyed;
  task.seq = top.seq;
  _pendingTasks.pop(); // reorders by priority and seq, which the swap left alone
  ++_activeWorkers;
  return task;
}

void app::notifyPoolIdleLocked() {
  if (_activeWorkers == 0 && _pendingTasks.empty())
    _threadPoolIdle.notify_all();
}

bool app::submitPooled(std::function<void()> task, thread_priority priority) {
  if (!task)
    return true;
  ThreadPoolTask t;
  t.task.swap(task); // nothing of the callable stays in `task`
  t.priority = priority;
  if (enqueuePoolTask(t) == PoolSubmit::queued)
    return true;
  // Refused: `t` is destroyed unrun, here, with interruption held off, so that an interrupt
  // pending on this thread isn't thrown inside a destructor reporting the drop (and dropped).
  boost::this_thread::disable_interruption keep_interrupt_pending;
  pool_drop_scope reason(kPoolStopped);
  t.task = nullptr;
  return false;
}

app::PoolSubmit app::trySubmitPooled(std::function<void()> &task, thread_priority priority) {
  return submitPooledHandingBack(task, priority, /*dropIfNoWorker=*/true);
}

app::PoolSubmit app::submitPooledHandingBack(std::function<void()> &task,
                                             thread_priority priority, bool dropIfNoWorker) {
  if (!task)
    return PoolSubmit::queued;
  ThreadPoolTask t;
  t.task.swap(task); // nothing of the callable stays in `task`
  t.priority = priority;
  t.dropIfNoWorker = dropIfNoWorker;
  const PoolSubmit submitted = enqueuePoolTask(t, /*handBackIfNoWorker=*/true);
  if (submitted == PoolSubmit::no_worker) {
    t.task.swap(task); // handed back, unrun
  } else if (submitted == PoolSubmit::stopped) {
    // Destroyed unrun here, with interruption held off, as submitPooled does.
    boost::this_thread::disable_interruption keep_interrupt_pending;
    pool_drop_scope reason(kPoolStopped);
    t.task = nullptr;
  }
  return submitted;
}

bool app::PoolQueue::take(std::uint64_t seq, ThreadPoolTask &out) {
  const auto it =
      std::find_if(c.begin(), c.end(), [seq](const ThreadPoolTask &t) { return t.seq == seq; });
  if (it == c.end())
    return false;
  // Swapped out, as takePoolTaskLocked does: nothing of the task stays behind.
  out.key.swap(it->key);
  out.task.swap(it->task);
  out.priority = it->priority;
  out.keyed = it->keyed;
  out.seq = it->seq;
  c.erase(it);
  std::make_heap(c.begin(), c.end(), comp);
  return true;
}

void app::PoolQueue::takeDroppable(std::vector<std::function<void()>> &out) {
  const auto droppable = [](const ThreadPoolTask &t) { return t.dropIfNoWorker; };
  const std::size_t n = std::count_if(c.begin(), c.end(), droppable);
  if (n == 0)
    return;
  out.reserve(out.size() + n); // the one step that can throw: nothing is taken yet
  for (ThreadPoolTask &t : c)
    if (t.dropIfNoWorker) {
      out.emplace_back();
      out.back().swap(t.task); // swapped out, as takePoolTaskLocked does
    }
  c.erase(std::remove_if(c.begin(), c.end(), droppable), c.end());
  std::make_heap(c.begin(), c.end(), comp);
}

app::PoolSubmit app::enqueuePoolTask(ThreadPoolTask &task, bool handBackIfNoWorker) {
  ThreadPoolTask keyed;
  PoolKick kick;
  std::uint64_t seq = 0;
  bool reliesOnCreations = false;
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    if (_poolStopping)
      return PoolSubmit::stopped; // the app is going away: the caller drops the task
    ThreadPoolTask queued;
    queued.key.swap(task.key);
    queued.task.swap(task.task);
    queued.priority = task.priority;
    queued.keyed = task.keyed;
    queued.dropIfNoWorker = task.dropIfNoWorker && !task.keyed;
    queued.seq = seq = _poolSeq++;
    if (queued.keyed && _activeWorkers + _pendingTasks.size() < _maxPoolSize) {
      // Room for it and for every task already queued, so it delays none of them: it starts
      // here, and its thread is in threads() under its key when startThreadPooled returns.
      keyed = std::move(queued);
      ++_activeWorkers;
    } else {
      _pendingTasks.push(std::move(queued));
      // A keyed task at the head of a queue that may start does the same.
      if (poolCanStartLocked() && _pendingTasks.top().keyed)
        keyed = takePoolTaskLocked();
    }
    kick = poolKickLocked();
    // No worker runs yet, only creations are under way (this call's, or another's): the task has
    // a worker only if one of them succeeds. This call decides what becomes of it then (below),
    // and no other drops it meanwhile (takeUnstartableLocked).
    reliesOnCreations = handBackIfNoWorker && !task.keyed && _creatingWorkers > 0 &&
                        _poolThreadCount == _creatingWorkers;
    if (reliesOnCreations)
      ++_handBackChecks;
  }
  applyPoolKick(kick);
  if (keyed.keyed) {
    // The caller's own keyed task (startThreadPooled with room for it): if its thread can't be
    // created, the caller hears of it, as when every pooled task had a thread of its own. One
    // taken off the head of the queue for another task has no caller here to tell: it is logged.
    const bool own = keyed.seq == seq;
    const std::string key = own ? keyed.key : std::string();
    if (!startKeyedPoolThread(keyed, /*logDrop=*/!own) && own)
      boost::throw_exception(boost::thread_resource_error(
          static_cast<int>(boost::system::errc::resource_unavailable_try_again),
          "cvc::app::startThreadPooled: no thread could be started for '" + key + "'"));
  }
  if (!reliesOnCreations)
    return PoolSubmit::queued;
  // If every one of those creations failed, nothing will take the task until a later call
  // manages to start a worker: hand it back instead. It is still queued then, since only a
  // worker in _poolWorkers takes an anonymous task and there is none. The other dropIfNoWorker
  // tasks queued go too (their submitters have returned): a creation that failed while this
  // call was deciding left them to it.
  PoolSubmit submitted = PoolSubmit::queued;
  std::vector<std::function<void()>> unstartable;
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    bool noWorker = false;
    try {
      noWorker = poolHasNoWorkerLocked(lock);
    } catch (...) {
      --_handBackChecks;
      throw;
    }
    --_handBackChecks;
    if (noWorker) {
      if (_pendingTasks.take(seq, task))
        submitted = PoolSubmit::no_worker;
      takeUnstartableLocked(unstartable);
      notifyPoolIdleLocked();
    }
  }
  dropPoolTasks(unstartable, kPoolNoWorker);
  return submitted;
}

bool app::poolHasNoWorkerLocked(boost::mutex::scoped_lock &lock) {
  // A worker, once created, runs until the pool stops, so the pool has one for good as soon as one
  // runs; a creation under way may still fail. While none runs, wait for those to succeed or fail
  // (bounded: a creation takes milliseconds at most), then look.
  _poolWorkerAdmitted.wait_for(lock, std::chrono::seconds(5), [this] {
    return _poolStopping || _creatingWorkers == 0 || _poolThreadCount > _creatingWorkers;
  });
  return !_poolStopping && _poolThreadCount == 0;
}

void app::takeUnstartableLocked(std::vector<std::function<void()>> &out) {
  // With no worker running or being created, a queued anonymous task waits for a later submission
  // (or setThreadPoolSize) to start one, which may never come. That is so for a submitPooled
  // task (and a compute_async job with no on_error); a trySubmitPooled one's submitter must hear
  // of it, so the task is dropped, as it would have been handed back had the creation failed
  // while its submitter waited for it. Not while a trySubmitPooled call still decides about its
  // own task: that call takes the rest once it has.
  if (_poolStopping || _poolThreadCount != 0 || _creatingWorkers != 0 || _handBackChecks != 0)
    return;
  try {
    _pendingTasks.takeDroppable(out);
  } catch (...) {
    return; // out of memory: they stay queued
  }
  notifyPoolIdleLocked();
}

void app::dropPoolTasks(std::vector<std::function<void()>> &tasks, const char *why) {
  if (tasks.empty())
    return;
  // Each reports its drop from its destructor, with `why` (poolDropReason()). Not interruptible:
  // an interrupt pending on this thread (a keyed thread's, say) stays pending instead of being
  // thrown in a task's destructor and dropped by its catch.
  boost::this_thread::disable_interruption keep_interrupt_pending;
  pool_drop_scope reason(why);
  for (std::function<void()> &task : tasks)
    task = nullptr;
  tasks.clear();
}

app::PoolKick app::poolKickLocked() {
  // _threadPoolMutex is held. Every queued task that may start now gets a worker on its way:
  // one already signalled or starting, else an idle one to wake, else (below the pool size) a
  // new one, which applyPoolKick() creates once the mutex is released (creating a thread can
  // take milliseconds on a loaded machine, and nobody may wait for the pool that long). A burst
  // of N tasks so gets min(N, pool size) workers at once, and each worker is started once per
  // app. Workers between two tasks aren't counted: they take a queued task before they wait, so
  // at worst a worker is started early.
  PoolKick kick;
  if (!poolCanStartLocked())
    return kick;
  const std::size_t startable =
      std::min<std::size_t>(_pendingTasks.size(), _maxPoolSize - _activeWorkers);
  std::size_t coming = std::size_t(_signalledWorkers) + _startingWorkers;
  for (; coming < startable && _idleWorkers > 0; ++coming) {
    --_idleWorkers;
    ++_signalledWorkers;
    ++kick.wakes;
  }
  for (; coming < startable && _poolThreadCount < _maxPoolSize; ++coming) {
    ++_poolThreadCount;
    ++_startingWorkers;
    ++_creatingWorkers;
    ++kick.spawns;
  }
  return kick;
}

bool app::applyPoolKick(const PoolKick &kick) {
  // _threadPoolMutex is NOT held. A worker counted as idle is in (or entering) its wait, which
  // releases _threadPoolMutex only once it can be signalled, so a signal sent now reaches it.
  for (unsigned int i = 0; i < kick.wakes; ++i)
    _poolWorkerWake.notify_one();
  bool started = true;
  for (unsigned int i = 0; i < kick.spawns; ++i) {
    // The new worker waits at its gate until it is in _poolWorkers: wait_for_threads and ~app
    // interrupt only the workers there, so one that took a task before it was would run that
    // task on uninterrupted (and ~app could go while it still runs).
    std::shared_ptr<PoolWorkerGate> gate;
    thread_ptr worker;
    bool created = true;
    try {
      if (_poolWorkerSpawnHook)
        _poolWorkerSpawnHook(); // a test making the creation fail (app.h)
      gate = std::make_shared<PoolWorkerGate>();
      worker.reset(new boost::thread(&app::poolWorkerLoop, this, gate));
    } catch (...) {
      created = false;
    }
    if (!created) {
      // No new thread (resources): the queued tasks wait for a running worker, or for the next
      // submission to try again. With no worker left, running or being created, the
      // trySubmitPooled tasks are dropped instead, here (takeUnstartableLocked): a submitter still
      // waiting for this creation sees it fail (poolHasNoWorkerLocked) and hands its task back,
      // and one that returned hears of it from the task's drop.
      std::vector<std::function<void()>> unstartable;
      {
        boost::mutex::scoped_lock lock(_threadPoolMutex);
        const unsigned int left = kick.spawns - i;
        _startingWorkers -= std::min(_startingWorkers, left);
        _creatingWorkers -= std::min(_creatingWorkers, left);
        _poolThreadCount -= std::min(_poolThreadCount, left);
        takeUnstartableLocked(unstartable);
      }
      _poolWorkerAdmitted.notify_all(); // poolHasNoWorkerLocked
      dropPoolTasks(unstartable, kPoolNoWorker);
      return false;
    }
    if (_poolWorkerCreatedHook) {
      try {
        _poolWorkerCreatedHook(); // a test holding the creator here (app.h)
      } catch (...) {
      }
    }
    bool registered = false;
    {
      boost::mutex::scoped_lock lock(_threadPoolMutex);
      if (_creatingWorkers > 0)
        --_creatingWorkers; // this creation is over (the notify below tells poolHasNoWorkerLocked)
      if (!_poolStopping) {
        try {
          _poolWorkers.push_back(worker); // ~app joins it
          registered = true;
        } catch (...) {
        }
      }
      gate->registered = registered;
      gate->open = true;
    }
    _poolWorkerAdmitted.notify_all();
    if (!registered) {
      // ~app has taken the workers it joins already (or this one couldn't be recorded): it ends
      // at its gate without taking a task. Interrupted and joined with a bound all the same, and
      // without taking an interrupt meant for this thread.
      started = false;
      {
        boost::this_thread::disable_interruption uninterruptible;
        try {
          worker->interrupt();
          worker->try_join_for(boost::chrono::seconds(5));
        } catch (...) {
        }
      }
      // Not recorded with the pool running on, it may have been the pool's one worker (it counted
      // itself out as it ended): as when its creation fails.
      std::vector<std::function<void()>> unstartable;
      {
        boost::mutex::scoped_lock lock(_threadPoolMutex);
        takeUnstartableLocked(unstartable);
      }
      dropPoolTasks(unstartable, kPoolNoWorker);
    }
  }
  return started;
}

void app::tryStartWorker() {
  // _threadPoolMutex is held by the caller.
  const PoolKick kick = poolKickLocked();
  for (unsigned int i = 0; i < kick.wakes; ++i)
    _poolWorkerWake.notify_one();
  _spawnRequests += kick.spawns;
}

void app::startRequestedWorkers() {
  // _threadPoolMutex is NOT held: start the workers tryStartWorker asked for.
  PoolKick kick;
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    kick.spawns = _spawnRequests;
    _spawnRequests = 0;
  }
  applyPoolKick(kick);
}

void app::poolWorkerLoop(std::shared_ptr<PoolWorkerGate> gate) {
  // Only a task may be interrupted (wait_for_threads, ~app). Idle and between tasks the worker
  // ignores interruption. Interrupts are sent under _threadPoolMutex, to the workers in
  // _poolWorkers, and bump _poolInterruptGen: one sent before the worker takes its next task is
  // cleared, under the mutex, before it takes it; one sent after is that task's, and it stays
  // pending until the task reaches an interruption point.
  boost::this_thread::disable_interruption uninterruptible;
  const auto clearStaleInterrupt = [&uninterruptible] {
    boost::this_thread::restore_interruption interruptible(uninterruptible);
    try {
      boost::this_thread::interruption_point(); // a request sent before the take: not the task's
    } catch (boost::thread_interrupted &) {
    }
  };
  boost::unique_lock<boost::mutex> lock(_threadPoolMutex);
  // Not a task before it is in _poolWorkers, where the interrupts go (applyPoolKick).
  while (!gate->open)
    _poolWorkerAdmitted.wait(lock);
  if (_startingWorkers > 0)
    --_startingWorkers;
  if (!gate->registered) {
    // Not kept (the pool is stopping, or it couldn't be recorded): it ends here, and the next
    // kick may start another.
    if (!_poolStopping && _poolThreadCount > 0)
      --_poolThreadCount;
    return;
  }
  // Anything sent so far came before its first task (the mutex is held from here until it takes
  // one or waits for one).
  clearStaleInterrupt();
  std::uint64_t interruptsSeen = _poolInterruptGen;
  for (;;) {
    while (!_poolStopping && !poolCanStartLocked()) {
      ++_idleWorkers;
      _poolWorkerWake.wait(lock);
      // Signalled or not (a spurious wake), it leaves the waiting workers' count.
      if (_signalledWorkers > 0)
        --_signalledWorkers;
      else if (_idleWorkers > 0)
        --_idleWorkers;
    }
    if (_poolStopping)
      return;
    if (interruptsSeen != _poolInterruptGen) {
      interruptsSeen = _poolInterruptGen;
      clearStaleInterrupt();
    }
    ThreadPoolTask task = takePoolTaskLocked();
    const PoolKick kick = poolKickLocked(); // more work it may start: pass it on
    lock.unlock();
    applyPoolKick(kick);
    if (task.keyed) {
      startKeyedPoolThread(task); // its own thread counts itself out
      lock.lock();
      continue;
    }
    {
      boost::this_thread::restore_interruption interruptible(uninterruptible);
      runPoolTask(task);
    }
    // Its captures go here, on the worker, before it counts as done: nothing of a finished
    // task outlives it.
    task.task = nullptr;
    lock.lock();
    --_activeWorkers;
    notifyPoolIdleLocked();
  }
}

void app::runPoolTask(ThreadPoolTask &task) {
  try {
    task.task();
  } catch (boost::thread_interrupted &) {
    // Interrupted (wait_for_threads, ~app): the task ends.
  } catch (std::exception &e) {
    try {
      log(0, std::string("Thread pool task error: ") + e.what());
    } catch (...) {
    }
  } catch (...) {
    try {
      log(0, "Thread pool task encountered unknown error");
    } catch (...) {
    }
  }
}

bool app::startKeyedPoolThread(ThreadPoolTask &task, bool logDrop) {
  // As startThread does: the thread waits at a gate until it is in threads() under its key, so
  // keyed progress and info calls from the task find it. Registering must not be interrupted
  // half way (the thread would wait at the gate forever), and a thread already running under
  // the key is replaced the way threads(key, ...) always does it.
  boost::this_thread::disable_interruption uninterruptible;
  auto ready = std::make_shared<std::atomic<bool>>(false);
  auto owned = std::make_shared<ThreadPoolTask>();
  owned->key.swap(task.key);
  owned->task.swap(task.task);
  owned->priority = task.priority;
  owned->keyed = true;
  owned->seq = task.seq;
  const std::string key = owned->key;
  thread_ptr thread;
  bool created = true;
  try {
    if (_poolWorkerSpawnHook)
      _poolWorkerSpawnHook(); // a test making the creation fail (app.h)
    thread.reset(new boost::thread([this, ready, owned]() mutable {
      while (!ready->load(std::memory_order_acquire))
        boost::this_thread::yield();
      std::shared_ptr<ThreadPoolTask> mine;
      mine.swap(owned); // the thread object keeps nothing of the task once it is done
      runKeyedPoolTask(*mine);
    }));
  } catch (...) {
    created = false;
  }
  if (!created) {
    // No thread (resources): the task can't run. It is dropped, and its slot freed. The caller
    // that submitted it hears of it (enqueuePoolTask); one started later is logged, as there is
    // no caller to tell. Logged while it still holds its slot, so that a pool seen idle (wait(),
    // the counts) has logged it. (Out of the catch: the kick below may drop tasks, whose
    // destructors call out.)
    owned.reset();
    if (logDrop) {
      try {
        log(0, "Thread pool: no thread could be started for keyed task '" + key +
                   "': dropped unrun");
      } catch (...) {
      }
    }
    PoolKick kick;
    {
      boost::mutex::scoped_lock lock(_threadPoolMutex);
      --_activeWorkers;
      kick = poolKickLocked();
      notifyPoolIdleLocked();
    }
    applyPoolKick(kick);
    return false;
  }
  try {
    threads(key, thread);
  } catch (...) {
  }
  ready->store(true, std::memory_order_release);
  return true;
}

void app::threadPoolWorker(ThreadPoolTask task) { runKeyedPoolTask(task); }

void app::runKeyedPoolTask(ThreadPoolTask &task) {
  {
    thread_info ti(*this, "Processing: " + task.key);
    runPoolTask(task);
    // Its captures go before it counts as done, with interruption held off as on an anonymous
    // task's worker: a destructor that reaches an interruption point (a join, say) must not throw.
    boost::this_thread::disable_interruption destroying;
    task.task = nullptr;
  }
  // The next task, if one may start: a keyed one starts here, before this thread ends, so that
  // joining this thread and then looking up the next key finds it (a chain of keyed tasks on a
  // full pool, as before); an anonymous one goes to a worker.
  boost::this_thread::disable_interruption uninterruptible;
  ThreadPoolTask next;
  PoolKick kick;
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    --_activeWorkers;
    if (poolCanStartLocked() && _pendingTasks.top().keyed)
      next = takePoolTaskLocked();
    kick = poolKickLocked();
    notifyPoolIdleLocked();
  }
  applyPoolKick(kick);
  if (next.keyed)
    startKeyedPoolThread(next);
}

void app::interruptPoolWorkersLocked() {
  // _threadPoolMutex is held, so each worker is either before its next take (and clears this as
  // stale, having seen the new _poolInterruptGen) or running a task this interrupts.
  ++_poolInterruptGen;
  for (thread_ptr &w : _poolWorkers) {
    try {
      if (w)
        w->interrupt();
    } catch (...) {
    }
  }
}

std::vector<thread_ptr> app::closeThreadPool() {
  std::vector<thread_ptr> workers;
  PoolQueue dropped;
  {
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    _poolStopping = true;
    // Interrupt the tasks still running (taken before this lock: none is taken after it).
    interruptPoolWorkersLocked();
    workers.swap(_poolWorkers);
    dropped.swap(_pendingTasks); // queued tasks never run
  }
  _poolWorkerWake.notify_all();
  _poolWorkerAdmitted.notify_all(); // a worker whose gate is open but not yet signalled
  _threadPoolIdle.notify_all();
  // The queued tasks go before anything is joined, unrun, without the mutex: what they captured
  // may report the drop (async_host settles its call, launch_pool_task posts on_error), and no
  // stuck task delays that. Not interruptible: an interrupt pending on this thread stays
  // pending instead of being thrown in a task's destructor (and dropped by its catch).
  {
    boost::this_thread::disable_interruption keep_interrupt_pending;
    pool_drop_scope reason(kPoolStopped);
    PoolQueue().swap(dropped);
  }
  return workers;
}

void app::joinPoolWorkers(std::vector<thread_ptr> &workers) {
  // Not interruptible: an interrupt pending on this thread would cut a join short (and be dropped
  // by the catch below). It stays pending instead.
  boost::this_thread::disable_interruption keep_interrupt_pending;
  for (thread_ptr &w : workers) {
    try {
      if (w && w->joinable())
        w->try_join_for(boost::chrono::seconds(5));
    } catch (...) {
    }
  }
}

void app::stopThreadPool() {
  std::vector<thread_ptr> workers = closeThreadPool();
  joinPoolWorkers(workers);
}

// ---------------------------------------------------------------------------
// Process-wide free helpers (no app instance required)
// ---------------------------------------------------------------------------
namespace {

// Meyer's singleton: built once on first call, thread-safe since C++11.
const std::map<std::string, data_type> &staticDataTypeEnum() {
  static const std::map<std::string, data_type> m = [] {
    std::map<std::string, data_type> mm;
    mm[typeid(char).name()] = Char;
    mm[typeid(unsigned char).name()] = UChar;
    mm[typeid(unsigned short).name()] = UShort;
    mm[typeid(int).name()] = Int;
    mm[typeid(unsigned int).name()] = UInt;
    mm[typeid(float).name()] = Float;
    mm[typeid(double).name()] = Double;
    mm[typeid(int64).name()] = Int64;
    mm[typeid(uint64).name()] = UInt64;
    return mm;
  }();
  return m;
}

const char *staticDataTypeName(data_type dt) {
  switch (dt) {
  case Char:
    return "char";
  case UChar:
    return "unsigned char";
  case UShort:
    return "unsigned short";
  case Int:
    return "int";
  case UInt:
    return "unsigned int";
  case Float:
    return "float";
  case Double:
    return "double";
  case Int64:
    return "int64";
  case UInt64:
    return "uint64";
  default:
    return "";
  }
}

} // anonymous namespace

template <class T> data_type dataType() {
  const auto &m = staticDataTypeEnum();
  auto it = m.find(typeid(T).name());
  return (it != m.end()) ? it->second : Undefined;
}

// Explicit instantiations for every type registered in registerDefaultTypes.
template data_type dataType<char>();
template data_type dataType<unsigned char>();
template data_type dataType<unsigned short>();
template data_type dataType<int>();
template data_type dataType<unsigned int>();
template data_type dataType<float>();
template data_type dataType<double>();
template data_type dataType<int64>();
template data_type dataType<uint64>();

std::string dataTypeName(data_type dt) { return std::string(staticDataTypeName(dt)); }
} // namespace cvc

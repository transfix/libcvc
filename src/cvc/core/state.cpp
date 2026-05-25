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

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/current_function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/regex.hpp>
#include <cvc/app.h>
#include <cvc/exception.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_message.h>
#include <cvc/utility.h>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cvc {
// SEPARATOR is now an inline variable defined in state.h (C++17).
// Keeping the symbol exported from the DLL would otherwise require an
// explicit __declspec(dllexport), since WINDOWS_EXPORT_ALL_SYMBOLS only
// covers function symbols, not data members.
state::init_func_vec state::_startup;
state::app_init_func_vec state::_appStartup;

namespace {

// Guards the two on_startup() registries (_startup, _appStartup)
// against concurrent registration and snapshot. Lives in this TU
// rather than as a static class member so it does not look like
// a singleton-of-state. The registries themselves are global
// configuration tables (see state::on_startup()).
boost::mutex &startup_registry_mutex() {
  static boost::mutex m;
  return m;
}

// Per-app mutex keyed by &app, used as a brief check-then-create
// critical section around the lazy root creation in instancePtr().
// Replaces the previous process-wide _instanceMutex so two
// independent apps no longer contend on each other's first-root
// creation.
boost::mutex &instance_registry_mutex() {
  static boost::mutex m;
  return m;
}

boost::mutex &mutex_for_app(app &ctx) {
  static std::map<const app *, std::shared_ptr<boost::mutex>> per_app;
  std::shared_ptr<boost::mutex> m;
  {
    boost::mutex::scoped_lock lock(instance_registry_mutex());
    auto &slot = per_app[&ctx];
    if (!slot)
      slot = std::make_shared<boost::mutex>();
    m = slot;
  }
  return *m;
}

} // namespace

// ------------
// state::state
// ------------
// Purpose:
//   Constructor for a state object.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
// 02/20/2012 -- Joe R. -- Adding notifyXmlRpc slot.
// 03/02/2012 -- Joe R. -- Setting last mod to minimum date by default.
// 03/15/2012 -- Joe R. -- Added initialized flag.
// 03/30/2012 -- Joe R. -- Added hidden flag.
// 01/12/2014 -- Joe R. -- Removing notifyXmlRpc.
state::state(app &ctx, const std::string &n, const state *p)
    : _ctx(ctx), _name(n), _parent(p), _lastMod(boost::posix_time::min_date_time), _hidden(false),
      _readOnly(false), _initialized(false) {
  // This slot propagates child changes up to parents
  childChanged.connect(
      map_change_signal::slot_type(&state::notifyParent, this, boost::placeholders::_1));
}

// -------------
// state::~state
// -------------
// Purpose:
//   Destructor.  Just signals that this object has been destroyed.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
state::~state() { destroyed(); }

// ------------------
// state::instancePtr
// ------------------
// ------------------
// state::instancePtr
// ------------------
// Purpose:
//   Returns a pointer to the root state object for the given app.
//   Stores it on the app's data map under "__state". Fires every
//   registered startup callback (nullary and per-app) exactly
//   once for this app the first time its root is created.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
// 01/12/2014 -- Joe R. -- Added startup function calls to do initialization based on cvcstate.
//                         Also moved xmlrpc server thread start elsewhere.
// 05/03/2026 -- Joe R. -- Per-app overload, decoupling state root from
//                         app::instance(). Legacy zero-arg form removed.
// 05/21/2026 -- Joe R. -- Removed singleton-flavored statics: process-wide
//                         _instanceMutex is now a per-app mutex via
//                         mutex_for_app(), and the once-per-process
//                         _startupFired flag is replaced by once-per-app
//                         firing keyed off the app data map.
state::state_ptr state::instancePtr(app &ctx) {
  bool fire_startup = false;
  state_ptr ptr;
  {
    // Per-app critical section around check-then-create on the
    // app's data map. Two distinct apps do not contend here.
    boost::mutex::scoped_lock lock(mutex_for_app(ctx));
    const std::string statekey("__state");
    try {
      ptr = ctx.data<state_ptr>(statekey);
    } catch (std::exception &) {
      // not yet present
    }
    if (!ptr) {
      ptr.reset(new state(ctx));
      ctx.data(statekey, ptr);
      fire_startup = true;
    }
  }

  if (fire_startup) {
    // Snapshot both registries under the registry guard so we do
    // not hold it while user callbacks run. Callbacks fire once
    // per app (not once per process) — see state.h.
    init_func_vec nullary_snapshot;
    app_init_func_vec app_snapshot;
    {
      boost::mutex::scoped_lock lock(startup_registry_mutex());
      nullary_snapshot = _startup;
      app_snapshot = _appStartup;
    }
    BOOST_FOREACH (nullary_func &init_func, nullary_snapshot) {
      init_func();
    }
    BOOST_FOREACH (app_init_func &init_func, app_snapshot) {
      init_func(ctx);
    }
  }

  return ptr;
}

// ---------------
// state::instance
// ---------------
// Purpose:
//   Returns a reference to the root state object for the given app.
state &state::instance(app &ctx) { return *instancePtr(ctx); }

// ----------
// app::root
// ----------
// Purpose:
//   Defined here (not in app.cpp) because returning a state& requires
//   the full state definition. Routes through state::instance(app&)
//   so both spellings produce the same per-app root.
state &app::root() { return state::instance(*this); }

// --------------
// state::lastMod
// --------------
// Purpose:
//   Returns the time this object was last modified.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
boost::posix_time::ptime state::lastMod() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _lastMod;
}

// ------------
// state::value
// ------------
// Purpose:
//   Returns the string value of this object.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
std::string state::value() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _value;
}

// --------------------
// state::valueTypeName
// --------------------
// Purpose:
//   Returns the type of the value as a string.
// ---- Change History ----
// 03/31/2012 -- Joe R. -- Creation.
std::string state::valueTypeName() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _valueTypeName;
}

// --------------
// state::comment
// --------------
// Purpose:
//   Returns the string comment for this object.
// ---- Change History ----
// 03/30/2012 -- Joe R. -- Creation.
std::string state::comment() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _comment;
}

// --------------
// state::comment
// --------------
// Purpose:
//   Sets a comment for this state object, useful at runtime for the user.
// ---- Change History ----
// 03/30/2012 -- Joe R. -- Creation.
state &state::comment(const std::string &c) {
  boost::this_thread::interruption_point();
  if (comment() == c)
    return *this; // do nothing if equal

  {
    boost::mutex::scoped_lock lock(_mutex);
    _comment = c;
    _lastMod = boost::posix_time::microsec_clock::universal_time();
    _initialized = true;
  }

  commentChanged();
  if (parent())
    parent()->childChanged(name());
  return *this;
}

// -------------
// state::hidden
// -------------
// Purpose:
//   Returns the hidden flag for this object.
// ---- Change History ----
// 03/30/2012 -- Joe R. -- Creation.
bool state::hidden() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _hidden;
}

// -------------
// state::hidden
// -------------
// Purpose:
//   Sets a hidden flag for this state object, useful to hide internal API
//   state objects that users shouldn't change.
// ---- Change History ----
// 03/30/2012 -- Joe R. -- Creation.
state &state::hidden(bool h) {
  boost::this_thread::interruption_point();
  if (hidden() == h)
    return *this; // do nothing if equal

  {
    boost::mutex::scoped_lock lock(_mutex);
    _hidden = h;
    _lastMod = boost::posix_time::microsec_clock::universal_time();
    _initialized = true;
  }

  hiddenChanged();
  if (parent())
    parent()->childChanged(name());
  return *this;
}

// ---------------
// state::readOnly
// ---------------
// Purpose:
//   Returns the read-only flag for this state object.
// ---- Change History ----
// 12/30/2025 -- Joe R. -- Creation.
bool state::readOnly() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _readOnly;
}

// ---------------
// state::readOnly
// ---------------
// Purpose:
//   Sets a read-only flag for this state object. When set, attempts to
//   modify value or data will throw a read_only_error exception.
// ---- Change History ----
// 12/30/2025 -- Joe R. -- Creation.
state &state::readOnly(bool ro) {
  boost::this_thread::interruption_point();
  if (readOnly() == ro)
    return *this; // do nothing if equal

  {
    boost::mutex::scoped_lock lock(_mutex);
    _readOnly = ro;
    _lastMod = boost::posix_time::microsec_clock::universal_time();
    _initialized = true;
  }

  readOnlyChanged();
  if (parent())
    parent()->childChanged(name());
  return *this;
}

// -------------
// state::values
// -------------
// Purpose:
//   Returns a vector of strings if the value of the object
//   is comma separated.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
std::vector<std::string> state::values(bool unique) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);

  using namespace std;
  using namespace boost;
  using namespace boost::algorithm;

  vector<string> vals;
  if (_value.empty())
    return vals;

  string valstr = _value;
  split(vals, valstr, is_any_of(","));
  BOOST_FOREACH (string &val, vals)
    trim(val);
  if (unique) {
    set<string> vals_set;
    copy(vals.begin(), vals.end(), inserter(vals_set, vals_set.begin()));
    vals.resize(vals_set.size());
    copy(vals_set.begin(), vals_set.end(), vals.begin());
  }
  return vals;
}

// ------------
// state::value
// ------------
// Purpose:
//   Sets the value of this object.  Returns a reference to this
//   to make it possible to add this to a chain of commands.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
// 03/15/2012 -- Joe R. -- Added initialized flag.
state &state::value(const std::string &v, bool setValueType) {
  boost::this_thread::interruption_point();

  // Get fullName before locking
  std::string full_name = fullName();

  // Phase 8: writes through a writable transparent link route to the
  // resolved target. Opaque links and non-writable transparent links
  // accept writes on the link node itself (the historical default).
  {
    link_mode mode_snapshot;
    bool writable_snapshot;
    bool is_link_snapshot;
    {
      boost::mutex::scoped_lock lock(_mutex);
      is_link_snapshot = !_linkTarget.empty();
      mode_snapshot = _linkMode;
      writable_snapshot = _linkWritable;
    }
    if (is_link_snapshot && mode_snapshot == link_mode::transparent && writable_snapshot) {
      link_resolution r = resolveLink();
      if (r.kind == link_resolution_kind::resolved && r.target != nullptr && r.target != this) {
        r.target->value(v, setValueType);
        return *this;
      }
      throw read_only_error(boost::str(
          boost::format("Cannot write through unresolvable transparent link: %1%") % full_name));
    }
  }

  // Check if this state is read-only
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_readOnly) {
      throw read_only_error(
          boost::str(boost::format("Cannot modify read-only state: %1%") % full_name));
    }
  }

  if (value() == v)
    return *this; // do nothing if equal

  {
    boost::mutex::scoped_lock lock(_mutex);
    _value = v;
    _lastMod = boost::posix_time::microsec_clock::universal_time();
    _initialized = true;

    if (setValueType)
      _valueTypeName = std::string("std::string");

    // Notify any threads waiting for value
    _valueCondition.notify_all();
  }

  valueChanged();
  if (parent())
    parent()->childChanged(name());
  return *this;
}

// ------------
// state::touch
// ------------
// Purpose:
//   Triggers signals as if this state obj changed.
// ---- Change History ----
// 03/02/2012 -- Joe R. -- Creation.
void state::touch() {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_mutex);
    _lastMod = boost::posix_time::microsec_clock::universal_time();
  }
  valueChanged();
  dataChanged();
  if (parent())
    parent()->childChanged(name());
}

// ------------
// state::reset
// ------------
// Purpose:
//   Sets value and data to default state, and does the same for
//   all children.
// ---- Change History ----
// 03/16/2012 -- Joe R. -- Creation.
// 03/30/2012 -- Joe R. -- Resetting comment.
void state::reset(bool resetChildren, bool fireCallbacks) {
  boost::this_thread::interruption_point();
  {
    boost::mutex::scoped_lock lock(_mutex);
    _value = std::string();
    _valueTypeName = std::string();
    _data = boost::any();
    _comment = std::string();
    _hidden = false;
    _initialized = false;
    if (resetChildren) {
      BOOST_FOREACH (child_map::value_type val, _children)
        val.second->reset(resetChildren, fireCallbacks);
    } else {
      // Detach children without resetting them - they persist via shared_ptr
      _children.clear();
    }
  }
  if (fireCallbacks) {
    touch();
  }
}

// ------------
// state::ptree
// ------------
// Purpose:
//   Returns a property tree describing this state object
//   and it's children.  Only stores string values, not 'data'.
// ---- Change History ----
// 03/16/2012 -- Joe R. -- Creation.
boost::property_tree::ptree state::ptree() {
  using namespace boost;
  property_tree::ptree pt;

  boost::this_thread::interruption_point();
  _ctx.log(6, boost::str(boost::format("%s :: %s = %s\n") % BOOST_CURRENT_FUNCTION % fullName() %
                         value()));

  if (!value().empty())
    pt.push_back(property_tree::ptree::value_type(fullName(), property_tree::ptree(value())));

  {
    boost::mutex::scoped_lock lock(_mutex);
    BOOST_FOREACH (child_map::value_type val, _children) {
      boost::this_thread::interruption_point();
      property_tree::ptree child_pt = val.second->ptree();
      pt.push_back(property_tree::ptree::value_type(val.second->fullName(), child_pt));
    }
  }

  return pt;
}

// ------------
// state::ptree
// ------------
// Purpose:
//  Sets this state object and its children based on an incoming
//  property tree.
// ---- Change History ----
// 03/16/2012 -- Joe R. -- Creation.
void state::ptree(const boost::property_tree::ptree &pt) {
  using namespace boost;
  BOOST_FOREACH (const property_tree::ptree::value_type &v, pt)
    (*this)(v.first).value(v.second.get_value<std::string>());
}

// -----------
// state::json
// -----------
// Purpose:
//  Returns a json string version of the property map.
// ---- Change History ----
// 01/12/2014 -- Joe R. -- Creation.
std::string state::json() { return cvc::json(ptree()); }

// -----------
// state::json
// -----------
// Purpose:
//  Sets this state object and its children based on an incoming json.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
void state::json(const std::string &j) { ptree(cvc::json(j)); }

// -----------
// state::save
// -----------
// Purpose:
//  Saves this state object and its children to the specified filename.
// ---- Change History ----
// 03/16/2012 -- Joe R. -- Creation.
// 01/12/2014 -- Joe R. -- Forcing json.
void state::save(const std::string &filename) { write_json(filename, ptree()); }

// --------------
// state::restore
// --------------
// Purpose:
//  Restores this state object and its children from the specified filename.
// ---- Change History ----
// 03/16/2012 -- Joe R. -- Creation.
// 01/12/2014 -- Joe R. -- Forcing json.
void state::restore(const std::string &filename) {
  using namespace boost;
  property_tree::ptree pt;
  read_json(filename, pt);
  ptree(pt);
}

// ---------------
// state::traverse
// ---------------
// Purpose:
//  Traverses the state tree, calling func for this and each child.
//  Use 're' to filter what children get visited.
// ---- Change History ----
// 03/16/2012 -- Joe R. -- Creation.
// 04/15/2012 -- Joe R. -- Triggering enter/exit signals.
void state::traverse(traversal_unary_func func, const std::string &re) {
  traverseEnter();
  func(fullName());
  std::vector<std::string> ch = children(re);
  BOOST_FOREACH (std::string c, ch)
    instance(_ctx)(c).traverse(func, re);
  traverseExit();
}

// -----------
// state::data
// -----------
// Purpose:
//   Returns the data of this object.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
boost::any state::data() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _data;
}

// -----------
// state::data
// -----------
// Purpose:
//   Sets this object's data.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
// 03/15/2012 -- Joe R. -- Added initialized flag.
// 04/20/2012 -- Joe R. -- Returning reference to this.
state &state::data(const boost::any &d) {
  boost::this_thread::interruption_point();

  // Get fullName before locking
  std::string full_name = fullName();

  // Check if this state is read-only
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_readOnly) {
      throw read_only_error(
          boost::str(boost::format("Cannot modify read-only state: %1%") % full_name));
    }
  }

  {
    boost::mutex::scoped_lock lock(_mutex);
    _data = d;
    _lastMod = boost::posix_time::microsec_clock::universal_time();
    _initialized = true;

    // Notify any threads waiting for data
    _dataCondition.notify_all();
  }
  dataChanged();
  if (parent())
    parent()->childChanged(name());
  return *this;
}

// -------------------
// state::dataTypeName
// -------------------
// Purpose:
//   Returns a string representing the type of the data.
// ---- Change History ----
// 03/31/2012 -- Joe R. -- Creation.
std::string state::dataTypeName() { return _ctx.dataTypeName(data()); }

// -----------------
// state::operator()
// -----------------
// Purpose:
//   Used for child object lookups.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
// 03/15/2012 -- Joe R. -- Added initialized flag.
state &state::operator()(const std::string &childname) {
  using namespace std;
  using namespace boost::algorithm;

  boost::this_thread::interruption_point();

  vector<string> keys;
  split(keys, childname, is_any_of(SEPARATOR));
  if (keys.empty())
    return *this;
  BOOST_FOREACH (string &key, keys)
    trim(key);
  // Ignore beginning empty keys
  while (!keys.empty() && keys.front().empty())
    keys.erase(keys.begin());
  if (keys.empty())
    return *this;

  string nearest = keys.front();
  keys.erase(keys.begin());
  {
    boost::mutex::scoped_lock lock(_mutex);
    // If we have the child state in our map, take out its part of the
    // keys vector and recursively call its operator().
    // If not, create a new one.
    if (_children.find(nearest) != _children.end() && _children[nearest])
      return (*_children[nearest])(join(keys, SEPARATOR));
    else {
      state_ptr s(new state(_ctx, nearest, this));
      _children[nearest] = s;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;
      return (*_children[nearest])(join(keys, SEPARATOR));
    }
  }
}

// ----------------
// state::linkTo / clearLink / isLink / linkTarget / resolveLink
// ----------------
// Purpose:
//   Phase 8 link-node operations. linkTo() marks this node as a
//   reference to another absolute path; resolveLink() walks the
//   chain with cycle detection and a hop budget.
// ----------------

namespace {

// Normalize a state path: trim, drop leading/trailing/duplicate
// SEPARATORs, return the canonical dot-separated form. Empty
// string means "the root".
std::string normalize_state_path(const std::string &p) {
  using namespace boost::algorithm;
  std::string s = p;
  trim(s);
  std::vector<std::string> keys;
  split(keys, s, is_any_of(state::SEPARATOR));
  std::vector<std::string> kept;
  kept.reserve(keys.size());
  for (auto &k : keys) {
    trim(k);
    if (!k.empty())
      kept.push_back(k);
  }
  return join(kept, state::SEPARATOR);
}

} // namespace

state &state::linkTo(const std::string &target_path) {
  std::string normalized = normalize_state_path(target_path);
  // Canonical root-link marker: any separator-only input (".",
  // "..", "  .  ", etc.) is interpreted as "link to the app
  // root" and stored as "." (DNS-style). A genuinely empty or
  // whitespace-only input still maps to "" and acts like
  // clearLink(). _linkTarget.empty() therefore continues to mean
  // "not a link"; "." means "link to root".
  {
    std::string trimmed = boost::algorithm::trim_copy(target_path);
    if (normalized.empty() && !trimmed.empty())
      normalized = state::SEPARATOR;
  }
  bool changed = false;
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_linkTarget != normalized) {
      _linkTarget = normalized;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;
      changed = true;
    }
  }
  if (changed) {
    linkChanged();
    if (parent())
      parent()->childChanged(name());
  }
  return *this;
}

bool state::clearLink() {
  bool was_link = false;
  {
    boost::mutex::scoped_lock lock(_mutex);
    was_link = !_linkTarget.empty();
    if (was_link) {
      _linkTarget.clear();
      _lastMod = boost::posix_time::microsec_clock::universal_time();
    }
    _linkMode = link_mode::opaque;
    _linkWritable = false;
  }
  if (was_link) {
    linkChanged();
    if (parent())
      parent()->childChanged(name());
  }
  return was_link;
}

state &state::linkTo(const std::string &target_path, link_mode mode) {
  std::string normalized = normalize_state_path(target_path);
  {
    std::string trimmed = boost::algorithm::trim_copy(target_path);
    if (normalized.empty() && !trimmed.empty())
      normalized = state::SEPARATOR;
  }
  bool changed = false;
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_linkTarget != normalized || _linkMode != mode) {
      _linkTarget = normalized;
      _linkMode = mode;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;
      changed = true;
    }
  }
  if (changed) {
    linkChanged();
    if (parent())
      parent()->childChanged(name());
  }
  return *this;
}

state::link_mode state::linkMode() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  return _linkMode;
}

state &state::setLinkMode(link_mode mode) {
  bool changed = false;
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_linkMode != mode) {
      _linkMode = mode;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      changed = true;
    }
  }
  if (changed) {
    linkChanged();
    if (parent())
      parent()->childChanged(name());
  }
  return *this;
}

bool state::linkWritable() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  return _linkWritable;
}

state &state::setLinkWritable(bool writable) {
  bool changed = false;
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_linkWritable != writable) {
      _linkWritable = writable;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      changed = true;
    }
  }
  if (changed) {
    linkChanged();
    if (parent())
      parent()->childChanged(name());
  }
  return *this;
}

std::string state::resolvedValue(std::size_t hop_budget) {
  // Cheap fast path: not a link, or opaque link, return own value.
  {
    boost::mutex::scoped_lock lock(_mutex);
    if (_linkTarget.empty() || _linkMode != link_mode::transparent)
      return _value;
  }
  link_resolution r = resolveLink(hop_budget);
  if (r.kind == link_resolution_kind::resolved && r.target != nullptr && r.target != this)
    return r.target->value();
  // Broken / cycle / budget exhausted: fall back to own value.
  return value();
}

bool state::isLink() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  return !_linkTarget.empty();
}

std::string state::linkTarget() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  return _linkTarget;
}

state *state::findDescendant(const std::string &path) {
  using namespace boost::algorithm;
  std::string normalized = normalize_state_path(path);
  if (normalized.empty())
    return this;
  std::vector<std::string> keys;
  split(keys, normalized, is_any_of(SEPARATOR));
  state *cur = this;
  for (auto &k : keys) {
    trim(k);
    if (k.empty())
      continue;
    boost::mutex::scoped_lock lock(cur->_mutex);
    auto it = cur->_children.find(k);
    if (it == cur->_children.end() || !it->second) {
      return nullptr;
    }
    cur = it->second.get();
  }
  return cur;
}

state::link_resolution state::resolveLink(std::size_t hop_budget) {
  link_resolution result;

  // The root for absolute target lookup is the per-app root.
  state &root = state::instance(_ctx);

  // Visited-set keyed by absolute path. Single-tree for now;
  // when multi-tree lands the key extends to (tree_id, path).
  std::unordered_set<std::string> seen;

  state *cur = this;
  // Record the starting node's path so cycles that loop back to
  // the start (including a self-link) are detected as cycles
  // rather than mistakenly classified as "resolved".
  seen.insert(cur->fullName());
  result.visited.push_back(cur->fullName());

  while (true) {
    std::string target;
    {
      boost::mutex::scoped_lock lock(cur->_mutex);
      target = cur->_linkTarget;
    }
    if (target.empty()) {
      // Terminal node: not a link.
      result.kind = (cur == this) ? link_resolution_kind::none : link_resolution_kind::resolved;
      result.target = cur;
      return result;
    }

    if (result.hops >= hop_budget) {
      result.kind = link_resolution_kind::budget_exhausted;
      result.target = nullptr;
      return result;
    }

    state *next = root.findDescendant(target);
    if (next == nullptr) {
      result.kind = link_resolution_kind::broken;
      result.target = nullptr;
      result.visited.push_back(target);
      return result;
    }
    ++result.hops;

    std::string next_path = next->fullName();
    if (!seen.insert(next_path).second) {
      result.kind = link_resolution_kind::cycle_detected;
      result.target = nullptr;
      result.visited.push_back(next_path);
      return result;
    }
    result.visited.push_back(next_path);
    cur = next;
  }
}

// ---------------
// state::resolveRemote
// ---------------
// Purpose:
//   Extends resolveLink() with authority-map awareness. When a
//   link target is not found locally, consults the default shard's
//   delegation manager to determine whether the target path is
//   owned by a remote cluster (resolved_remote), has an expired
//   lease (lease_expired), or is genuinely absent (broken).
// -------------------
state::remote_link_resolution state::resolveRemote(std::size_t hop_budget) {
  remote_link_resolution result;

  state &root = state::instance(_ctx);
  std::unordered_set<std::string> seen;
  state *cur = this;
  seen.insert(cur->fullName());
  result.visited.push_back(cur->fullName());

  while (true) {
    std::string target;
    {
      boost::mutex::scoped_lock lock(cur->_mutex);
      target = cur->_linkTarget;
    }
    if (target.empty()) {
      // Terminal node: not a link.
      result.kind =
          (cur == this) ? remote_resolution_kind::none : remote_resolution_kind::resolved_local;
      result.target = cur;
      result.resolved_path = cur->fullName();
      result.owner_is_local = true;
      // If a default shard exists, report its cluster_id.
      state_cluster_shard *shard = state_cluster_shard::default_for(_ctx);
      if (shard != nullptr)
        result.owner_cluster_id = shard->cluster_id();
      return result;
    }

    if (result.hops >= hop_budget) {
      result.kind = remote_resolution_kind::budget_exhausted;
      result.target = nullptr;
      return result;
    }

    state *next = root.findDescendant(target);
    if (next == nullptr) {
      // Target does not exist locally. Consult the authority map
      // via the default shard's delegation manager.
      state_cluster_shard *shard = state_cluster_shard::default_for(_ctx);
      if (shard != nullptr) {
        auto decision = shard->delegation().route(target);
        if (decision.kind == state_delegation_manager::route_kind::remote) {
          result.kind = remote_resolution_kind::resolved_remote;
          result.target = nullptr;
          result.resolved_path = target;
          result.owner_cluster_id = decision.cluster_id;
          result.endpoint = decision.endpoint;
          result.owner_is_local = false;
          result.visited.push_back(target);
          ++result.hops;
          return result;
        }
        if (decision.kind == state_delegation_manager::route_kind::expired) {
          result.kind = remote_resolution_kind::lease_expired;
          result.target = nullptr;
          result.resolved_path = target;
          result.owner_cluster_id = decision.cluster_id;
          result.endpoint = decision.endpoint;
          result.owner_is_local = false;
          result.visited.push_back(target);
          ++result.hops;
          return result;
        }
      }
      // No shard, or delegation says local but node absent: broken.
      result.kind = remote_resolution_kind::broken;
      result.target = nullptr;
      result.resolved_path = target;
      result.visited.push_back(target);
      return result;
    }
    ++result.hops;

    std::string next_path = next->fullName();
    if (!seen.insert(next_path).second) {
      result.kind = remote_resolution_kind::cycle_detected;
      result.target = nullptr;
      result.resolved_path = next_path;
      result.visited.push_back(next_path);
      return result;
    }
    result.visited.push_back(next_path);
    cur = next;
  }
}

// ---------------
// state::sendMessage
// ---------------
// Purpose:
//   Cluster-agnostic out-of-band send. Follows any link chain
//   to its terminal node, looks up the default shard for this
//   state's app context, and delegates to
//   state_cluster_shard::send_message(). The developer API never
//   names a cluster_id: routing is the shard's responsibility.
// -------------------
state::send_message_result state::sendMessage(const std::string &payload,
                                              const std::string &content_type,
                                              std::size_t hop_budget) {
  send_message_result r;

  // 1. Resolve through any link chain.
  state *target = this;
  if (isLink()) {
    auto lr = resolveLink(hop_budget);
    switch (lr.kind) {
    case link_resolution_kind::resolved:
    case link_resolution_kind::none:
      target = lr.target;
      break;
    case link_resolution_kind::broken:
      r.status = send_message_result::status_kind::broken_link;
      if (!lr.visited.empty())
        r.resolved_path = lr.visited.back();
      return r;
    case link_resolution_kind::cycle_detected:
      r.status = send_message_result::status_kind::cycle_detected;
      if (!lr.visited.empty())
        r.resolved_path = lr.visited.back();
      return r;
    case link_resolution_kind::budget_exhausted:
      r.status = send_message_result::status_kind::budget_exhausted;
      if (!lr.visited.empty())
        r.resolved_path = lr.visited.back();
      return r;
    }
  }
  r.resolved_path = target->fullName();

  // 2. Find the default shard for this app context. With no
  // shard registered (common in unit tests of pure-state code)
  // the call is a structured no-op rather than an error.
  state_cluster_shard *shard = state_cluster_shard::default_for(_ctx);
  if (shard == nullptr) {
    r.status = send_message_result::status_kind::no_shard;
    return r;
  }

  // 3. Build the message and hand off. The shard fills in
  // cluster_id from its authority map; we never name it here.
  state_message m;
  m.path = r.resolved_path;
  m.content_type = content_type;
  m.string_value = payload;

  auto sr = shard->send_message(std::move(m));
  r.owner_cluster_id = sr.owner_cluster_id;
  r.owner_is_local = sr.owner_is_local;
  r.local_admitted = sr.local_admitted;
  r.peers_delivered = sr.peers_delivered;
  r.peers_targeted = sr.peers_targeted;
  switch (sr.status) {
  case state_cluster_shard::send_message_result::status_kind::delivered:
    r.status = send_message_result::status_kind::delivered;
    break;
  case state_cluster_shard::send_message_result::status_kind::duplicate_local:
    r.status = send_message_result::status_kind::duplicate_local;
    break;
  case state_cluster_shard::send_message_result::status_kind::no_transport:
    r.status = send_message_result::status_kind::no_transport;
    break;
  }
  return r;
}

// ---------------
// state::children
// ---------------
// Purpose:
//   Returns a vector of children state object names. Filters children by
//   a regular expression if regex isn't empty.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
// 02/24/2012 -- Joe R. -- Adding regex support.
// 01/12/2014 -- Joe R. -- Returning empty vector if invalid regex.
std::vector<std::string> state::children(const std::string &re) {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  std::vector<std::string> ret;
  BOOST_FOREACH (child_map::value_type val, _children) {
    if (!re.empty()) {
      try {
        boost::regex expression(re.c_str());
        boost::cmatch what;

        _ctx.log(6, boost::str(boost::format("%s :: check match %s\n") % BOOST_CURRENT_FUNCTION %
                               val.second->fullName()));

        if (boost::regex_match(val.second->fullName().c_str(), what, expression)) {
          _ctx.log(6, boost::str(boost::format("%s :: matched! %s\n") % BOOST_CURRENT_FUNCTION %
                                 val.second->fullName()));

          ret.push_back(val.second->fullName());
        }
      } catch (boost::bad_expression &) {
        _ctx.log(2, boost::str(boost::format("%s :: invalid regex '%s'\n") %
                               BOOST_CURRENT_FUNCTION % re));
        return ret;
      }
    } else
      ret.push_back(val.second->fullName());

    // Get any matches from this state's children if any.
    std::vector<std::string> childret = val.second->children(re);
    ret.insert(ret.end(), childret.begin(), childret.end());
  }
  return ret;
}

// ------------------
// state::numChildren
// ------------------
// Purpose:
//   Returns the number of children.
// ---- Change History ----
// 04/06/2012 -- Joe R. -- Creation.
size_t state::numChildren() {
  boost::this_thread::interruption_point();
  boost::mutex::scoped_lock lock(_mutex);
  return _children.size();
}

// -------- Expiring state --------

state &state::expireAt(boost::posix_time::ptime when) {
  // Root cannot be expired \u2014 there is nobody to detach it from.
  if (_parent == NULL)
    return *this;
  {
    boost::mutex::scoped_lock lock(_mutex);
    _expiryTime = when;
    _lastMod = boost::posix_time::microsec_clock::universal_time();
  }
  return *this;
}

state &state::expireAfter(boost::posix_time::time_duration d) {
  return expireAt(boost::posix_time::microsec_clock::universal_time() + d);
}

state &state::clearExpiry() {
  {
    boost::mutex::scoped_lock lock(_mutex);
    _expiryTime = boost::posix_time::ptime(); // not_a_date_time
    _lastMod = boost::posix_time::microsec_clock::universal_time();
  }
  return *this;
}

bool state::hasExpiry() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  return !_expiryTime.is_not_a_date_time();
}

boost::posix_time::ptime state::expiryTime() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  return _expiryTime;
}

bool state::isExpired() const {
  boost::mutex::scoped_lock lock(const_cast<boost::mutex &>(_mutex));
  if (_expiryTime.is_not_a_date_time())
    return false;
  return boost::posix_time::microsec_clock::universal_time() >= _expiryTime;
}

std::size_t state::sweepExpired() {
  boost::this_thread::interruption_point();

  // Snapshot direct children under our mutex so recursion happens
  // without holding the parent lock (avoids deadlock if subscribers
  // re-enter via signals).
  std::vector<state_ptr> snapshot;
  {
    boost::mutex::scoped_lock lock(_mutex);
    snapshot.reserve(_children.size());
    BOOST_FOREACH (child_map::value_type &val, _children) {
      snapshot.push_back(val.second);
    }
  }

  std::size_t removed = 0;
  BOOST_FOREACH (state_ptr &c, snapshot) {
    removed += c->sweepExpired();
  }

  // Now collect direct children that are themselves expired and
  // erase them from our child map. Holding shared_ptrs in
  // to_remove keeps them alive long enough to fire `expiring`.
  std::vector<std::pair<std::string, state_ptr>> to_remove;
  {
    boost::mutex::scoped_lock lock(_mutex);
    for (child_map::iterator it = _children.begin(); it != _children.end();) {
      if (it->second->isExpired()) {
        to_remove.push_back(std::make_pair(it->first, it->second));
        _children.erase(it++);
      } else {
        ++it;
      }
    }
  }

  // Fire `expiring` while the node is still alive, then drop our
  // reference so the dtor (and `destroyed`) fires next, then
  // notify our own parent chain that our child set changed.
  for (std::size_t i = 0; i < to_remove.size(); ++i) {
    to_remove[i].second->expiring();
    childChanged(to_remove[i].first);
    to_remove[i].second.reset();
    ++removed;
  }

  return removed;
}

// -----------------
// state::on_startup
// -----------------
// Purpose:
//   Add to the list of functions to call when first initializing cvcstate.
// ---- Change History ----
// 01/12/2014 -- Joe R. -- Creation.
void state::on_startup(const nullary_func &init_func) {
  boost::mutex::scoped_lock lock(startup_registry_mutex());
  _startup.push_back(init_func);
}

void state::on_startup(const app_init_func &init_func) {
  boost::mutex::scoped_lock lock(startup_registry_mutex());
  _appStartup.push_back(init_func);
}

// -------------------
// state::notifyParent
// -------------------
// Purpose:
//   Used to propagate child change signals up the tree to the root node.
//   Because of this, every change to the entire tree will trigger the root node's
//   childChanged signal.
// ---- Change History ----
// 02/18/2012 -- Joe R. -- Creation.
void state::notifyParent(const std::string &childname) {
  boost::this_thread::interruption_point();
  if (parent())
    parent()->childChanged(name() + SEPARATOR + childname);
}

// ------------------
// isValidStateName
// ------------------
// Purpose:
//   Validates that a state name conforms to C identifier rules:
//   - Must start with letter or underscore
//   - Can contain letters, digits, and underscores
//   - No special characters, spaces, or dashes
// ---- Change History ----
// 12/30/2025 -- Added for state name validation.
bool state::isValidStateName(const std::string &name) {
  if (name.empty()) {
    return false;
  }

  // First character must be letter or underscore
  char first = name[0];
  if (!std::isalpha(first) && first != '_') {
    return false;
  }

  // Remaining characters must be alphanumeric or underscore
  for (size_t i = 1; i < name.length(); ++i) {
    char c = name[i];
    if (!std::isalnum(c) && c != '_') {
      return false;
    }
  }

  return true;
}

// ------------------
// sanitizeStateName
// ------------------
// Purpose:
//   Converts an arbitrary string into a valid C identifier:
//   - Replaces invalid characters with underscores
//   - Ensures first character is valid (prepends underscore if needed)
//   - Handles empty strings
// ---- Change History ----
// 12/30/2025 -- Added for state name sanitization.
std::string state::sanitizeStateName(const std::string &name) {
  if (name.empty()) {
    return "unnamed";
  }

  std::string result;
  result.reserve(name.length());

  // Handle first character
  char first = name[0];
  if (std::isalpha(first) || first == '_') {
    result += first;
  } else if (std::isdigit(first)) {
    // If starts with digit, prepend underscore
    result += '_';
    result += first;
  } else {
    // Replace invalid first character with underscore
    result += '_';
  }

  // Handle remaining characters
  for (size_t i = 1; i < name.length(); ++i) {
    char c = name[i];
    if (std::isalnum(c) || c == '_') {
      result += c;
    } else {
      // Replace invalid characters (including dashes, spaces, etc.) with underscore
      result += '_';
    }
  }

  return result;
}
} // namespace cvc

namespace {
// -----------
// system_init
// -----------
// Purpose:
//   Sets the default system settings and initial info.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
class system_init {
public:
  static void init(cvc::app &ctx) {
    using namespace boost::posix_time;
    cvc::state::instance(ctx)("__system.start")
        .value(to_simple_string(microsec_clock::universal_time()));
  }

  system_init() { cvc::state::on_startup(cvc::state::app_init_func(init)); }
} static_init;
} // namespace

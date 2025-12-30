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

/* $Id: State.h 5559 2012-05-11 21:43:22Z transfix $ */

#ifndef __CVC_STATE_H__
#define __CVC_STATE_H__

#include <cvc/namespace.h>
#include <cvc/types.h>
#include <cvc/exception.h>
#include <cvc/app.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/function.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

#include <vector>

namespace CVC_NAMESPACE
{
  // Forward declaration for state_future
  class state;
  
  // ----------------
  // cvc::state_future
  // ----------------
  // Purpose:
  //   A future-like object that blocks until a state value is set.
  //   Provides async access to state values with timeout support.
  // ---- Change History ----
  // 12/08/2025 -- Added for async state value retrieval.
  template<typename T>
  class state_future
  {
  public:
    state_future(state* s);
    
    ~state_future() {
      if (_connection.connected()) {
        _connection.disconnect();
      }
    }
    
    // Move constructor and assignment
    state_future(state_future&& other) 
      : _state(other._state)
      , _ready(other._ready)
      , _has_value(other._has_value)
      , _connection(std::move(other._connection))
    {
      other._state = nullptr;
    }
    
    state_future& operator=(state_future&& other) {
      if (this != &other) {
        if (_connection.connected()) {
          _connection.disconnect();
        }
        _state = other._state;
        _ready = other._ready;
        _has_value = other._has_value;
        _connection = std::move(other._connection);
        other._state = nullptr;
      }
      return *this;
    }
    
    // Delete copy constructor and assignment
    state_future(const state_future&) = delete;
    state_future& operator=(const state_future&) = delete;
    
    // Block until value is available, then return it
    T get() {
      boost::unique_lock<boost::mutex> lock(_mutex);
      while (!_ready) {
        _condition.wait(lock);
      }
      return getValue();
    }
    
    // Block with timeout (returns false on timeout)
    bool wait_for(const boost::chrono::milliseconds& timeout) {
      boost::unique_lock<boost::mutex> lock(_mutex);
      return _condition.wait_for(lock, timeout, [this]() { return _ready; });
    }
    
    // Get value with timeout (throws on timeout)
    T get_for(const boost::chrono::milliseconds& timeout) {
      if (!wait_for(timeout)) {
        throw timeout_error("state_future timeout waiting for value");
      }
      return getValue();
    }
    
    // Check if value is ready without blocking
    bool is_ready() const {
      boost::mutex::scoped_lock lock(_mutex);
      return _ready;
    }
    
  private:
    state* _state;
    bool _ready;
    bool _has_value;
    mutable boost::mutex _mutex;
    boost::condition_variable _condition;
    boost::signals2::connection _connection;
    
    T getValue();  // Implemented below after state is defined
  };
  
  // ----------
  // cvc::state
  // ----------
  // Purpose: 
  //   Central program state manangement.  Provides a tree
  //   to which property values and arbitrary data can be attached.
  //   Written to be thread safe and to be used also as a thread
  //   messaging system.  With xmlrpc that messaging can extend to
  //   threads in other processes and nodes on the network.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.
  // 03/02/2012 -- Joe R. -- Added touch()
  // 03/15/2012 -- Joe R. -- Added initialized flag.
  // 03/16/2012 -- Joe R. -- Added reset(), ptree() and traverse()
  // 03/30/2012 -- Joe R. -- Added comment and hidden field.
  // 03/31/2012 -- Joe R. -- Added dataTypeName().
  // 01/12/2014 -- Joe R. -- Added init_funcs and json()
  // 01/13/2014 -- Joe R. -- Removing notifyXmlRpc() once and for all.
  // 12/08/2025 -- Added futures API for async value retrieval.
  class state
  {
  public:  
    typedef boost::shared_ptr<state> state_ptr;
    typedef std::map<std::string,state_ptr> child_map;
    typedef boost::function<void (std::string)> traversal_unary_func;
    typedef boost::function<void ()> nullary_func;
    typedef std::vector<nullary_func> init_func_vec;

    static const std::string SEPARATOR;
    static init_func_vec _startup;

    virtual ~state();

    // ***** Main API

    //Use instance() to grab a reference to the singleton application object.
    static state& instance();

    const std::string& name()   const { return _name;   }
    const state*       parent() const { return _parent; }

    //return's the parent's fullName
    std::string parentName() const { 
      std::string tmp;
      return parent() ? 
        (!parent()->name().empty() ?
         ((tmp=parent()->parentName()).empty()?
          parent()->name() : 
          tmp + SEPARATOR + parent()->name()) 
         : "")
        : "";
    }

    std::string fullName() const { 
      std::string pn = parentName();
      return pn.empty() ?
        name() :
        pn + SEPARATOR + name();
    }

    boost::posix_time::ptime lastMod();

    std::string value();
    std::string valueTypeName();
    std::vector<std::string> values(bool unique = false); //shortcut for comma separated values in value()
    state& value(const std::string& v, bool setValueType = true);
    
    // Get value with optional callback that fires when value changes
    template <class T> 
    T value(const boost::function<void(T)>& callback = boost::function<void(T)>()) { 
      if (callback) {
        // Connect callback to valueChanged signal
        valueChanged.connect([this, callback]() {
          try {
            T val = boost::lexical_cast<T>(value());
            callback(val);
          } catch (...) {
            // Silently ignore conversion errors in callback
          }
        });
      }
      return boost::lexical_cast<T>(value()); 
    }
    
    template <class T> state& value(const T& v) {
      std::string str_value = boost::lexical_cast<std::string>(v);
      
      // Check if read-only before acquiring lock
      std::string full_name = fullName();
      
      {
        boost::mutex::scoped_lock lock(_mutex);
        
        // Check if this state is read-only
        if(_readOnly) {
          throw read_only_error(
            boost::str(boost::format("Cannot modify read-only state: %1%") % full_name)
          );
        }
        
        if(_value == str_value) return *this; //do nothing if equal
        
        _valueTypeName = cvcapp.dataTypeName<T>();
        _value = str_value;
        _lastMod = boost::posix_time::microsec_clock::universal_time();
        _initialized = true;
        // Notify any threads waiting for value
        _valueCondition.notify_all();
      }
      
      valueChanged();
      if(parent()) parent()->childChanged(full_name);
      return *this;
    }
    
    // Get a future that blocks until value is set
    template <class T>
    state_future<T> value_future() {
      return state_future<T>(this);
    }
    
    // Wait for value to be initialized, then return it
    template <class T>
    T wait_for_value(const boost::chrono::milliseconds& timeout = boost::chrono::milliseconds(0)) {
      if (timeout.count() == 0) {
        // Wait indefinitely
        boost::unique_lock<boost::mutex> lock(_mutex);
        while (!_initialized) {
          _valueCondition.wait(lock);
        }
        return boost::lexical_cast<T>(_value);
      } else {
        // Wait with timeout
        boost::unique_lock<boost::mutex> lock(_mutex);
        if (!_valueCondition.wait_for(lock, timeout, [this]() { return _initialized; })) {
          throw timeout_error("Timeout waiting for state value to be initialized");
        }
        return boost::lexical_cast<T>(_value);
      }
    }
    
    signal valueChanged;

    boost::any data();
    state& data(const boost::any&);

    template<class T>
    T data()
    {
      try {
        return boost::any_cast<T>(data());
      } catch (const boost::bad_any_cast& e) {
        throw type_conversion_error(
          boost::str(boost::format("Failed to cast data to requested type: %1%") % e.what())
        );
      }
    }
    
    // Get data with optional callback that fires when data changes
    template<class T>
    T data(const boost::function<void(T)>& callback)
    {
      if (callback) {
        // Connect callback to dataChanged signal
        dataChanged.connect([this, callback]() {
          try {
            T val = boost::any_cast<T>(data());
            callback(val);
          } catch (...) {
            // Silently ignore cast errors in callback
          }
        });
      }
      return boost::any_cast<T>(data());
    }
    
    // Wait for data to be set, then return it
    template<class T>
    T wait_for_data(const boost::chrono::milliseconds& timeout = boost::chrono::milliseconds(0)) {
      if (timeout.count() == 0) {
        // Wait indefinitely
        boost::unique_lock<boost::mutex> lock(_mutex);
        while (_data.empty()) {
          _dataCondition.wait(lock);
        }
        return boost::any_cast<T>(_data);
      } else {
        // Wait with timeout
        boost::unique_lock<boost::mutex> lock(_mutex);
        if (!_dataCondition.wait_for(lock, timeout, [this]() { return !_data.empty(); })) {
          throw timeout_error("Timeout waiting for state data to be set");
        }
        return boost::any_cast<T>(_data);
      }
    }

    template<class T>
    bool isData()
    {
      try
        {
          T val = data<T>();
        }
      catch(...)
        {
          return false;
        }
      return true;
    }

    std::string dataTypeName();

    signal dataChanged;

    state& operator()(const std::string& childname = std::string());
    std::vector<std::string> children(const std::string& re = std::string());
    size_t numChildren();
    map_change_signal childChanged;

    operator std::string(){ return value(); }
    
    signal destroyed;

    void touch();

    bool initialized() const { return _initialized; }

    //like propertyData from CVC::App
    template<class T>
    std::vector<T> valueData(bool uniqueElements = false)
    {
      using namespace std;
      using namespace boost;
      using namespace boost::algorithm;
      vector<string> vals = values(uniqueElements);
      vector<T> ret_data;
      BOOST_FOREACH(string dkey, vals)
        {
          trim(dkey);
          if(CVC_NAMESPACE::state::instance()(dkey).isData<T>())
            ret_data.push_back(CVC_NAMESPACE::state::instance()(dkey).data<T>());
        }
      return ret_data;
    }

    void reset();

    //converting to and from a boost property tree.  Useful for saving and restoring state.
    boost::property_tree::ptree ptree();
    operator boost::property_tree::ptree(){ return ptree(); }
    void ptree(const boost::property_tree::ptree&);

    //returns a json version of the property map
    std::string json();

    //sets this property tree based on a json
    void json(const std::string& j);

    void save(const std::string& filename);
    void restore(const std::string& filename);

    void traverse(traversal_unary_func func, const std::string& re = std::string());
    signal traverseEnter;
    signal traverseExit;

    std::string comment();
    state& comment(const std::string& c);
    signal commentChanged;

    bool hidden();
    state& hidden(bool h);
    signal hiddenChanged;

    bool readOnly();
    state& readOnly(bool ro);
    signal readOnlyChanged;

    static void on_startup(const nullary_func& init_func);

  protected:
    state(const std::string& n = std::string(),
          const state* p = NULL);

    void notifyParent(const std::string& childname);

    boost::mutex                     _mutex;
    boost::posix_time::ptime         _lastMod;

    std::string                      _name;
    const state*                     _parent;

    std::string                      _value;
    std::string                      _valueTypeName;
    boost::any                       _data;
    std::string                      _comment;
    bool                             _hidden;
    bool                             _readOnly;
    child_map                        _children;

    bool                             _initialized;
    
    // Condition variables for futures/blocking operations
    boost::condition_variable        _valueCondition;
    boost::condition_variable        _dataCondition;
    
    static state_ptr                 instancePtr();
    static state_ptr                 _instance;
    static boost::mutex              _instanceMutex;
  private:
    state(const state&);
  };
  
  // Template implementations for state_future
  template<typename T>
  state_future<T>::state_future(state* s) 
    : _state(s), _ready(false), _has_value(false) 
  {
    // Connect to valueChanged signal
    _connection = _state->valueChanged.connect([this]() {
      boost::mutex::scoped_lock lock(_mutex);
      _ready = true;
      _has_value = true;
      _condition.notify_all();
    });
  }
  
  template<typename T>
  T state_future<T>::getValue() {
    return _state->template value<T>();
  }
}

//Shorthand to access the cvc::state object from anywhere
#define cvcstate CVC_NAMESPACE::state::instance()

#endif

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

#include <cvc/state.h>
#include <cvc/app.h>
#include <cvc/utility.h>
#include <cvc/exception.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/current_function.hpp>
#include <boost/regex.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <set>
#include <sstream>

namespace CVC_NAMESPACE
{
  const std::string state::SEPARATOR(".");
  state::init_func_vec state::_startup;
  state::state_ptr state::_instance;
  boost::mutex state::_instanceMutex;

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
  state::state(const std::string& n, const state* p) :
    _name(n),
    _parent(p),
    _lastMod(boost::posix_time::min_date_time),
    _hidden(false),
    _readOnly(false),
    _initialized(false)
  {
    //This slot propagates child changes up to parents
    childChanged.connect(
      map_change_signal::slot_type(
        &state::notifyParent, this, boost::placeholders::_1
      )
    );
  }

  // -------------
  // state::~state
  // -------------
  // Purpose: 
  //   Destructor.  Just signals that this object has been destroyed.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.
  state::~state()
  {
    destroyed();
  }

  // ------------------
  // state::instancePtr
  // ------------------
  // Purpose: 
  //   Returns a pointer to the root state object singleton. Currently
  //   stores the root state object on the cvcapp datamap, though this
  //   might not always be the case.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.  
  // 01/12/2014 -- Joe R. -- Added startup function calls to do initialization based on cvcstate.
  //                         Also moved xmlrpc server thread start elsewhere.
  state::state_ptr state::instancePtr()
  {
    bool do_startup = false;
    {
      boost::mutex::scoped_lock lock(_instanceMutex);

      if(!_instance)
	{
	  //Keep our static instance in the data map!
	  const std::string statekey("__state");
	  state_ptr ptr(new state);
	  cvcapp.data(statekey,ptr);
	  
	  _instance = cvcapp.data<state_ptr>(statekey);
	  do_startup = true;
	}
    }

    if(do_startup)
      {
	BOOST_FOREACH(nullary_func& init_func, _startup)
	  {
	    init_func();
	  }
      }

    return _instance;
  }

  // ---------------
  // state::instance
  // ---------------
  // Purpose: 
  //   Returns a reference to the singleton root object.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.  
  state& state::instance()
  {
    return *instancePtr();
  }

  // --------------
  // state::lastMod
  // --------------
  // Purpose: 
  //   Returns the time this object was last modified.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.  
  boost::posix_time::ptime state::lastMod()
  {
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
  std::string state::value()
  {
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
  std::string state::valueTypeName()
  {
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
  std::string state::comment()
  {
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
  state& state::comment(const std::string& c)
  {
    boost::this_thread::interruption_point();
    if(comment() == c) return *this; //do nothing if equal
    
    {
      boost::mutex::scoped_lock lock(_mutex);
      _comment = c;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;      
    }

    commentChanged();
    if(parent()) parent()->childChanged(name());
    return *this;
  }

  // -------------
  // state::hidden
  // -------------
  // Purpose: 
  //   Returns the hidden flag for this object.
  // ---- Change History ----
  // 03/30/2012 -- Joe R. -- Creation.  
  bool state::hidden()
  {
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
  state& state::hidden(bool h)
  {
    boost::this_thread::interruption_point();
    if(hidden() == h) return *this; //do nothing if equal

    {
      boost::mutex::scoped_lock lock(_mutex);
      _hidden = h;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;      
    }

    hiddenChanged();
    if(parent()) parent()->childChanged(name());
    return *this;
  }

  // ---------------
  // state::readOnly
  // ---------------
  // Purpose: 
  //   Returns the read-only flag for this state object.
  // ---- Change History ----
  // 12/30/2025 -- Joe R. -- Creation.  
  bool state::readOnly()
  {
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
  state& state::readOnly(bool ro)
  {
    boost::this_thread::interruption_point();
    if(readOnly() == ro) return *this; //do nothing if equal

    {
      boost::mutex::scoped_lock lock(_mutex);
      _readOnly = ro;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;      
    }

    readOnlyChanged();
    if(parent()) parent()->childChanged(name());
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
  std::vector<std::string> state::values(bool unique)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_mutex);
    
    using namespace std;
    using namespace boost;
    using namespace boost::algorithm;

    vector<string> vals;
    if(_value.empty()) return vals;

    string valstr = _value;
    split(vals,valstr,is_any_of(","));
    BOOST_FOREACH(string& val, vals) trim(val);
    if(unique)
      {
        set<string> vals_set;
        copy(vals.begin(), vals.end(), 
             inserter(vals_set, vals_set.begin()));
        vals.resize(vals_set.size());
        copy(vals_set.begin(), vals_set.end(),
             vals.begin());
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
  state& state::value(const std::string& v, bool setValueType)
  {
    boost::this_thread::interruption_point();
    
    // Get fullName before locking
    std::string full_name = fullName();
    
    // Check if this state is read-only
    {
      boost::mutex::scoped_lock lock(_mutex);
      if(_readOnly) {
        throw read_only_error(
          boost::str(boost::format("Cannot modify read-only state: %1%") % full_name)
        );
      }
    }
    
    if(value() == v) return *this; //do nothing if equal

    {
      boost::mutex::scoped_lock lock(_mutex);
      _value = v;
      _lastMod = boost::posix_time::microsec_clock::universal_time();
      _initialized = true;

      if(setValueType)
        _valueTypeName = std::string("std::string");
      
      // Notify any threads waiting for value
      _valueCondition.notify_all();
    }

    valueChanged();
    if(parent()) parent()->childChanged(full_name);
    return *this;
  }

  // ------------
  // state::touch
  // ------------
  // Purpose: 
  //   Triggers signals as if this state obj changed.
  // ---- Change History ----
  // 03/02/2012 -- Joe R. -- Creation.
  void state::touch()
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_mutex);
      _lastMod = boost::posix_time::microsec_clock::universal_time();
    }
    valueChanged();
    dataChanged();
    if(parent()) parent()->childChanged(name());
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
  void state::reset()
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_mutex);
      _value = std::string();
      _valueTypeName = std::string();
      _data = boost::any();
      _comment = std::string();
      _hidden = false;
      _initialized = false;
      BOOST_FOREACH(child_map::value_type val, _children)
        val.second->reset();
    }
    touch();
  }

  // ------------
  // state::ptree
  // ------------
  // Purpose: 
  //   Returns a property tree describing this state object 
  //   and it's children.  Only stores string values, not 'data'.
  // ---- Change History ----
  // 03/16/2012 -- Joe R. -- Creation.
  boost::property_tree::ptree state::ptree()
  {
    using namespace boost;
    property_tree::ptree pt;

    boost::this_thread::interruption_point();
    cvcapp.log(6,boost::str(boost::format("%s :: %s = %s\n")
                            % BOOST_CURRENT_FUNCTION
                            % fullName()
                            % value()));

    if(!value().empty())
      pt.push_back(property_tree::ptree::value_type(fullName(),
						    property_tree::ptree(value())));
    
    {
      boost::mutex::scoped_lock lock(_mutex);
      BOOST_FOREACH(child_map::value_type val, _children)
        {
          boost::this_thread::interruption_point();
          property_tree::ptree child_pt = val.second->ptree();
          pt.push_back(property_tree::ptree::value_type(val.second->fullName(), 
                                                        child_pt));
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
  void state::ptree(const boost::property_tree::ptree& pt)
  {
    using namespace boost;
    BOOST_FOREACH(const property_tree::ptree::value_type &v, pt)
      (*this)(v.first).value(v.second.get_value<std::string>());
  }  

  // -----------
  // state::json
  // -----------
  // Purpose: 
  //  Returns a json string version of the property map.
  // ---- Change History ----
  // 01/12/2014 -- Joe R. -- Creation.
  std::string state::json()
  {
    return CVC_NAMESPACE::json(ptree());
  }

  // -----------
  // state::json
  // -----------
  // Purpose: 
  //  Sets this state object and its children based on an incoming json.
  // ---- Change History ----
  // 01/13/2014 -- Joe R. -- Creation.
  void state::json(const std::string& j)
  {
    ptree(CVC_NAMESPACE::json(j));
  }

  // -----------
  // state::save
  // -----------
  // Purpose: 
  //  Saves this state object and its children to the specified filename.
  // ---- Change History ----
  // 03/16/2012 -- Joe R. -- Creation.
  // 01/12/2014 -- Joe R. -- Forcing json.
  void state::save(const std::string& filename)
  {
    write_json(filename, ptree());
  }

  // --------------
  // state::restore
  // --------------
  // Purpose: 
  //  Restores this state object and its children from the specified filename.
  // ---- Change History ----
  // 03/16/2012 -- Joe R. -- Creation.
  // 01/12/2014 -- Joe R. -- Forcing json.
  void state::restore(const std::string& filename)
  {
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
  void state::traverse(traversal_unary_func func, const std::string& re)
  {
    traverseEnter();
    func(fullName());
    std::vector<std::string> ch = children(re);
    BOOST_FOREACH(std::string c, ch)
      cvcstate(c).traverse(func,re);
    traverseExit();
  }
  
  // -----------
  // state::data
  // -----------
  // Purpose: 
  //   Returns the data of this object.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.  
  boost::any state::data()
  {
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
  state& state::data(const boost::any& d)
  {
    boost::this_thread::interruption_point();
    
    // Get fullName before locking
    std::string full_name = fullName();
    
    // Check if this state is read-only
    {
      boost::mutex::scoped_lock lock(_mutex);
      if(_readOnly) {
        throw read_only_error(
          boost::str(boost::format("Cannot modify read-only state: %1%") % full_name)
        );
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
    if(parent()) parent()->childChanged(full_name);
    return *this;
  }

  // -------------------
  // state::dataTypeName
  // -------------------
  // Purpose: 
  //   Returns a string representing the type of the data.
  // ---- Change History ----
  // 03/31/2012 -- Joe R. -- Creation.
  std::string state::dataTypeName()
  {
    return cvcapp.dataTypeName(data());
  }

  // -----------------
  // state::operator()
  // -----------------
  // Purpose: 
  //   Used for child object lookups.
  // ---- Change History ----
  // 02/18/2012 -- Joe R. -- Creation.
  // 03/15/2012 -- Joe R. -- Added initialized flag.
  state& state::operator()(const std::string& childname)
  {
    using namespace std;
    using namespace boost::algorithm;

    boost::this_thread::interruption_point();

    vector<string> keys;
    split(keys, childname, is_any_of(SEPARATOR));
    if(keys.empty()) return *this;
    BOOST_FOREACH(string& key, keys) trim(key);
    //Ignore beginning empty keys
    while(!keys.empty() && keys.front().empty())
      keys.erase(keys.begin());
    if(keys.empty()) return *this;

    string nearest = keys.front();
    keys.erase(keys.begin());
    {
      boost::mutex::scoped_lock lock(_mutex);
      //If we have the child state in our map, take out its part of the
      //keys vector and recursively call its operator().
      //If not, create a new one.
      if(_children.find(nearest)!=_children.end() &&
         _children[nearest])
        return (*_children[nearest])(join(keys,SEPARATOR));
      else
        {
          state_ptr s(new state(nearest,this));
          _children[nearest] = s;
          _lastMod = boost::posix_time::microsec_clock::universal_time();
          _initialized = true;
          return (*_children[nearest])(join(keys,SEPARATOR));
        }
    }
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
  std::vector<std::string> state::children(const std::string& re)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_mutex);
    std::vector<std::string> ret;
    BOOST_FOREACH(child_map::value_type val, _children)
      {
        if(!re.empty())
          {
	    try
	      {
		boost::regex expression(re.c_str());
		boost::cmatch what;
		
		cvcapp.log(6,boost::str(boost::format("%s :: check match %s\n")
					% BOOST_CURRENT_FUNCTION
					% val.second->fullName()));
		
		if(boost::regex_match(val.second->fullName().c_str(),what,expression))
		  {
		    cvcapp.log(6,boost::str(boost::format("%s :: matched! %s\n")
					    % BOOST_CURRENT_FUNCTION
					    % val.second->fullName()));
		    
		    ret.push_back(val.second->fullName());
		  }
	      }
	    catch(boost::bad_expression&)
	      {
		cvcapp.log(2,boost::str(boost::format("%s :: invalid regex '%s'\n")
					% BOOST_CURRENT_FUNCTION
					% re));
		return ret;
	      }
          }
        else
          ret.push_back(val.second->fullName());

        //Get any matches from this state's children if any.
        std::vector<std::string> childret = 
          val.second->children(re);
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
  size_t state::numChildren()
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_mutex);
    return _children.size();
  }

  // -----------------
  // state::on_startup
  // -----------------
  // Purpose: 
  //   Add to the list of functions to call when first initializing cvcstate.
  // ---- Change History ----
  // 01/12/2014 -- Joe R. -- Creation.
  void state::on_startup(const nullary_func& init_func)
  {
    _startup.push_back(init_func);
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
  void state::notifyParent(const std::string& childname)
  {
    boost::this_thread::interruption_point();
    if(parent()) parent()->childChanged(name() + SEPARATOR + childname);
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
  bool state::isValidStateName(const std::string& name)
  {
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
  std::string state::sanitizeStateName(const std::string& name)
  {
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
}

namespace
{
  // -----------
  // system_init
  // -----------
  // Purpose: 
  //   Sets the default system settings and initial info.
  // ---- Change History ----
  // 01/13/2014 -- Joe R. -- Creation.  
  class system_init
  {
  public:
    static void init()
    {
      using namespace boost::posix_time;
      cvcstate("__system.start").value(to_simple_string(microsec_clock::universal_time()));
    }

    system_init()
    {
      CVC_NAMESPACE::state::on_startup(init);
    }
  } static_init;
}

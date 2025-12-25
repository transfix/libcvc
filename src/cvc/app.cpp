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

#include <cvc/app.h>
#include <cvc/types.h>
#include <cvc/bounding_box.h>
#include <cvc/dimension.h>
#include <cvc/state.h>
#include <cvc/utility.h>

#ifdef USING_LOG4CPLUS_DEFAULT
#include <log4cplus/logger.h>
#include <log4cplus/fileappender.h>
#include <log4cplus/loglevel.h>
#include <log4cplus/configurator.h>
#endif

#ifndef CVC_APP_XML_PROPERTY_MAP
#include <boost/property_tree/info_parser.hpp>
#else
#include <boost/property_tree/xml_parser.hpp>
#endif

#include <boost/shared_array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/xtime.hpp>

#include <iostream>
#include <fstream>
#include <set>
#include <queue>
#include <algorithm>
#include <iterator>
#include <cstdlib>
#include <cmath>

namespace CVC_NAMESPACE
{
  app::app_ptr app::_instance;
  boost::mutex app::_instanceMutex;

  // 07/15/2011 -- Joe R. -- Moved data type registration here
  // 07/22/2011 -- Joe R. -- Added Char enum
  // 09/09/2011 -- Joe R. -- Added Int and Int64 enum
  // 03/30/3012 -- Joe R. -- Registering bool.
  // 04/06/2012 -- Joe R. -- Registering some boost::shared_array types
  // 05/11/2012 -- Joe R. -- Adding state.
  app::app_ptr app::instancePtr()
  {
    boost::mutex::scoped_lock lock(_instanceMutex);
    if(!_instance)
      {
        _instance.reset(new app);
        
        //Register some primitive types with the system
        _instance->registerDataType(char);
        _instance->registerDataType<char>(Char);
        _instance->registerDataType(unsigned char);
        _instance->registerDataType<unsigned char>(UChar);
        _instance->registerDataType(unsigned short);
        _instance->registerDataType<unsigned short>(UShort);
        _instance->registerDataType(int);
        _instance->registerDataType<int>(Int);
        _instance->registerDataType(unsigned int);
        _instance->registerDataType<unsigned int>(UInt);
        _instance->registerDataType(float);
        _instance->registerDataType<float>(Float);
        _instance->registerDataType(double);
        _instance->registerDataType<double>(Double);
        _instance->registerDataType(int64);
        _instance->registerDataType<int64>(Int64);
        _instance->registerDataType(uint64);
        _instance->registerDataType<uint64>(UInt64);
        _instance->registerDataType(std::string);
        _instance->registerDataType(bounding_box);
        _instance->registerDataType(dimension);
        _instance->registerDataType(bool);
        _instance->registerDataType(boost::shared_array<unsigned char>);
        _instance->registerDataType(boost::shared_array<float>);
        _instance->registerDataType(boost::shared_array<double>);
        _instance->registerDataType(state);

        //Register a call to wait for all child threads to finish before exiting
        //the main thread.
        std::atexit(wait_for_threads);

#ifdef USING_LOG4CPLUS_DEFAULT
        // Initialize log4cplus
        std::ifstream testfile("log4cplus.properties");
        if (testfile)
          {
            testfile.close();
            log4cplus::PropertyConfigurator::doConfigure("log4cplus.properties");
          }
        else 
          {
            log4cplus::BasicConfigurator::doConfigure();
          }
        static log4cplus::Logger logger = log4cplus::Logger::getInstance("app");
        LOG4CPLUS_ERROR(logger, "log4cplus initialized");
 #endif
      }
    return _instance;
  }

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
  void app::wait_for_threads()
  {
    using namespace CVC_NAMESPACE;

    //Get all the threads
    thread_map map = cvcapp.threads();
    
    // Phase 1: Interrupt all threads to signal them to exit
    // Many algorithms have interruption checkpoints that allow clean exit
    BOOST_FOREACH(thread_map::value_type val, map)
      {
        try
          {
            if (val.second)
              val.second->interrupt();
          }
        catch(...) {}
      }
    
    // Phase 2: Attempt to join all threads with a timeout
    std::vector<std::string> failed_threads;
    BOOST_FOREACH(thread_map::value_type val, map)
      {
        try
          {
            using namespace boost;
            if (val.second && val.second->joinable()) {
              cvcapp.log(3,str(format("%s :: waiting for thread %s\n")
                               % BOOST_CURRENT_FUNCTION
                               % val.first));
              // Try to join with a 5 second timeout per thread
              if (!val.second->try_join_for(boost::chrono::seconds(5))) {
                // Thread didn't join in time, track it
                failed_threads.push_back(val.first);
              }
            }
          }
        catch(boost::thread_interrupted&) {}
        catch(...) {}
      }
    
    // Phase 3: Report any threads that failed to join
    if (!failed_threads.empty()) {
      using namespace boost;
      std::string msg = str(format("%s :: WARNING: %d thread(s) failed to join within timeout and may cause cleanup issues:\n")
                            % BOOST_CURRENT_FUNCTION
                            % failed_threads.size());
      BOOST_FOREACH(const std::string& thread_name, failed_threads) {
        msg += str(format("  - %s\n") % thread_name);
      }
      cvcapp.log(0, msg); // Error level
      std::cerr << msg; // Also print to stderr to ensure visibility
    }
  }

  app& app::instance()
  {
    app& app = *instancePtr();
    return app;
  }

  app::app() 
    : _maxPoolSize(boost::thread::hardware_concurrency() > 0 ? 
                   boost::thread::hardware_concurrency() : 4),
      _activeWorkers(0)
  {
  }

  app::~app()
  {
  }
  
  data_map app::data()
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataMutex);
    //Return a copy of the datamap.  Objects in the datamap
    //should be copyable without incuring much overhead, like
    //for instance objects that use a copy-on-write pattern for
    //large arrays internal to an object.
    return _data; 
  }

  boost::any app::data(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataMutex);
    if(_data.find(key)!=_data.end())
      return _data[key];
    return boost::any();
  }

  void app::data(const std::string& key, const boost::any& value)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_dataMutex);
      if(value.empty())
        _data.erase(key); //remove if empty
      else if(!key.empty()) //don't allow empty keys
        _data[key] = value;
    }
    dataChanged(key);
  }

  void app::data(const data_map& map)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_dataMutex);
      _data = map;
      //erase empty data
      std::list<std::string> emptyProps;
      BOOST_FOREACH(data_map::value_type val, _data)
        if(val.first.empty() || val.second.empty()) 
          emptyProps.push_back(val.first);
      BOOST_FOREACH(std::string key, emptyProps)
        _data.erase(key);
    
    }
    dataChanged("all");
  }

  std::string app::dataTypeName(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataMutex);
    std::string retval;
    if(_data.find(key)!=_data.end())
      {
        std::string rawname = _data[key].type().name();
        if(_dataTypeNames.find(rawname)!=_dataTypeNames.end())
          retval = _dataTypeNames[rawname];
        else
          retval = rawname;
      }      
    
    return retval;
  }

  std::string app::dataTypeName(const boost::any& d)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataMutex);
    std::string retval;
    std::string rawname = d.type().name();
    if(_dataTypeNames.find(rawname)!=_dataTypeNames.end())
      retval = _dataTypeNames[rawname];
    else
      retval = rawname;
    return retval;
  }

  data_type app::dataType(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataMutex);
    data_type retval = Undefined;
    if(_data.find(key)!=_data.end())
      {
        std::string rawname = _data[key].type().name();
        if(_dataTypeEnum.find(rawname)!=_dataTypeEnum.end())
          retval = _dataTypeEnum[rawname];
      }      
    return retval;
  }

  std::vector<std::string> app::listify(const std::string& keylist)
  {
    using namespace std;
    using namespace boost;
    using namespace boost::algorithm;
    string separators = properties("system.list_separators");
    if(separators.empty()) separators=",";
    vector<string> keys;
    split(keys, keylist, is_any_of(separators));
    BOOST_FOREACH(string& key, keys) trim(key);
    return keys;
  }

  std::string app::listify(const std::vector<std::string>& keys)
  {
    using namespace std;
    using namespace boost;
    using namespace boost::algorithm;
    string separators = properties("system.list_separators");
    if(separators.empty()) separators=",";
    separators.resize(1); //just use the first char
    vector<string> localKeys = keys;
    BOOST_FOREACH(string& key, localKeys) trim(key);
    string listkey = join(localKeys,separators);
    return listkey;
  }

  data_reader_collection app::dataReaders()
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataReadersMutex);
    return _dataReaders;
  }

  void app::dataReaders(const data_reader_collection& dlc)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataReadersMutex);
    _dataReaders = dlc;
  }

  data_reader app::dataReader(data_reader_collection::size_type idx)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataReadersMutex);
    return _dataReaders[idx];
  }

  void app::dataReader(const data_reader& dl)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataReadersMutex);
    _dataReaders.push_back(dl);
  }

  bool app::readData(const std::string& path)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_dataReadersMutex);
    BOOST_FOREACH(data_reader dl, _dataReaders)
      if(dl(path)) return true;
    return false;
  }

  property_map app::properties()
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_propertiesMutex);
    return _properties;
  }

  std::string app::properties(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_propertiesMutex);
    if(_properties.find(key)!=_properties.end())
      return _properties[key];
    return std::string();
  }

  void app::properties(const std::string& key, const std::string& val)
  {
    boost::this_thread::interruption_point();
    bool propChanged = true;
    {
      boost::mutex::scoped_lock lock(_propertiesMutex);
      if(val.empty()) //remove if empty
        _properties.erase(key);
      else if(_properties[key] == val) //if no change, no signal
        propChanged = false;
      else if(!key.empty()) //don't allow empty keys
        _properties[key] = val;
    }
    if(propChanged) propertiesChanged(key);
  }

  void app::properties(const property_map& map)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_propertiesMutex);
      _properties = map;
      //erase empty properties
      std::list<std::string> emptyProps;
      BOOST_FOREACH(property_map::value_type val, _properties)
        if(val.first.empty() || val.second.empty()) 
          emptyProps.push_back(val.first);
      BOOST_FOREACH(std::string key, emptyProps)
        _properties.erase(key);
    }
    propertiesChanged("all");
  }

  void app::addProperties(const property_map& map)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_propertiesMutex);
      BOOST_FOREACH(property_map::value_type val, map)
        if(!val.first.empty() && !val.second.empty())
          _properties[val.first] = val.second;
    }
    propertiesChanged("all");
  }

  bool app::hasProperty(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_propertiesMutex);
    if(_properties.find(key)!=_properties.end())
      return true;
    return false;
  }

  std::vector<std::string> app::listProperty(const std::string& key,
                                             bool uniqueElements)
  {
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
    if(separators.empty()) separators=",";
    split(vals, valstring, is_any_of(separators));
    BOOST_FOREACH(string& val, vals) trim(val);
    if(uniqueElements)
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


  // arand, 5-3-2011: two new functions for managing property lists below
  void app::listPropertyAppend(const std::string& key, const std::string& val) {    
    std::vector<std::string> listProp = listProperty(key,true);
    //string thumbGeoNames1;
    
    for (int i=0; i<listProp.size(); i++) {
      if (val.compare(listProp[i]) == 0) {
        // its already in the list... just return
        return;
      }
    }
    listProp.push_back(val);
    properties(key, listify(listProp));
  }

  void app::listPropertyRemove(const std::string& key, const std::string& val) {
    std::vector<std::string> listProp = listProperty(key,true);
    //string thumbGeoNames1;
    
    bool found = false;
    for (int i=0; i<listProp.size(); i++) {
      if (val.compare(listProp[i]) == 0) {
        listProp.erase(listProp.begin()+i);
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
  void app::sleep(double ms)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    boost::xtime xt;
    boost::xtime_get( &xt, boost::TIME_UTC_ );
    xt.nsec += uint64(ms * std::pow(10.0,6.0));
    boost::thread::sleep( xt );
  }

  // 12/16/2011 -- transfix -- Creation.
  void app::propertyTreeTraverse(const boost::property_tree::ptree& pt,
                                 const std::string& parentkey)
  {
    using namespace boost;
    using boost::property_tree::ptree;

    std::vector<std::string> filesToRead;

    BOOST_FOREACH(const ptree::value_type &v, pt)
      {
        std::string childkey = v.first;
        std::string fullkey = 
          parentkey.empty() ? childkey : 
          parentkey + "." + childkey;

        properties(fullkey, v.second.get_value<std::string>());
        propertyTreeTraverse(v.second,fullkey);
      }
  }

  // 12/16/2011 -- transfix -- Creation.
  void app::readPropertyMap(const std::string& path)
  {
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
  void app::writePropertyMap(const std::string& path)
  {
    using boost::property_tree::ptree;
    ptree pt;
    property_map localmap = properties();
    BOOST_FOREACH(const property_map::value_type &v, localmap)
      pt.put(v.first, v.second);
#ifndef CVC_APP_XML_PROPERTY_MAP
    write_info(path, pt);
#else
    write_xml(path, pt);
#endif
  }

  thread_map app::threads()
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_threadsMutex);
    return _threads;
  }

  thread_ptr app::threads(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_threadsMutex);
    if(_threads.find(key)!=_threads.end())
      return _threads[key];
    return thread_ptr();
  }

  //01/13/2014 -- Joe R. -- interrupting the currently running thread with this key
  void app::threads(const std::string& key, const thread_ptr& val)
  {
    boost::this_thread::interruption_point();

    if(hasThread(key))
      threads(key)->interrupt();

    {
      boost::mutex::scoped_lock lock(_threadsMutex);
      if(!val)
        _threads.erase(key);
      else
        _threads[key] = val;
      updateThreadKeys();
    }

    threadsChanged(key);
  }

  //01/13/2014 -- Joe R. -- interrupting all the threads in the map
  void app::threads(const thread_map& map)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_threadsMutex);
      
      BOOST_FOREACH(thread_map::value_type t, _threads)
	if(t.second) t.second->interrupt();

      _threads = map;
      _threadProgress.clear();
      updateThreadKeys();
    }
    threadsChanged("all");
  }

  bool app::hasThread(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_threadsMutex);
    if(_threads.find(key)!=_threads.end())
      return true;
    return false;
  }

  double app::threadProgress(const std::string& key)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_threadsMutex);

    boost::thread::id tid;
    if(key.empty())
      tid = boost::this_thread::get_id();
    else if(_threads.find(key)!=_threads.end() &&
            _threads[key])
      tid = _threads[key]->get_id();
    else
      return 0.0;
    
    return _threadProgress[tid];
  }

  void app::threadProgress(double progress)
  {
    threadProgress(std::string(),progress);
  }

  void app::threadProgress(const std::string& key, double progress)
  {
    boost::this_thread::interruption_point();

    //clamp progress to [0.0,1.0]
    progress = progress < 0.0 ? 
      0.0 : 
      progress > 1.0 ? 1.0 :
      progress;

    bool changed = false;
    {
      boost::mutex::scoped_lock lock(_threadsMutex);
      boost::thread::id tid;

      if(key.empty())
        {
          tid = boost::this_thread::get_id();
          changed = true;
        }
      else if(_threads.find(key)!=_threads.end() &&
              _threads[key])
        {
          tid = _threads[key]->get_id();
          changed = true;
        }

      if(changed)
        _threadProgress[tid]=progress;
    }

    if(changed)
      threadsChanged(key);
  }

  void app::finishThreadProgress(const std::string& key)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_threadsMutex);

      boost::thread::id tid;
      if(key.empty())
        tid = boost::this_thread::get_id();
      else if(_threads.find(key)!=_threads.end() &&
              _threads[key])
        tid = _threads[key]->get_id();
      else
        return;

      _threadProgress.erase(tid);
    }
    threadsChanged(key);
  }

  std::string app::threadKey()
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_threadsMutex);
      if(_threadKeys.find(boost::this_thread::get_id())!=_threadKeys.end())
        return _threadKeys[boost::this_thread::get_id()];
      else
        return std::string("unknown");
    }
  }

  void app::removeThread(const std::string& key)
  {
    threads(key,thread_ptr());
  }

  std::string app::uniqueThreadKey(const std::string& hint)
  {
    std::string h = hint.empty() ? "thread" : hint;
    //Make a unique key name to use by adding a number to the key
    std::string uniqueThreadKey = h;
    unsigned int i = 0;
    while(hasThread(uniqueThreadKey))
      uniqueThreadKey = 
        h + boost::lexical_cast<std::string>(i++);
    return uniqueThreadKey;
  }

  void app::threadInfo(const std::string& key, const std::string& infostr)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_threadsMutex);

      boost::thread::id tid;
      if(key.empty())
        tid = boost::this_thread::get_id();
      else if(_threads.find(key)!=_threads.end() &&
              _threads[key])
        tid = _threads[key]->get_id();
      else
        return;

      _threadInfo[tid] = infostr;
    }
    threadsChanged(key);
  }

  std::string app::threadInfo(const std::string& key)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_threadsMutex);
      
      boost::thread::id tid;
      if(key.empty())
        tid = boost::this_thread::get_id();
      else if(_threads.find(key)!=_threads.end() &&
              _threads[key])
        tid = _threads[key]->get_id();
      else
        return std::string();

      return _threadInfo[tid];
    }
  }

  //TODO: write a version of log that returns a stream to use with
  //stream operators.
  // 09/09/2011 -- Joe R. -- Removing references to cvcapp because it will crash
  //                         if you try to use them in ~App.
  void app::log(unsigned int verbosity_level, const std::string& buf)
  {
#ifdef USING_LOG4CPLUS_DEFAULT
    static log4cplus::Logger logger = log4cplus::Logger::getInstance("cvc.app.log");
    std::string msg = buf.substr(0, buf.length()-1); // take off trailing newline
    if (verbosity_level == 0) {// || verbosity_level == 1) {
      LOG4CPLUS_ERROR(logger, msg);
    }
    else if (verbosity_level == 1) {
      LOG4CPLUS_WARN(logger, msg);
    }
    else if (verbosity_level == 2) {
      LOG4CPLUS_INFO(logger, msg);
    }
    else if (verbosity_level == 3) {
      LOG4CPLUS_DEBUG(logger, msg);
    }
    else {
      LOG4CPLUS_TRACE(logger, msg);
    }
#else
    using namespace std;
    using namespace boost;
    using namespace boost::algorithm;
    boost::mutex::scoped_lock lock(_logMutex);
    unsigned int log_verbosity =
      hasProperty("system.log_verbosity") ?
      properties<unsigned int>("system.log_verbosity") :
      6;
    if(verbosity_level < log_verbosity)
      {
        string output_locs = properties("system.log_output");
        string log_prefix = properties("system.log_prefix");
        string log_postfix = properties("system.log_postfix");
        string output_string = log_prefix+buf+log_postfix;

        vector<string> key_idents;
        split(key_idents,output_locs,is_any_of(","));
        BOOST_FOREACH(string loc, key_idents)
          {
            trim(loc);
            if(loc == "stdout")
              {
                cout<<output_string;
                continue;
              }
            else if(loc == "stderr" || loc.empty())
              {
                cerr<<output_string;
                continue;
              }
            //check if loc is an std::string on the datamap
            bool isDataKey = false;
            std::vector<std::string> dataKeys = data<std::string>();
            BOOST_FOREACH(std::string key, dataKeys)
              if(loc == key) //if so, append buf to the log string
                {
                  data(key, data<std::string>(key) + output_string);
                  isDataKey = true;
                  break;
                }
    
            //if it was a data key, we're done
            if(isDataKey) continue;

            //if nothing else, interpret as a filename
            ofstream outfile(loc.c_str(),
                             ios_base::out|ios_base::app);
            outfile<<output_string;
          }
      }
#endif
  }

  //This should only be called after we lock the threads mutex
  // 07/29/2011 - Joe R. - handling thread info here too.
  void app::updateThreadKeys()
  {
    using namespace std;

    _threadKeys.clear();

    set<boost::thread::id> infoIds;
    BOOST_FOREACH(thread_info_map::value_type val, _threadInfo)
      infoIds.insert(val.first);

    set<boost::thread::id> currentIds;
    BOOST_FOREACH(thread_map::value_type val, _threads)
      {
        thread_ptr ptr = val.second;
        if(ptr)
          {
            _threadKeys[ptr->get_id()]=val.first;

            //set the thread info to a default state if not already set
            if(_threadInfo[ptr->get_id()].empty())
              _threadInfo[ptr->get_id()] = "running";
          }

        currentIds.insert(ptr->get_id());
      }

    //compute thread ids that need to be removed from the threadInfo map
    set<boost::thread::id> infoIdsToRemove;
    set_difference(infoIds.begin(), infoIds.end(),
                   currentIds.begin(), currentIds.end(),
                   inserter(infoIdsToRemove,infoIdsToRemove.begin()));

    BOOST_FOREACH(boost::thread::id tid, infoIdsToRemove)
      _threadInfo.erase(tid);
  }

  mutex_ptr app::mutex(const std::string& name)
  {
    bool changed = false;
    mutex_ptr ptr;
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_mutexMapMutex);
      if(!_mutexMap[name].get<0>())
        {
          _mutexMap[name].get<0>().reset(new boost::mutex);
          changed = true;
        }
      ptr = _mutexMap[name].get<0>();
    }

    if(changed) mutexesChanged(name);

    return ptr;
  }

  void app::mutexInfo(const std::string& name, const std::string& in)
  {
    boost::this_thread::interruption_point();
    {
      boost::mutex::scoped_lock lock(_mutexMapMutex);
      _mutexMap[name].get<1>() = in;
    }
    mutexesChanged(name);
  }

  std::string app::mutexInfo(const std::string& name)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_mutexMapMutex);
    return _mutexMap[name].get<1>();
  }

  // -------------------------
  // Thread Pool Implementation
  // -------------------------
  // 12/25/2025 -- Joe R. -- Creation.
  
  void app::setThreadPoolSize(unsigned int size)
  {
    boost::this_thread::interruption_point();
    boost::mutex::scoped_lock lock(_threadPoolMutex);
    _maxPoolSize = (size > 0) ? size : 1;
    
    // Try to start workers if we have pending tasks
    while (!_pendingTasks.empty() && _activeWorkers < _maxPoolSize)
    {
      tryStartWorker();
    }
  }

  unsigned int app::getThreadPoolSize() const
  {
    boost::mutex::scoped_lock lock(const_cast<boost::mutex&>(_threadPoolMutex));
    return _maxPoolSize;
  }

  unsigned int app::getActiveThreadCount() const
  {
    boost::mutex::scoped_lock lock(const_cast<boost::mutex&>(_threadPoolMutex));
    return _activeWorkers;
  }

  unsigned int app::getPendingThreadCount() const
  {
    boost::mutex::scoped_lock lock(const_cast<boost::mutex&>(_threadPoolMutex));
    return _pendingTasks.size();
  }

  void app::tryStartWorker()
  {
    // Assumes _threadPoolMutex is already locked by caller
    boost::this_thread::interruption_point();
    
    if (_pendingTasks.empty() || _activeWorkers >= _maxPoolSize)
      return;

    _activeWorkers++;
    
    // Start a worker thread (not tracked in thread_map, internal to pool)
    boost::thread worker(&app::threadPoolWorker, this);
    worker.detach(); // Let it run independently
  }

  void app::threadPoolWorker()
  {
    while (true)
    {
      ThreadPoolTask task;
      
      {
        boost::mutex::scoped_lock lock(_threadPoolMutex);
        
        // If no tasks, decrement worker count and exit
        if (_pendingTasks.empty())
        {
          _activeWorkers--;
          return;
        }
        
        // Get highest priority task
        task = _pendingTasks.top();
        _pendingTasks.pop();
      }
      
      // Execute the task with the user's key in the thread map
      try
      {
        threads(task.key, thread_ptr(new boost::thread(task.task)));
        
        // Wait for completion
        thread_ptr tptr = threads(task.key);
        if (tptr)
        {
          tptr->join();
        }
      }
      catch (boost::thread_interrupted&)
      {
        // Thread was interrupted, clean up and continue
      }
      catch (std::exception& e)
      {
        // Log error but continue processing
        log(0, std::string("Thread pool worker error: ") + e.what());
      }
      catch (...)
      {
        // Unknown error, log and continue
        log(0, "Thread pool worker encountered unknown error");
      }
      
      // After completing task, check if there are more pending tasks
      {
        boost::mutex::scoped_lock lock(_threadPoolMutex);
        
        if (_pendingTasks.empty())
        {
          _activeWorkers--;
          return;
        }
        // Otherwise, loop continues to process next task
      }
    }
  }
}

/*
  Copyright 2007-2011 The University of Texas at Austin

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

#ifndef __CVC_EXCEPTIONS_H__
#define __CVC_EXCEPTIONS_H__

#include <cvc/namespace.h>

#include <boost/format.hpp>
#include <boost/exception/exception.hpp>

#include <string>

namespace CVC_NAMESPACE
{
  /***** Exceptions ****/
  class exception : public boost::exception
  {
  public:
    exception() {}
    virtual ~exception() throw() {}
    virtual const std::string& what_str() const throw () = 0;
    virtual const char *what () const throw()
    {
      return what_str().c_str();
    }
  };
  
#define CVC_DEF_EXCEPTION(name) \
  class name : public CVC_NAMESPACE::exception \
  { \
  public: \
    name () : _msg("cvc::"#name) {} \
    name (const std::string& msg) : \
      _msg(boost::str(boost::format("cvc::" #name " exception: %1%") % msg)) {} \
    virtual ~name() throw() {} \
    virtual const std::string& what_str() const throw() { return _msg; } \
  private: \
    std::string _msg; \
  }

  CVC_DEF_EXCEPTION(read_error);
  CVC_DEF_EXCEPTION(write_error);
  CVC_DEF_EXCEPTION(memory_allocation_error);
  CVC_DEF_EXCEPTION(unsupported_exception);
  CVC_DEF_EXCEPTION(index_out_of_bounds);
  CVC_DEF_EXCEPTION(volume_properties_mismatch);
  CVC_DEF_EXCEPTION(volume_cache_directory_file_error);
  CVC_DEF_EXCEPTION(command_line_error);
  CVC_DEF_EXCEPTION(timeout_error);
};

#endif


/*
  Copyright (c): Xiaoyu Zhang (xiaoyu@csusm.edu)

  This file is part of sdf (signed distance function).

  sdf is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  sdf is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
// RawivPaser.h: interface for the RawivPaser class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RAWIVPASER_H__938E0E03_F02E_4298_B690_AD4DE7EF3AB4__INCLUDED_)
#define AFX_RAWIVPASER_H__938E0E03_F02E_4298_B690_AD4DE7EF3AB4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <string.h>

#include "Reg3Parser.h"

class RawivParser : public Reg3Parser  
{
public:
	RawivParser();
	virtual ~RawivParser();

	virtual bool parse(Reg3Data<float>* data, const char* fname);

	virtual bool write(const Reg3Data<float>& data, const char* fname);

private:
	bool isRawivFile(const char* fname) {
		int len = (int)strlen(fname);
		return (len > 6 && strcmp(fname+len-6, ".rawiv") == 0);
	}
};

#endif // !defined(AFX_RAWIVPASER_H__938E0E03_F02E_4298_B690_AD4DE7EF3AB4__INCLUDED_)

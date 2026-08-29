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
// Reg3Parser.h: interface for the Reg3Parser class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_REG3PARSER_H__AE88C884_6FE3_433E_B6D2_F84CD6E39724__INCLUDED_)
#define AFX_REG3PARSER_H__AE88C884_6FE3_433E_B6D2_F84CD6E39724__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "reg3data.h"

class Reg3Parser  
{
public:
	Reg3Parser();
	virtual ~Reg3Parser();

	virtual bool parse(Reg3Data<float>* data, const char* fname) {
		return true;
	}

	virtual bool write(const Reg3Data<float>& data, const char* fname) {
		return true;
	}
};

#endif // !defined(AFX_REG3PARSER_H__AE88C884_6FE3_433E_B6D2_F84CD6E39724__INCLUDED_)

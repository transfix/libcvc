

/***********************************************************************************/
/*																				   */
/*	  Copyright 2003 University of Texas at Austin                                 */
/*	  Supervisor: Dr C Bajaj bajaj@cs.utexas.edu,                                  */
/*    Authors:    Anthony Thane thanea@ices.utexas.edu                             */
/*                S K Vinay  skvinay@cs.utexas.edu                                 */
/*																				   */
/*    This program is free software; you can redistribute it and/or                */
/*    modify it under the terms of the GNU Lesser General Public                   */
/*    License version 2.1 as published by the Free Software Foundation.            */
/*																				   */
/*    This program is distributed in the hope that it will be useful,              */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of               */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU            */
/*    Lesser General Public License for more details.                              */
/*																				   */
/*    You should have received a copy of the GNU Lesser General Public             */
/*    License along with this library; if not, write to the Free Software          */
/*    Foundation, Inc., 51 Franklin Street, Fifth Floor,                           */
/*    Boston, MA  02110-1301  USA                                                  */
/*                                                                                 */
/***********************************************************************************/


// GeometrySceneArray.h: interface for the GeometrySceneArray class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_GEOMETRYSCENEARRAY_H__A5F41949_2CB3_476B_9C01_F48B9C11E564__INCLUDED_)
#define AFX_GEOMETRYSCENEARRAY_H__A5F41949_2CB3_476B_9C01_F48B9C11E564__INCLUDED_

class GeometryScene;

class GeometrySceneArray
{
public:
	GeometrySceneArray();
	virtual ~GeometrySceneArray();

	void initArray();

	void doubleArray();
	int add( GeometryScene* geometryScene );
	bool set( GeometryScene* geometryScene, unsigned int index );
	GeometryScene* remove( unsigned int index );
	GeometryScene* get( unsigned int index );
	unsigned int getNumberOfObjects();

	void clear();

protected:
	GeometryScene** m_GeometriesScene;
	unsigned int m_NumberOfObjects;
	unsigned int m_SizeOfObjectsArray;
};


#endif // !defined(AFX_GEOMETRYSCENEARRAY_H__A5F41949_2CB3_476B_9C01_F48B9C11E564__INCLUDED_)



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


// GeometryScene.h: interface for the GeometryScene class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_GEOMETRYSCENE_H__8FE62DCD_4415_42D1_B253_3F58FFDDA79C__INCLUDED_)
#define AFX_GEOMETRYSCENE_H__8FE62DCD_4415_42D1_B253_3F58FFDDA79C__INCLUDED_

#include "SceneArrayNode.h"
#include "IntQueue.h"

class Geometry;

//template<> class Q_EXPORT ExpandableArray<Grid*> { };

class GeometryScene
{
public:
	GeometryScene();
	virtual ~GeometryScene();

	int add( SceneArrayNode* sceneArrayNode );
	int add( Geometry* geometry );
	bool set( SceneArrayNode* sceneArrayNode, unsigned int index );
	SceneArrayNode* remove( unsigned int index );
	SceneArrayNode* get( unsigned int index );
	SceneArrayNode* getIth(unsigned int i) const;
	unsigned int getNumberOfSceneArrayNodes();

	void clear();

	void addMe();
	void deleteMe();
protected:
	void initArrays();
	void initObjectArray();
	void initIndexArray();
	void deleteArrays();
	void doubleObjectArray();
	void doubleIndexArray();

	unsigned int m_NumberOfObjects;
	unsigned int m_SizeOfObjectsArray;
	SceneArrayNode** m_SceneArrayNode;
	int* m_ObjToIndex;


	unsigned int m_NextIndexEntry;
	unsigned int m_SizeOfIndexToObjectsArray;
	int* m_IndexToObj;

	Queue m_HoleList;

	unsigned int m_ReferenceCount;
};

#endif // !defined(AFX_GEOMETRYSCENE_H__8FE62DCD_4415_42D1_B253_3F58FFDDA79C__INCLUDED_)
 
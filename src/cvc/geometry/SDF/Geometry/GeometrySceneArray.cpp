

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


// GeometrySceneArray.cpp: implementation of the GeometrySceneArray class.
//
//////////////////////////////////////////////////////////////////////

#include "GeometrySceneArray.h"
#include "GeometryScene.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GeometrySceneArray::GeometrySceneArray()
{
	initArray();
}

GeometrySceneArray::~GeometrySceneArray()
{

}

void GeometrySceneArray::initArray() {

	unsigned int c;
	
	m_NumberOfObjects = 0;
	m_SizeOfObjectsArray = 16;
	m_GeometriesScene = new GeometryScene*[m_SizeOfObjectsArray];

	for(c=0;c<m_SizeOfObjectsArray;c++) {
		m_GeometriesScene[c] = 0;
	}
}

void GeometrySceneArray::doubleArray()
{
	unsigned int c;
	if (m_NumberOfObjects >= m_SizeOfObjectsArray) {
		GeometryScene** oldGeometryScene = m_GeometriesScene;
		m_GeometriesScene = new GeometryScene*[m_SizeOfObjectsArray*2];

		for(c=0;c<m_SizeOfObjectsArray;c++) {
			m_GeometriesScene[c] = oldGeometryScene[c];
		}
		m_SizeOfObjectsArray *= 2;

		delete [] oldGeometryScene;
	}
}

int GeometrySceneArray::add( GeometryScene* geometryScene )
{
	doubleArray();
	m_GeometriesScene[m_NumberOfObjects] = geometryScene;
	m_NumberOfObjects++;
	return m_NumberOfObjects-1;
}

bool GeometrySceneArray::set( GeometryScene* geometryScene, unsigned int index )
{
	if( index <= m_NumberOfObjects ) 
	{
		m_GeometriesScene[index] = geometryScene;
		return true;
	}
	return false;	
}

GeometryScene* GeometrySceneArray::remove( unsigned int index )
{
	if( index <= m_NumberOfObjects && m_GeometriesScene[index] ) 
	{
		GeometryScene* temp = m_GeometriesScene[index];
		m_GeometriesScene[index] = 0;
		return temp;
	}
	else
	{
		return 0;
	}
}

GeometryScene* GeometrySceneArray::get( unsigned int index )
{
	if( index <= m_NumberOfObjects && m_GeometriesScene[index] ) 
	{
		return m_GeometriesScene[index];
	}
	else
	{
		return 0;
	}
}

unsigned int GeometrySceneArray::getNumberOfObjects()
{
	return m_NumberOfObjects;
}

void GeometrySceneArray::clear()
{
	unsigned int c;
	for(c=0;c<m_NumberOfObjects;c++)
	{
		delete m_GeometriesScene[c];
		m_GeometriesScene[c] = 0;
	}
	m_NumberOfObjects=0;
}

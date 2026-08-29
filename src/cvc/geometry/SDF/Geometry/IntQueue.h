

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


// IntQueue.h: interface for the IntQueue class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_INTQUEUE_H__5BCDDDD5_C74D_413B_A351_382DAF0EA82A__INCLUDED_)
#define AFX_INTQUEUE_H__5BCDDDD5_C74D_413B_A351_382DAF0EA82A__INCLUDED_

class QueueNode {
public:
	QueueNode(QueueNode* next, int index);
	QueueNode(const QueueNode& copy);
	QueueNode& operator=(const QueueNode& copy);

	~QueueNode();
	QueueNode* m_Prev;
	QueueNode* m_Next;
	int m_Index;
};


class Queue {
public:
	Queue();
	~Queue();

	Queue& enQueue(int index);
	int deQueue();
	void clearQueue();
	bool isEmpty() const;
	QueueNode* m_Head;
	QueueNode* m_Tail;
};

#endif // !defined(AFX_INTQUEUE_H__5BCDDDD5_C74D_413B_A351_382DAF0EA82A__INCLUDED_)

/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* $Id: GDTVFilter.cpp 4742 2011-10-21 22:09:44Z transfix $ */

#include <cmath>
#include <cvc/volmagick.h>
#include <cvc/app.h>

using namespace std;

namespace CVC_NAMESPACE
{
  static inline float phi(float x, float q)
  {
    return ((2-q)*pow(x, 1-q));
  }

  /*--------------------------------------------------------------------*/
  void Gradient(const voxels& input, voxels& grad)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    float val[6];

    for(unsigned int i = 0; i < input.XDim(); i++)
      {
#ifdef _OPENMP
	#pragma omp parallel for private(val) schedule(dynamic)
#endif
	for(unsigned int j = 0; j < input.YDim(); j++)
	  for(unsigned int k = 0; k < input.ZDim(); k++)
	    {
	      for(int l = 0; l < 6; l++)
		{
		  val[l] = 0.0;
		} 
	      if(i+1 < input.XDim()) val[0] = input(i+1,j,k)-input(i,j,k);
	      if(i >= 1) val[1] = input(i-1,j,k)-input(i,j,k);
	      if(j+1 < input.YDim()) val[2] = input(i,j+1,k)-input(i,j,k);
	      if(j >= 1) val[3] = input(i,j-1,k)-input(i,j,k);
	      if(k+1<input.ZDim())   val[4] = input(i,j,k+1)-input(i,j,k);
	      if(k >= 1) val[5] = input(i,j,k-1)-input(i,j,k);
	      
	      double sum = 0.0;
	      for(int l = 0; l < 6; l++)
		{
		  sum += val[l]*val[l];
		}
	      grad(i,j,k,sqrt(sum));
	    }

	cvcapp.threadProgress(float(i)/float(input.XDim()));
      }
  }  



  /*--------------------------------------------------------------------*/
  void Gradient2(const voxels& input, voxels& grad)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    float val[26];
    for(unsigned int i = 0; i < input.XDim(); i++)
      {
#ifdef _OPENMP
	#pragma omp parallel for private(val) schedule(dynamic)
#endif
	for(unsigned int j = 0; j < input.YDim(); j++)
	  for(unsigned int k = 0; k < input.ZDim(); k++)
	    {
	      for(int l = 0; l < 26; l++)
		{
		  val[l] = 0.0;
		} 
	      if(i+1 < input.XDim()) 
		{
		  val[0] = input(i+1,j,k)-input(i,j,k);
		  if(j+1 < input.YDim()) 
		    {
		      val[6] = input(i+1,j+1,k)-input(i,j,k);
		      if(k+1 < input.ZDim()) val[18] = input(i+1,j+1,k+1)-input(i,j,k);
		      if(k >= 1) val[19] = input(i+1,j+1,k-1)-input(i,j,k);
		    }
		  if(k+1 < input.ZDim()) val[7] = input(i+1,j,k+1)-input(i,j,k);
		  if(j >= 1)
		    {
		      val[8] = input(i+1,j-1,k)-input(i,j,k);
		      if(k+1 < input.ZDim()) val[20] = input(i+1,j-1,k+1)-input(i,j,k);
		      if(k >= 1) val[21] = input(i+1,j-1,k-1)-input(i,j,k);
		    }
		  if(k >= 1) val[9] = input(i+1,j,k-1)-input(i,j,k);
		}
	      if(i >= 1) 
		{
		  val[1] = input(i-1,j,k)-input(i,j,k);
		  if(j+1 < input.YDim()) 
		    {
		      val[10] = input(i-1,j+1,k)-input(i,j,k);
		      if(k+1 < input.ZDim()) val[22] = input(i-1,j+1,k+1)-input(i,j,k);
		      if(k >= 1) val[23] = input(i-1,j+1,k-1)-input(i,j,k);
		    }
		  if(k+1 < input.ZDim()) val[11] = input(i-1,j,k+1)-input(i,j,k);
		  if(j >= 1) 
		    {
		      val[12] = input(i-1,j-1,k)-input(i,j,k);
		      if(k+1 < input.ZDim()) val[24] = input(i-1,j-1,k+1)-input(i,j,k);
		      if(k >= 1) val[25] = input(i-1,j-1,k-1)-input(i,j,k);
		    }
		  if(k >= 1) val[13] = input(i-1,j,k-1)-input(i,j,k);
		}
	      if(j+1 < input.YDim()) val[2] = input(i,j+1,k)-input(i,j,k);
	      if(j >= 1) val[3] = input(i,j-1,k)-input(i,j,k);
	      if(k+1<input.ZDim())   
		{
		  val[4] = input(i,j,k+1)-input(i,j,k);
		  if(j+1 < input.YDim()) val[14] = input(i,j+1,k+1)-input(i,j,k);
		  if(j >= 1) val[15] = input(i,j-1,k+1)-input(i,j,k);
		}
	      if(k >= 1) 
		{
		  val[5] = input(i,j,k-1)-input(i,j,k);
		  if(j+1 < input.YDim()) val[16] = input(i,j+1,k-1)-input(i,j,k);
		  if(j >= 1) val[17] = input(i,j-1,k-1)-input(i,j,k);
		}
            
            
	      double sum = 0.0;
	      for(int l = 0; l < 6; l++)
		{
		  sum += val[l]*val[l];
		}
	      for(int l = 6; l < 18; l++)
		{
		  sum += val[l]*val[l]/2.0;
		}
	      for(int l = 18; l < 26; l++)
		{
		  sum += val[l]*val[l]/3.0;
		}
	      grad(i,j,k,sqrt(sum));
	    }

	cvcapp.threadProgress(float(i)/float(input.XDim()));
      }
  }  


  /*------------------------------------------------------------------------*/
  void filtGDTV(const voxels& input, const voxels& grad, voxels&  funcval, float q, float lbda)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    voxels funcvalue(grad);    
    int l;
    float epsilon = 0.0001;
    float temp[7];
    float sum = 0.0;

    for(unsigned int i = 0; i < grad.XDim(); i++)
      {
	for(unsigned int j = 0; j < grad.YDim(); j++)
	  for(unsigned int k = 0; k < grad.ZDim(); k++)
	    {
	      funcvalue(i,j,k,0.0);
	      for(l = 0; l < 7; l++)
		{
		  temp[l] = 0.0; 
		}
           
	      if(grad(i,j,k) != 0.0)
		{
		  if((i < grad.XDim()-1))
		    {
		      if((grad(i+1,j,k)!=0)) temp[0] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j,k),q)/grad(i+1,j,k);
		      else temp[0] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((i >1))
		    {
		      if((grad(i-1,j,k)!=0)) temp[1] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j,k),q)/grad(i-1,j,k);
		      else  temp[1] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((j < grad.YDim()-1))
		    {
		      if((grad(i,j+1,k)!=0)) temp[2] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j+1,k),q)/grad(i,j+1,k);
		      else temp[2] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((j > 1))
		    { 
		      if((grad(i,j-1,k)!=0))  temp[3] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j-1,k),q)/grad(i,j-1,k);
		      else   temp[3] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((k < grad.ZDim()-1))
		    {
		      if((grad(i,j,k+1)!=0))  temp[4] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j,k+1),q)/grad(i,j,k+1);
		      else   temp[4] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((k >1))
		    {
		      if((grad(i,j,k-1)!=0))  temp[5] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j,k-1),q)/grad(i,j,k-1);
		      else  temp[5] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		}

	      temp[6] = lbda;
	      sum = 0.0;
	      for(l = 0; l < 7; l++)
		sum += temp[l];
          
	      for(l = 0; l < 7; l++)
		temp[l] = temp[l]/sum;
          
         
	      double temple = 0.0;
	      if((i < grad.XDim()-1))  temple +=funcval(i+1,j,k)*temp[0];
	      if((i >1))   temple += funcval(i-1,j,k)*temp[1];
	      if((j < grad.YDim()-1))  temple += funcval(i,j+1,k)*temp[2];
	      if((j > 1))   temple += funcval(i,j-1,k)*temp[3];
	      if((k < grad.ZDim()-1))  temple += funcval(i,j,k+1)*temp[4];
	      if((k >1))   temple += funcval(i,j,k-1)*temp[5];
	      temple += input(i,j,k)*temp[6];
   
	      funcvalue(i,j,k,temple); 
	    }
	
	cvcapp.threadProgress(float(i)/float(grad.XDim()));
      }

    for(unsigned int i = 0; i < grad.XDim(); i++)
      for(unsigned int j = 0; j < grad.YDim(); j++)
	for(unsigned int k = 0; k < grad.ZDim(); k++)
	  {
	    funcval(i,j,k,funcvalue(i,j,k));
	  }
  }


  /*----------------------------------------------------------------------------------------------*/

  void filtGDTV2(const voxels& input, const voxels& grad, voxels&  funcval, float q, float lbda)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    voxels funcvalue(grad);

    int l;
    float epsilon = 0.0001;
    float temp[27];
    float sum = 0.0;

    for(unsigned int i = 0; i < grad.XDim(); i++)
      {
	for(unsigned int j = 0; j < grad.YDim(); j++)
	  for(unsigned int k = 0; k < grad.ZDim(); k++)
	    {
	      funcvalue(i,j,k,0.0);
	      for(l = 0; l < 27; l++)
		{
		  temp[l] = 0.0; 
		}
           
	      if(grad(i,j,k) != 0.0)
		{
		  if((i < grad.XDim()-1))
		    {
		      if((grad(i+1,j,k)!=0)) temp[0] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j,k),q)/grad(i+1,j,k);
		      else temp[0] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		      if((j < grad.YDim()-1))
			{
			  if((grad(i+1,j+1,k)!=0)) temp[6] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j+1,k),q)/grad(i+1,j+1,k));                     else temp[6] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			  if((k < grad.ZDim()-1))
			    {
			      if((grad(i+1,j+1,k+1)!=0))  temp[18] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j+1,k+1),q)/grad(i+1,j+1,k+1))/3.0;
			      else  temp[18] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }
			  if((k >1))
			    {
			      if((grad(i+1,j+1,k-1)!=0))  temp[19] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j+1,k-1),q)/grad(i+1,j+1,k-1))/3.0;
			      else  temp[19] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }
			}
		      if((j > 1))
			{ 
			  if((grad(i+1,j-1,k)!=0)) temp[7] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j-1,k),q)/grad(i+1,j-1,k));
			  if((grad(i+1,j-1,k)==0)) temp[7] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			  if((k < grad.ZDim()-1))
			    {
			      if((grad(i+1,j-1,k+1)!=0))  temp[20] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j-1,k+1),q)/grad(i+1,j-1,k+1))/3.0;
			      else  temp[20] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }
			  if((k >1))
			    {
			      if((grad(i+1,j-1,k-1)!=0))  temp[21] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j-1,k-1),q)/grad(i+1,j-1,k-1))/3.0;
			      else  temp[21] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }

			}
		      if((k < grad.ZDim()-1))
			{
			  if((grad(i+1,j,k+1)!=0))  temp[8] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j,k+1),q)/grad(i+1,j,k+1));
			  else  temp[8] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		      if((k >1))
			{
			  if((grad(i+1,j,k-1)!=0))  temp[9] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i+1,j,k-1),q)/grad(i+1,j,k-1));
			  else  temp[9] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		    }
		  if((i >1))
		    {
		      if((grad(i-1,j,k)!=0)) temp[1] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j,k),q)/grad(i-1,j,k);
		      if((grad(i-1,j,k)==0)) temp[1] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		      if((j < grad.YDim()-1))
			{
			  if((grad(i-1,j+1,k)!=0)) temp[10] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j+1,k),q)/grad(i-1,j+1,k));                    else temp[10] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			  if((k < grad.ZDim()-1))
			    {
			      if((grad(i-1,j+1,k+1)!=0))  temp[22] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j+1,k+1),q)/grad(i-1,j+1,k+1))/3.0;
			      else  temp[22] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }
			  if((k >1))
			    {
			      if((grad(i-1,j+1,k-1)!=0))  temp[23] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j+1,k-1),q)/grad(i-1,j+1,k-1))/3.0;
			      else  temp[23] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }

			}
		      if((j > 1))
			{ 
			  if((grad(i-1,j-1,k)!=0)) temp[11] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j-1,k),q)/grad(i-1,j-1,k));
			  else temp[11] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			  if((k < grad.ZDim()-1))
			    {
			      if((grad(i-1,j-1,k+1)!=0))  temp[24] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j-1,k+1),q)/grad(i-1,j-1,k+1))/3.0;
			      else  temp[24] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }
			  if((k >1))
			    {
			      if((grad(i-1,j-1,k-1)!=0))  temp[25] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j-1,k-1),q)/grad(i-1,j-1,k-1))/3.0;
			      else  temp[25] = (phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon)/3.0;
			    }

			}
		      if((k < grad.ZDim()-1))
			{
			  if((grad(i-1,j,k+1)!=0))  temp[12] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j,k+1),q)/grad(i-1,j,k+1));
			  else  temp[12] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		      if((k >1))
			{
			  if((grad(i-1,j,k-1)!=0))  temp[13] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i-1,j,k-1),q)/grad(i-1,j,k-1));
			  else  temp[13] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		    }
		  if((j < grad.YDim()-1))
		    {
		      if((grad(i,j+1,k)!=0)) temp[2] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j+1,k),q)/grad(i,j+1,k);
		      else  temp[2] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((j > 1))
		    { 
		      if((grad(i,j-1,k)!=0))  temp[3] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j-1,k),q)/grad(i,j-1,k);
		      else  temp[3] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		    }
		  if((k < grad.ZDim()-1))
		    {
		      if((grad(i,j,k+1)!=0))  temp[4] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j,k+1),q)/grad(i,j,k+1);
		      else  temp[4] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		      if((j < grad.YDim()-1))
			{
			  if((grad(i,j+1,k+1)!=0)) temp[14] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j+1,k+1),q)/grad(i,j+1,k+1));                    else temp[14] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		      if((j > 1))
			{ 
			  if((grad(i,j-1,k+1)!=0)) temp[15] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j-1,k+1),q)/grad(i,j-1,k+1));
			  else temp[15] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		    }
		  if((k >1))
		    {
		      if((grad(i,j,k-1)!=0))  temp[5] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j,k-1),q)/grad(i,j,k-1);
		      else  temp[5] = phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon;
		      if((j < grad.YDim()-1))
			{
			  if((grad(i,j+1,k-1)!=0)) temp[16] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j+1,k-1),q)/grad(i,j+1,k-1));                    else temp[16] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}
		      if((j > 1))
			{ 
			  if((grad(i,j-1,k-1)!=0)) temp[17] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(grad(i,j-1,k-1),q)/grad(i,j-1,k-1));
			  else temp[17] = 0.5*(phi(grad(i,j,k),q)/grad(i,j,k)+phi(epsilon,q)/epsilon);
			}

		    }
		}

	      temp[26] = lbda;
	      sum = 0.0;
	      for(l = 0; l < 27; l++)
		sum += temp[l];
          
	      for(l = 0; l < 27; l++)
		temp[l] = temp[l]/sum;
          
         
	      double temple = 0.0;
	      if((i < grad.XDim()-1)) 
		{
		  temple +=funcval(i+1,j,k)*temp[0];
		  if((j < grad.YDim()-1))
		    {
		      temple +=funcval(i+1,j+1,k)*temp[6];
		      if((k < grad.ZDim()-1))  temple +=funcval(i+1,j+1,k+1)*temp[18];
		      if((k > 1))  temple +=funcval(i+1,j+1,k-1)*temp[19];
		    }
		  if(( j > 1))
		    {
		      temple +=funcval(i+1,j-1,k)*temp[7];
		      if((k < grad.ZDim()-1))  temple +=funcval(i+1,j-1,k+1)*temp[20];
		      if((k > 1))  temple +=funcval(i+1,j-1,k-1)*temp[21];
		    } 
		  if((k < grad.ZDim()-1))    temple +=funcval(i+1,j,k+1)*temp[8];
		  if((k > 1))      temple +=funcval(i+1,j,k-1)*temp[9];
		}
	      if((i >1))
		{
		  temple += funcval(i-1,j,k)*temp[1];
		  if((j < grad.YDim()-1))
		    {
		      temple +=funcval(i-1,j+1,k)*temp[10];
		      if((k < grad.ZDim()-1))  temple +=funcval(i-1,j+1,k+1)*temp[22];
		      if((k > 1))  temple +=funcval(i-1,j+1,k-1)*temp[23];
		    }
		  if(( j > 1))
		    {
		      temple +=funcval(i-1,j-1,k)*temp[11];
		      if((k < grad.ZDim()-1))  temple +=funcval(i-1,j-1,k+1)*temp[24];
		      if((k > 1))  temple +=funcval(i-1,j-1,k-1)*temp[25];
		    } 
		  if((k < grad.ZDim()-1))    temple +=funcval(i-1,j,k+1)*temp[12];
		  if((k > 1))      temple +=funcval(i-1,j,k-1)*temp[13];
		}
	      if((j < grad.YDim()-1))  temple += funcval(i,j+1,k)*temp[2];
	      if((j > 1))   temple += funcval(i,j-1,k)*temp[3];
	      if((k < grad.ZDim()-1))  
		{
		  temple += funcval(i,j,k+1)*temp[4];
		  if((j < grad.YDim()-1))    temple +=funcval(i,j+1,k+1)*temp[14];
		  if((j > 1)) temple += funcval(i,j-1,k+1)*temp[15];
		}
	      if((k >1))   
		{
		  temple += funcval(i,j,k-1)*temp[5];
		  if((j < grad.YDim()-1))    temple +=funcval(i,j+1,k-1)*temp[16];
		  if((j > 1)) temple += funcval(i,j-1,k-1)*temp[17];
		}

	      temple += input(i,j,k)*temp[26];
   
	      funcvalue(i,j,k,temple); 
	    }

	cvcapp.threadProgress(float(i)/float(grad.XDim()));
      }

    for(unsigned int i = 0; i < grad.XDim(); i++)
      for(unsigned int j = 0; j < grad.YDim(); j++)
	for(unsigned int k = 0; k < grad.ZDim(); k++)
	  {
	    funcval(i,j,k,funcvalue(i,j,k));
	  }
  }


  voxels& voxels::gdtvFilter(double parameterq, double lambda, unsigned int iteration, unsigned int neigbour)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    voxels filter(*this);
    voxels gradient(*this);
    gradient.voxelType(Float);
    
    if(neigbour == 0){
      Gradient((*this),gradient);
      
      for(unsigned int tt = 0; tt < iteration; tt++){
	filtGDTV((*this), gradient, filter, parameterq, lambda);
      }
    }
    else {
      Gradient2((*this),gradient);
      
      for(unsigned int tt = 0; tt < iteration; tt++){
	filtGDTV2((*this), gradient, filter, parameterq, lambda);
      }
    }

    *this = filter;
    return *this;
  }
}

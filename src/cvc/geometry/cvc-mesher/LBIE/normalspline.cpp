#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "normalspline.h"
#include <cstddef>
#include <vector>
#define DBL_EPSILON 1.0e-10

namespace LBIE
{

/*--------------------------------------------------------------------------*/
 void	ConvertToInterpolationCoefficients
		(
			float	*c,		/* input samples --> output coefficients */
			int	DataLength,	/* number of samples or coefficients */
			float	*z,		/* poles */
			int	NbPoles,	/* number of poles */
			float	Tolerance	/* admissible relative error */
		)

{ /* begin ConvertToInterpolationCoefficients */

	float	Lambda = 1.0;
	int	n, k;

	/* special case required by mirror boundaries */
	if (DataLength == 1) {
		return;
	}
	/* compute the overall gain */
	for (k = 0; k < NbPoles; k++) {
		Lambda = Lambda * (1.0f - z[k]) * (1.0f - 1.0f / z[k]);
	}
	/* apply the gain */
	for (n = 0; n < DataLength; n++) {
		c[n] *= Lambda;
	}
	/* loop over all poles */
	for (k = 0; k < NbPoles; k++) {
		/* causal initialization */
		c[0] = InitialCausalCoefficient(c, DataLength, z[k], Tolerance);
		/* causal recursion */
		for (n = 1; n < DataLength; n++) {
			c[n] += z[k] * c[n - 1];
		}
		/* anticausal initialization */
		c[DataLength - 1] = InitialAntiCausalCoefficient(c, DataLength, z[k]);
		/* anticausal recursion */
		for (n = DataLength - 2; 0 <= n; n--) {
			c[n] = z[k] * (c[n + 1] - c[n]);
		}
	}
} /* end ConvertToInterpolationCoefficients */


double  BS_Fun(double x)
{
	double result;

	if(x<0) x=-x;

	if(x<=1.0){ result=2.0/3.0-x*x+x*x*x/2.0; return result;}
	else {if(x<=2.0) result=(2.0-x)*(2.0-x)*(2.0-x)/6.0; return result;}

	return 0.0;

}


double  BS_GraFun(double x)
{
	double result,y;

	y=x;
	if(y<0.0) y=-y;
	if(y<=1.0){ result=-2.0*y+3.0*y*y/2.0; }
	else {if(y<=2.0) result=-(2.0-y)*(2.0-y)/2.0; }
	if(y>=2.0) result=0.0;
	if(x<0.0) result=-result;

	return result;

}


float  InitialAntiCausalCoefficient
		(
			float	*c,		/* coefficients */
			int	DataLength,	/* number of samples or coefficients */
			float	z			/* actual pole */
		)

{ /* begin InitialAntiCausalCoefficient */

	/* this initialization corresponds to mirror boundaries */
	return((z / (z * z - 1.0f)) * (z * c[DataLength - 2] + c[DataLength - 1]));
} /* end InitialAntiCausalCoefficient */


float	InitialCausalCoefficient
		(
			float	*c,		/* coefficients */
			int	DataLength,	/* number of coefficients */
			float	z,			/* actual pole */
			float	Tolerance	/* admissible relative error */
		)

{ /* begin InitialCausalCoefficient */

	float	Sum, zn, z2n, iz;
	int	n, Horizon;

	/* this initialization corresponds to mirror boundaries */
	Horizon = DataLength;
	if (Tolerance > 0.0) {
		Horizon = (int)ceil(log(Tolerance) / log(fabs(z)));
	}
	if (Horizon < DataLength) {
		/* accelerated loop */
		zn = z;
		Sum = c[0];
		for (n = 1; n < Horizon; n++) {
			Sum += zn * c[n];
			zn *= z;
		}
		return(Sum);
	}
	else {
		/* full loop */
		zn = z;
		iz = 1.0f / z;
		z2n = (float)pow(z, (DataLength - 1));
		Sum = c[0] + z2n * c[DataLength - 1];
		z2n *= z2n * iz;
		for (n = 1; n <= DataLength - 2; n++) {
			Sum += (zn + z2n) * c[n];
			zn *= z;
			z2n *= iz;
		}
		return(Sum / (1.0f - zn * zn));
	}
} /* end InitialCausalCoefficient */


// The lines TransImg2Spline converts, in its order: slice by slice, the slice's Height lines along x
// and then its Width lines along y; then the Width*Height lines along z, x by x.
std::size_t TransImg2SplineLines(int Width, int Height, int SliceNum)
{
	return std::size_t(SliceNum)*std::size_t(Height + Width) + std::size_t(Width)*std::size_t(Height);
}

// Convert line n of TransImg2SplineLines() in place in OutImg (the x lines read InImage); `line`
// is scratch.
void TransImg2SplineLine(float* InImage, float* OutImg, int Width, int Height, int SliceNum,
			 std::size_t n, std::vector<float>& line)
{
	float	Pole[2];
	int	NbPoles;
	int	x, y;
	int i,j,k,index;

	NbPoles = 1;
	Pole[0] = (float) sqrt(3.0) - 2.0f;

	const std::size_t xy = std::size_t(SliceNum)*std::size_t(Height + Width);
	if (n < xy) {
		k = int(n/std::size_t(Height + Width));
		const int r = int(n%std::size_t(Height + Width));
		if (r < Height) {
			/* in-place separable process, along x */
			y = r;
			line.resize(Width);
			index=k*Width*Height+y*Width;
			for(i=0;i<Width;i++) line[i]=InImage[index+i];
			ConvertToInterpolationCoefficients(line.data(), Width, Pole, NbPoles, (float)DBL_EPSILON);
			for(i=0;i<Width;i++) OutImg[index+i]=line[i];
		}
		else {
			/* in-place separable process, along y */
			x = r - Height;
			line.resize(Height);
			index=k*Width*Height+x;
			for(j=0;j<Height;j++) line[j]=OutImg[index+j*Width];
			ConvertToInterpolationCoefficients(line.data(), Height, Pole, NbPoles, (float)DBL_EPSILON);
			for(j=0;j<Height;j++) OutImg[index+j*Width]=line[j];
		}
		return;
	}

	/* in-place separable process, along z */
	x = int((n - xy)/std::size_t(Height));
	y = int((n - xy)%std::size_t(Height));
	line.resize(SliceNum);
	index=y*Width+x;
	for(k=0;k<SliceNum;k++) line[k]=OutImg[index+k*Width*Height];
	ConvertToInterpolationCoefficients(line.data(), SliceNum, Pole, NbPoles, (float)DBL_EPSILON);
	for(k=0;k<SliceNum;k++) OutImg[index+k*Width*Height]=line[k];
}

void TransImg2Spline (float* InImage, float* OutImg, int Width,	int	Height, int SliceNum)

{ 
	/* convert the image samples into interpolation coefficients, a line at a time */
	std::vector<float> line;
	const std::size_t lines = TransImg2SplineLines(Width, Height, SliceNum);
	for (std::size_t n = 0; n < lines; n++)
		TransImg2SplineLine(InImage, OutImg, Width, Height, SliceNum, n, line);
} 


void GradientAtPoint(float *SplineCoeff,float x, float y, float z, int sizeX, int sizeY, int sizeZ,float *grad)
{
	int i,j,k;
	int fx,fy,fz,cx,cy,cz;
	int indexi,indexj,indexk,index;
	//double r;
 
	double tempdoublex,tempdoubley,tempdoublez;
	//double resultx,resulty,resultz;

	tempdoublex=0.0;  tempdoubley=0.0;  tempdoublez=0.0;

	fx=(int)floor(x);  fy=(int)floor(y); fz=(int)floor(z);
	cx=fx+1;           cy=fy+1;          cz=fz+1;

	for(i=fx-1;i<=cx+1;i++)
		for(j=fy-1;j<=cy+1;j++)
			for(k=fz-1;k<=cz+1;k++) {
 
				indexi=i;	indexj=j;	indexk=k;
   
				if(i<0) indexi=-i;
				if(i>=sizeX) indexi=2*sizeX-2-i;
  
				if(j<0) indexj=-j;
				if(j>=sizeY) indexj=2*sizeY-2-j;
  
				if(k<0) indexk=-k;
				if(k>=sizeZ) indexk=2*sizeZ-2-k;

				index=indexk*sizeX*sizeY+indexj*sizeX+indexi;
	   
				tempdoublex+=SplineCoeff[index]*BS_GraFun(x-i)*BS_Fun(j-y)*BS_Fun(k-z);
				tempdoubley+=SplineCoeff[index]*BS_Fun(i-x)*BS_GraFun(y-j)*BS_Fun(k-z);
				tempdoublez+=SplineCoeff[index]*BS_Fun(i-x)*BS_Fun(j-y)*BS_GraFun(z-k);

			}

	grad[0]=(float)tempdoublex;
	grad[1]=(float)tempdoubley;
	grad[2]=(float)tempdoublez;

   }

}

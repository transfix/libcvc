#include <math.h>
  
#include <stdio.h>
#include <stdlib.h>
  
#include <string.h>

#include <cstddef>
#include <vector>

namespace LBIE
{

float   InitialAntiCausalCoefficient(float *, int, float);	
float	InitialCausalCoefficient(float *, int, float, float);
void	ConvertToInterpolationCoefficients(float *, int, float *, int ,float);

void    TransImg2Spline(float *, float *, int, int, int);
// TransImg2Spline a line at a time (normalspline.cpp)
std::size_t TransImg2SplineLines(int, int, int);
void    TransImg2SplineLine(float *, float *, int, int, int, std::size_t, std::vector<float> &);

double  BS_Fun(double);

double  BS_GraFun(double);
					
void GradientAtPoint(float *,float , float , float , int, int , int, float *);

}

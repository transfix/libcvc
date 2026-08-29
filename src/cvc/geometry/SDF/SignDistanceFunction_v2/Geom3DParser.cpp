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
#include <stdio.h>

#include "Geom3DParser.h"

Geom3DParser::Geom3DParser(void)
{
}

Geom3DParser::~Geom3DParser(void)
{
}

bool Geom3DParser::ParseRawFile(FaceVertSet3D& fvs, const char* fname)
{
	FILE	*fp;
	int		i, nverts, ntris;
	float	temp[3];

	if ((fp = fopen(fname, "r")) == NULL)
	{
		fprintf(stderr, "ERROR: fopen(%s)\n", fname);
		return false;
	}

	printf("Reading Geometry: %s\n", fname);

	if (fscanf(fp,"%d %d", &nverts, &ntris) == EOF)
	{
		printf("Input file is not valid....Exiting...\n");
		return false;
	}

	printf("vert= %d and tri = %d \n", nverts,ntris);

	for (i=0; i<nverts; i++)
	{
		if (fscanf(fp,"%f %f %f", &temp[0], &temp[1], &temp[2] ) == EOF)
		{
			printf("Input file has to have %d Vertices....Exiting...\n",nverts);
			return false;
		}

		fvs.addVert(temp[0], temp[1], temp[2]);
	}

	printf("Finished reading the Vertices.. Now reading the Triangles\n");

	int v1, v2, v3;
	for (i=0; i<ntris; i++)
	{
		if (fscanf(fp,"%d %d %d", &v1, &v2, &v3 ) == EOF)
		{
			printf("Input file has to have %d Triangles....Exiting...\n",ntris);
			return false;
		}
		TriId3i t(v1, v2, v3);
		fvs.AddTri(t);
	}

	fclose(fp);
	//TODO: add code to correctly orient all triangles.
	fvs.buildBBox();
	fvs.computeTriNormals();
	//fvs.flipTriNormals();
	
	printf("File %s read.. verts: %d, tris: %d\n",fname, fvs.vertCount(), fvs.triCount());	
	
	return true;
}

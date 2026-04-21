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

/* http://xmipp.cnb.csic.es/twiki/bin/view/Xmipp/ImportingExporting  */
/* Spider read/write routines ---------------------------------------------- */
#include <cvc/volmagick.h>
#include <cvc/endians.h>
#include <cvc/app.h>

#include <boost/scoped_array.hpp>
#include <boost/format.hpp>

#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <sys/stat.h>

using namespace std;
 // TODO: modify this so I/O for spider files is out-of-core.

namespace CVC_NAMESPACE
{
  CVC_DEF_EXCEPTION(invalid_spider_file);

  const char *read_err_strings[] = { "OK",
				     "Cannot open file",
				     "File is not a Spider volume",
				     "Problem when computing the size of the file",
				     "Filetype is neither volume ('V') or image ('I')" };

  typedef struct
  {
    /* 1 */
    float fNslice; // NUMBER OF SLICES (PLANES) IN VOLUME
    // (=1 FOR AN IMAGE)  FOR NEW LONG LABEL
    // FORMAT THE VALUE OF NSLICE STORED IN
    // THE FILE IS NEGATIVE.

    /* 2 */
    float fNrow; // NUMBER OF ROWS PER SLICE (Y)

    /* 3 */
    float fNrec; // TOTAL NUMBER OF RECORDS (SEE NOTE #3).

    /* 4 */
    float fNlabel; // AUXILIARY NUMBER TO COMPUTE TOTAL NUMBER OF RECS

    /* 5 */
    float fIform; // FILE TYPE SPECIFIER.
    // +3 FOR A 3-D FILE  (FLOAT)
    // +1 FOR A 2-D IMAGE (FLOAT)
    // -1 FOR A 2-D FOURIER TRANSFORM
    // -3 FOR A 3-D FOURIER TRANSFORM
    // -5 FOR A NEW 2-D FOURIER TRANSFORM
    // -7 FOR A NEW 3-D FOURIER TRANSFORM
    // +8 FOR A 2-D EIGHT BIT IMAGE FILE
    // +9 FOR A 2-D INT IMAGE FILE
    // 10 FOR A 3-D INT IMAGE FILE
    // 11 FOR A 2-D EIGHT BIT COLOR IMAGE FILE

    /* 6 */
    float fImami; // MAXIMUM/MINIMUM FLAG. IS SET AT 0 WHEN THE
    // FILE IS CREATED, AND AT 1 WHEN THE MAXIMUM AND
    // MINIMUM HAVE BEEN COMPUTED, AND HAVE BEEN STORED
    // INTO THIS LABEL RECORD (SEE FOLLOWING WORDS)

    /* 7 */
    float fFmax; // MAXIMUM VALUE

    /* 8 */
    float fFmin; // MINIMUM VALUE

    /* 9 */
    float fAv; // AVERAGE VALUE

    /* 10*/
    float fSig; // STANDARD DEVIATION. A VALUE OF -1. INDICATES
    // THAT SIG HAS NOT BEEN COMPUTED PREVIOUSLY.

    /* 11*/
    float fIhist; // FLAG INDICATING IF THE HISTOGRAM HAS BE
    // COMPUTED. NOT USED IN 3D FILES!

    /* 12*/
    float fNcol; // NUMBER OF PIXELS PER LINE (Columns X)

    /* 13*/
    float fLabrec; // NUMBER OF LABEL RECORDS IN FILE HEADER

    /* 14*/
    float fIangle; // FLAG THAT TILT ANGLES HAVE BEEN FILLED

    /* 15*/
    float fPhi; // EULER: ROTATIONAL ANGLE

    /* 16*/
    float fTheta; // EULER: TILT ANGLE

    /* 17*/
    float fPsi; // EULER: PSI  = TILT ANGLE

    /* 18*/
    float fXoff; // X TRANSLATION

    /* 19*/
    float fYoff; // Y TRANSLATION

    /* 20*/
    float fZoff; // Z TRANSLATION

    /* 21*/
    float fScale; // SCALE

    /* 22*/
    float fLabbyt; // TOTAL NUMBER OF BYTES IN LABEL

    /* 23*/
    float fLenbyt; // RECORD LENGTH IN BYTES
    char  fNada[24]; // this is a spider incongruence

    /* 30*/
    float fFlag; // THAT ANGLES ARE SET. 1 = ONE ADDITIONAL
    // ROTATION IS PRESENT, 2 = ADDITIONAL ROTATION
    // THAT PRECEEDS THE ROTATION THAT WAS STORED IN
    // 15 FOR DETAILS SEE MANUAL CHAPTER VOCEUL.MAN

    /* 31*/
    float fPhi1;

    /* 32*/
    float fTheta1;

    /* 33*/
    float fPsi1;

    /* 34*/
    float fPhi2;

    /* 35*/
    float fTheta2;

    /* 36*/
    float fPsi2;

    double fGeo_matrix[3][3]; // x9 = 72 bytes: Geometric info
    float fAngle1; // angle info

    float fr1;
    float fr2; // lift up cosine mask parameters

    /** Fraga 23/05/97  For Radon transforms **/
    float RTflag; // 1=RT, 2=FFT(RT)
    float Astart;
    float Aend;
    float Ainc;
    float Rsigma; // 4*7 = 28 bytes
    float Tstart;
    float Tend;
    float Tinc; // 4*3 = 12, 12+28 = 40B

    /** Sjors Scheres 17/12/04 **/
    float Weight; // For Maximum-Likelihood refinement
    float Flip; // 0=no flipping operation (false), 1=flipping (true)

    char fNada2[576]; // empty 700-76-40=624-40-8= 576 bytes

    /*212-214*/
    char szIDat[12]; // LOGICAL * 1 ARRAY DIMENSIONED 10, CONTAINING
    // THE DATE OF CREATION (10 CHARS)

    /*215-216*/
    char szITim[8]; // LOGICAL * 1 ARRAY DIMENSIONED 8, CONTAINING
    // THE TIME OF CREATION (8 CHARS)

    /*217-256*/
    char szITit[160]; // LOGICAL * 1 ARRAY DIMENSIONED 160
  } SpiderHeader;

  // -----
  // _FREAD
  // -----
  // Purpose:
  //   Reads from a file stream to a buffer, swapping bytes if reverse is non-zero.
  //   Meant for reading single fields.
  // ---- Change History ----
  // ??/??/2008 -- Joe R. -- Creation.
  size_t _FREAD(void *dest, size_t size, size_t nitems, FILE *&fp, int reverse)
  {
#if 0
    size_t retval;
    if (!reverse)
      retval = fread(dest, size, nitems, fp);
    else
      {
	char *ptr = (char *)dest;
	int end = 0;
	retval = 0;
	for (unsigned int n = 0; n < nitems; n++)
	  {
	    for (unsigned int i = size - 1; i >= 0; i--)
	      {
		if (fread(ptr + i, 1, 1, fp) != 1)
		  {
		    end = 1;
		    break;
		  }
	      }
	    if (end)
	      break;
	    else
	      retval++;
	    ptr += size;
	  }
      }
    return retval;
#endif

    size_t retval;
    retval = fread(dest, size, nitems, fp);
    if(reverse)
      {
	char *ptr = (char *)dest;
	switch(size)
	  {
	  case 2:
	    for(unsigned int n = 0; n < nitems; n++)
	      {
		SWAP_16(ptr);
		ptr += 2;
	      }
	    break;
	  case 4:
	    for(unsigned int n = 0; n < nitems; n++)
	      {
		SWAP_32(ptr);
		ptr += 4;
	      }
	    break;
	  case 8:
	    for(unsigned int n = 0; n < nitems; n++)
	      {
		SWAP_64(ptr);
		ptr += 8;
	      }
	    break;
	  default: break; //ignore weird sizes for now
	  }
      }

    return retval;
  }

  // --------------
  // readSpiderFile
  // --------------
  // Purpose:
  //   This function reads a Spider volume. Call it as
  //
  //   int dim[3];
  //   float *data=NULL;
  //   readSpiderFile("myfile.vol",'V',dim,&data);
  //   readSpiderFile("myfile.img",'I',dim,&data);
  //
  //   Code errors:
  //   0 - OK
  //   1 - Cannot open file
  //   2 - File is not a Spider volume
  //   3 - Problem when computing the size of the file
  //   4 - filetype is not 'V' or 'I'
  // ---- Change History ----
  // ??/??/2008 -- Joe R. -- Creation.
  int readSpiderFile(const char *filename, char filetype,
		     int dim[], float **data)
  {
    FILE* fp=NULL;
    union {
      unsigned char c[4];
      float         f;
    } file_type;
    int fileReversed=0, machineReversed=0, reversed=0;
    SpiderHeader header;
    unsigned long usfNcol, usfNrow, usfNslice, usfHeader, size, tmpSize,
      volSize, n, currentPosition;
    struct stat info;
    float *ptr;

    /* Set dimensions to 0, just in case it cannot be read */
    dim[0]=0;
    dim[1]=0;
    dim[2]=0;    

    /* Check that the filetype is correct */
    if (filetype!='V' && filetype!='I')
      return 4;

    /* Open file */
    if ((fp = fopen(filename, "rb")) == NULL)
      return 1;

    /* Check that the input file is really a Spider volume */
    currentPosition=ftell(fp);

    /* Check file type */
    fseek(fp, currentPosition+16, SEEK_SET);
    for (int i = 0; i < 4; i++)
      {
	size_t retval = fread(&(file_type.c[i]), sizeof(unsigned char), 1, fp);
      }
    fseek(fp,  currentPosition+0, SEEK_SET);
    switch (filetype)
      {
      case 'V':
	if (file_type.c[0]==  0 && file_type.c[1]== 0 &&
	    file_type.c[2]== 64 && file_type.c[3]==64)
	  {
	    fileReversed=0;
	  }
	else if (file_type.c[0]==64 && file_type.c[1]==64 &&
		 file_type.c[2]== 0 && file_type.c[3]== 0)
	  {
	    fileReversed=1;
	  }
	else
	  {
	    fclose(fp);
	    return 2;
	  }
	break;
      case 'I':
	if (file_type.c[0]==  0 && file_type.c[1]== 0 &&
	    file_type.c[2]==128 && file_type.c[3]==63)
	  {
	    fileReversed=0;
	  }
	else if (file_type.c[0]==63 && file_type.c[1]==128 &&
		 file_type.c[2]== 0 && file_type.c[3]==  0)
	  {
	    fileReversed=1;
	  }
	else
	  {
	    fclose(fp);
	    return 2;
	  }
	break;
      }

    /* Check machine type */
    file_type.f = 1;
    if (file_type.c[0]==63 && file_type.c[1]==128 && file_type.c[2]==0 &&
	file_type.c[3]==0)
      machineReversed=1;

    /* Read header */
    reversed=fileReversed ^ machineReversed;
    if (!reversed)
      {
	size_t retval = fread(&header, sizeof(SpiderHeader), 1, fp);
      }
    else
      {
	_FREAD(&header,             sizeof(float),  36, fp, true);
	_FREAD(&header.fGeo_matrix, sizeof(double),  9, fp, true);
	_FREAD(&header.fAngle1,     sizeof(float),  13, fp, true);
	_FREAD(&header.fNada2,      sizeof(char),  756, fp, true);
      }

    /* Compute file size, header size and volume dimensions */
    usfNcol = (unsigned long) header.fNcol;
    usfNrow = (unsigned long) header.fNrow;
    usfNslice = (unsigned long) header.fNslice;
    usfHeader = (unsigned long) header.fNcol *
      (unsigned long) header.fLabrec * sizeof(float);
    if (fstat(fileno(fp), &info))
      {
	fclose(fp);
	return 3;
      }

    /* Readjust the number of rows in "aberrant" images*/
    if (filetype=='I' || header.fIform == 1)
      if (usfNcol*usfNrow*sizeof(float) == info.st_size)
	{
	  usfNrow = (unsigned long)(--header.fNrow);
	  --header.fNrec;
	}

    /* Check that the file size is correct */
    switch (filetype)
      {
      case 'I':
	size = usfHeader + usfNcol * usfNrow * sizeof(float);
	if ((size != info.st_size) || (header.fIform != 1))
	  {
	    fclose(fp);
	    return 2;
	  }
	break;
      case 'V':
	size = usfHeader + usfNslice * usfNcol * usfNrow * sizeof(float);
	if ((size != info.st_size) || (header.fIform != 3))
	  {
	    fclose(fp);
	    return 2;
	  }
	break;
      }
    
    /* Read the extra filling header space */
    tmpSize = (unsigned long)(header.fNcol * header.fLabrec * 4);
    tmpSize -= sizeof(SpiderHeader);
    for (unsigned int i = 0; i < tmpSize / 4; i++)
      {
	float tmp;
	size_t retval = fread(&tmp, sizeof(float), 1, fp);
      }
    currentPosition=ftell(fp);

    /* Fill the dimensions */
    dim[0]=(int)header.fNcol;
    dim[1]=(int)header.fNrow;
    if (filetype=='V') dim[2]=(int)header.fNslice;
    else               dim[2]=1;
    volSize = (unsigned long) dim[0]*dim[1]*dim[2];

    /* Read the whole file */
    if (*data!=NULL)
      free(data);
    *data=(float *) malloc(volSize*sizeof(float));
    ptr=*data;
    for (n=0; n<volSize; n++)
      _FREAD(ptr++, sizeof(float), 1, fp, reversed);

    /* Close file */
    fclose(fp);
    return 0;
  }

  // ---------------
  // writeSpiderFile
  // ---------------
  // Purpose:
  //   This function writes a Spider volume. Call it as
  //
  //   writeSpiderFile("myfile.vol",'V',dim,data);
  //   writeSpiderFile("myfile.img",'I',dim,data);
  //    
  //   Code errors:
  //   0 - OK
  //   1 - Cannot open file
  // ---- Change History ----
  // ??/??/2008 -- Joe R. -- Creation.
  int writeSpiderFile(const char *filename, char filetype,
		      int dim[], float *data)
  {
    FILE* fp;
    SpiderHeader header;
    unsigned long volSize, tmpSize;
    float tmp=0.0F;
    int headrec;

    /* Check that the filetype is correct */
    if (filetype!='V' && filetype!='I')
      return 4;

    /* Open file for writing */
    if ((fp = fopen(filename, "wb")) == NULL)
      return 1;

    /* Adjust header */
    header.fNcol = dim[0];
    header.fNrow = dim[1];
    header.fNslice=dim[2];
    if ((header.fNcol != 0) && (header.fNrow != 0))
      {
	header.fNlabel = (float)((int)(256 / header.fNcol + 1));
	header.fLabrec = (float) ceil((float)(256 / (float)header.fNcol));

	headrec = (int) 1024 / ((int)header.fNcol * 4);

	if ((1024 % (int)header.fNcol != 0))
	  {
	    header.fNrec = header.fNrow + 1;
	    headrec++;
	  }
	else
	  header.fNrec = header.fNrow;

	header.fLabbyt = header.fNcol * header.fLabrec * 4;
	header.fLenbyt = (float) header.fNcol * 4;
      }
    if (filetype=='V') header.fIform = 3;
    else if (filetype=='I') header.fIform = 1;
    header.fScale = 1;
    for (unsigned i = 0; i < 3; i++)
      for (unsigned j = 0; j < 3; j++)
	if (i == j)
	  header.fGeo_matrix[i][j] = (double)1.0;
	else
	  header.fGeo_matrix[i][j] = (double)0.0;
    header.fSig = -1;
    header.fImami = header.fFmax = header.fFmin = header.fAv =
      header.fIhist = header.fIangle = header.fPhi = header.fTheta =
      header.fPsi = header.fXoff = header.fYoff = header.fZoff =
      header.fPhi1 = header.fTheta1 = header.fPsi1 =
      header.fPhi2 = header.fTheta2 = header.fPsi2 = header.fAngle1 = 
      header.fr1 = header.fr2 = header.RTflag = header.Astart =
      header.Aend = header.Ainc = header.Rsigma = header.Tstart =
      header.Tend = header.Tinc = header.Weight = header.Flip = 0.0F;
    header.szIDat[0] = header.szITim[0] = header.szITit[0] =
      header.fNada2[0] = header.fNada[0] = 0;

    // Write the header and the necessary empty space
    fwrite(&header, sizeof(SpiderHeader), 1, fp);
    tmpSize = (header.fNcol * header.fLabrec * 4);
    tmpSize -= sizeof(SpiderHeader);
    for (unsigned int i = 0; i < tmpSize/4; i++)
      fwrite(&tmp, sizeof(float), 1, fp);

    // Now write the data
    fwrite(data, sizeof(float), (unsigned long) dim[0]*dim[1]*dim[2], fp);

    // Close file
    fclose(fp);
    return 0;
  }

  // ---------
  // spider_io
  // ---------
  // Purpose: 
  //   Provides Spider file support.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Initial implemenetation.
  struct spider_io : public volume_file_io
  {
    // --------------------
    // spider_io::spider_io
    // --------------------
    // Purpose:
    //   Initializes the extension list and id.
    // ---- Change History ----
    // 11/20/2009 -- Joe R. -- Creation.
    spider_io()
      : _id("spider_io : v1.0")
    {
      _extensions.push_back(".vol");
      _extensions.push_back(".xmp");
      _extensions.push_back(".spi");
    }

    // -------------
    // spider_io::id
    // -------------
    // Purpose:
    //   Returns a string that identifies this volume_file_io object.  This should
    //   be unique, but is freeform.
    // ---- Change History ----
    // 11/20/2009 -- Joe R. -- Creation.
    virtual const std::string& id() const
    {
      return _id;
    }

    // ---------------------
    // spider_io::extensions
    // ---------------------
    // Purpose:
    //   Returns a list of extensions that this volume_file_io object supports.
    // ---- Change History ----
    // 11/20/2009 -- Joe R. -- Creation.
    virtual const extension_list& extensions() const
    {
      return _extensions;
    }

    // ----------------------------
    // spider_io::getVolumeFileInfo
    // ----------------------------
    // Purpose:
    //   Writes to a structure containing all info that VolMagick needs
    //   from a volume file.
    // ---- Change History ----
    // ??/??/2008 -- Joe R. -- Creation.
    // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class.
    // 12/11/2009 -- Joe R. -- Freeing buf if read successful.
    virtual void getVolumeFileInfo(app& /*ctx*/,
				   volume_file_info::data& d,
				   const std::string& filename) const
    {
      thread_info ti(BOOST_CURRENT_FUNCTION);

      int dim[3];
      float *buf=NULL;
      int retval = readSpiderFile(filename.c_str(),'I',dim,&buf);
      if(retval)
	retval = readSpiderFile(filename.c_str(),'V',dim,&buf);
      if(retval)
	{
	  if(buf) free(buf);
	  switch(retval)
	    {
	    case 1:
	      throw read_error(read_err_strings[retval]);
	      break;
	    case 2:
	    default:
	      throw invalid_spider_file(read_err_strings[retval]);
	    }
	}

      if(buf) free(buf);
    
      /**** at this point, the volume header has no errors, so we may start modifiying this object ****/
      d._numVariables = 1;
      d._numTimesteps = 1;
      d._tmin = d._tmax = 0.0;
    
      d._dimension = dimension(dim);
      d._boundingBox = bounding_box(0.0, 0.0, 0.0,
				      double(dim[0]-1),double(dim[1]-1),double(dim[2]-1));

      d._voxelTypes.clear();
      d._voxelTypes.push_back(Float);
      d._names.clear();
      d._names.push_back("No Name");

      /* new volume, so min/max is now unset */
      d._minIsSet.clear();
      d._minIsSet.resize(d._numVariables); for(int i=0; i<d._minIsSet.size(); i++) d._minIsSet[i].resize(d._numTimesteps);
      d._min.clear();
      d._min.resize(d._numVariables); for(int i=0; i<d._min.size(); i++) d._min[i].resize(d._numTimesteps);
      d._maxIsSet.clear();
      d._maxIsSet.resize(d._numVariables); for(int i=0; i<d._maxIsSet.size(); i++) d._maxIsSet[i].resize(d._numTimesteps);
      d._max.clear();
      d._max.resize(d._numVariables); for(int i=0; i<d._max.size(); i++) d._max[i].resize(d._numTimesteps);

      d._filename = filename;
    }

    // -------------------------
    // spider_io::readVolumeFile
    // -------------------------
    // Purpose:
    //   Writes to a Volume object after reading from a volume file.
    // ---- Change History ----
    // ??/??/2008 -- Joe R. -- Creation.
    // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class.
    // 12/11/2009 -- Joe R. -- Freeing data buffer.
    virtual void readVolumeFile(app& /*ctx*/, volume& vol,
				const std::string& filename, 
				unsigned int var, unsigned int time,
				uint64 off_x, uint64 off_y, uint64 off_z,
				const dimension& subvoldim) const
    {
      thread_info ti(BOOST_CURRENT_FUNCTION);

      int dim[3];
      float *data=NULL;

      if(var > 0)
	throw index_out_of_bounds("Variable index out of bounds.");
      if(time > 0)
	throw index_out_of_bounds("Timestep index out of bounds.");
      if(subvoldim.isNull())
	throw index_out_of_bounds("Specified subvolume dimension is null.");

      int retval = readSpiderFile(filename.c_str(),'I',dim,&data);
      if(retval)
	retval = readSpiderFile(filename.c_str(),'V',dim,&data);
      if(retval)
	{
	  if(data) free(data);
	  switch(retval)
	    {
	    case 1:
	      throw read_error(read_err_strings[retval]);
	      break;
	    case 2:
	    default:
	      throw invalid_spider_file(read_err_strings[retval]);
	    }
	}

      if((off_x + subvoldim[0] - 1 >= (unsigned int)dim[0]) ||
	 (off_y + subvoldim[1] - 1 >= (unsigned int)dim[1]) ||
	 (off_z + subvoldim[2] - 1 >= (unsigned int)dim[2]))
	throw index_out_of_bounds("Subvolume specified is outside volume dimensions");

      vol.voxelType(Float);
      vol.voxel_dimensions(subvoldim);
      vol.boundingBox(bounding_box(double(off_x),
				  double(off_y),
				  double(off_z),
				  double(off_x+subvoldim[0]-1),
				  double(off_y+subvoldim[1]-1),
				  double(off_z+subvoldim[2]-1)));

      for(uint64 k = 0; k < vol.ZDim(); k++)
	for(uint64 j = 0; j < vol.YDim(); j++)
	  for(uint64 i = 0; i < vol.XDim(); i++)
	    vol(i,j,k, data[dim[0]*dim[1]*(k+off_z) + dim[0]*(j+off_y) + (i+off_x)]);

      free(data);
    }

    // ---------------------------
    // spider_io::createVolumeFile
    // ---------------------------
    // Purpose:
    //   Creates an empty volume file to be later filled in by writeVolumeFile
    // ---- Change History ----
    // ??/??/2008 -- Joe R. -- Creation.
    // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class.
    virtual void createVolumeFile(app& /*ctx*/,
				  const std::string& filename,
				  const bounding_box& boundingBox,
				  const dimension& dimension,
				  const std::vector<data_type>& voxelTypes,
				  unsigned int numVariables, unsigned int numTimesteps,
				  double min_time, double max_time) const
    {
      using namespace boost;
      thread_info ti(BOOST_CURRENT_FUNCTION);

      int dim[3] = { static_cast<int>(dimension[0]), static_cast<int>(dimension[1]), static_cast<int>(dimension[2]) };
      boost::scoped_array<float> data(new float[dimension.size()]);

      if(boundingBox.isNull())
	throw invalid_bounding_box("Bounding box must not be null");
      if(dimension.isNull())
	throw invalid_bounding_box("Dimension must not be null");
      if(numVariables > 1)
	throw invalid_spider_file(str(format("Spider format only supports 1 variable (%d requested)") % numVariables));
      if(numTimesteps > 1)
	throw invalid_spider_file(str(format("Spider format only supports 1 timestep (%d requested)") % numTimesteps));
      if(voxelTypes.size() > 1)
	throw invalid_spider_file("Spider format only supports 1 variable and 1 timestep. (too many voxel types specified)");
      if(voxelTypes[0] != Float)
	throw invalid_spider_file("Spider format only supports float valued voxels.");
      if(min_time != max_time)
	throw invalid_spider_file("Spider format does not support multiple timesteps. (min time and max time must be the same)");

      int retval = writeSpiderFile(filename.c_str(), 'V',dim,data.get());
      if(retval)
	throw invalid_spider_file(read_err_strings[retval]);
    }

    // --------------------------
    // spider_io::writeVolumeFile
    // --------------------------
    // Purpose:
    //   Writes the volume contained in wvol to the specified volume file. Should create
    //   a volume file if the filename provided doesn't exist.  Else it will simply
    //   write data to the existing file.  A common user error arises when you try to
    //   write over an existing volume file using this function for unrelated volumes.
    //   If what you desire is to overwrite an existing volume file, first run
    //   createVolumeFile to replace the volume file.
    // ---- Change History ----
    // ??/??/2008 -- Joe R. -- Creation.
    // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class.
    virtual void writeVolumeFile(app& ctx, const volume& wvol, 
				 const std::string& filename,
				 unsigned int var, unsigned int time,
				 uint64 off_x, uint64 off_y, uint64 off_z) const
    {
      thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

      volume vol(ctx);

      if(var > 0)
	throw index_out_of_bounds("Variable index out of bounds.");
      if(time > 0)
	throw index_out_of_bounds("Timestep index out of bounds.");

      try
	{
	  volume_file_info volinfo(ctx, filename);
	  readVolumeFile(ctx, vol, filename, 0, 0, 0, 0, 0, volinfo.voxel_dimensions());
	}
      catch(read_error &e)
	{
	  CVC_NAMESPACE::createVolumeFile(ctx, filename,
			   bounding_box(0.0,0.0,0.0,1.0,1.0,1.0),
			   wvol.voxel_dimensions(),
			   std::vector<data_type>(1,wvol.voxelType()),
			   0,0,0.0,0.0);
	  readVolumeFile(ctx, vol, filename, 0, 0, 0, 0, 0, wvol.voxel_dimensions());
	}

      for(uint64 k = 0; k < wvol.ZDim(); k++)
	for(uint64 j = 0; j < wvol.YDim(); j++)
	  for(uint64 i = 0; i < wvol.XDim(); i++)
	    vol(i+off_x,j+off_y,k+off_z, wvol(i,j,k));

      int dim[] = { static_cast<int>(vol.XDim()), static_cast<int>(vol.YDim()), static_cast<int>(vol.ZDim()) };
      vol.voxelType(Float); //make sure we have a float volume (though we shouldnt get here if we dont)
      int retval = writeSpiderFile(filename.c_str(),'V',dim,reinterpret_cast<float*>(*vol));
      if(retval)
	throw invalid_spider_file(read_err_strings[retval]);
    }

  protected:
    std::string _id;
    std::list<std::string> _extensions;
  };
}

namespace CVC_NAMESPACE
{
  void register_spider_io(app& /*ctx*/)
  {
    volume_file_io::insertHandler(
      volume_file_io::ptr(new spider_io)
    );
  }
}


/*---------------  write Ary into files of various formats ------------*/
/*
   G.Matas, 17-Jun-93
     created
     Bary2P4pbm is a modification of Andrew Elms's code

*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)writeAry.c	2.7	95/06/07 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <stdlib.h>
#include "ary.h"

/*-------------------  auxilliary functions ---------------------------*/
static FILE *  ImageOpen(char * filename,char * type)
{
  FILE * file;

  if (NULL == filename)
    ErrHandle("ImageOpen (write): filename is NULL",-1);

  if      (filename[0]=='-' && filename[1]=='\0')  file = stdout;
  else if (NULL == (file = fopen(filename,type)))
    ErrHandle("ImageOpen: can't open file for write",-1);

  return file;
}

static void ImageClose(FILE * file)
{
  if (file != stdout) fclose(file);
}

/*---------- Write Ary into a format that can store ARY of any type --------*/

const void * WriteAry(const void *a, char *filename)
{
  const ARY *  ary = a;
  FILE * file = ImageOpen(filename,"wb");
  int elSize  = ary->el_size;

  if ((EOF==fprintf(file,"ARY%d %d\n",(int)ary->ary_type,elSize)) ||
      (EOF==fprintf(file,"%d %d %d %d\n",ary->lb2,ary->ub2,ary->lb1,ary->ub1)))
    ErrHandle("WriteAry can't write ARY header",-1);
  
  ImageClose(file);
   
  file = ImageOpen(filename,"ab");
  {
    int n_of_bytes = (ary->ub1-ary->lb1+1)*(ary->ub2-ary->lb2+1)* elSize;
    if (n_of_bytes != fwrite((char*)(ary->el[ary->lb1])+elSize*ary->lb2,
		              sizeof(char),n_of_bytes,file))
      ErrHandle("WriteAry: can't write ARY data",-1);
  }

  ImageClose(file);

  return ary;
}

/*--------------------- I/O interface to the pbmplus package ---------------*/
void * Bary2P5pgmCm(BARY * image,char * filename,char * comment)
{
   FILE * file = ImageOpen(filename,"w");
   int cols = image->ub2 - image->lb2+1;
   int rows = image->ub1 - image->lb1+1;
   
   if (NULL == comment) comment="";

   if (EOF == fprintf(file,"P5\n%s%d %d\n255\n",comment,cols,rows))
     ErrHandle("Bary2P5pgm: couldn't write image header",-1);

   ImageClose(file);
   ImageOpen(filename,"ab");  /* the file must be reopened as binary */
			      /* otherwise any newlines might be expanded */

   if (rows*cols != fwrite(
 	&(image->el[image->lb1][image->lb2]),sizeof(char),cols*rows,file))
     ErrHandle("Bary2P5pgm: couldn't write the whole image",-1);

   ImageClose(file);
   return image;
}
void * Bary2P5pgm(BARY * image,char * filename)
  { return Bary2P5pgmCm(image,filename,""); }

/*---------------------------------------------------------------------*/
void * Bary2P4pbmCm(BARY * image,char * filename, char * comment)
{
  FILE * file = ImageOpen(filename,"w");
  int cols = image->ub2 - image->lb2+1;
  int rows = image->ub1 - image->lb1+1;
  int i,j;
  unsigned char mask, outputByte;
  
  if (NULL == comment) comment="";

  if (EOF == fprintf(file,"P4\n%s%d %d\n",comment,cols,rows))
    ErrHandle("Bary2P4pbm: couldn't write image header",-1);

  ImageClose(file);
  ImageOpen(filename,"ab"); /* the file must be reopened as binary */
			    /* otherwise any newlines might be expanded */

  for (i=0; i<rows; i++)
    for (j=0; j<cols; )
    {
      outputByte = 0;

      for(mask=128; (mask>0) && j!=cols; mask >>= 1)
	if(image->el[i][j++]) outputByte |= mask;

      if(EOF == fputc(outputByte,file))
	    ErrHandle("Error while writting data to pbm P4 file ",-1);
    }

  ImageClose(file);

  return image;
}

void * Bary2P4pbm(BARY * image,char * filename)
  { return  Bary2P4pbmCm(image,filename,""); }

/*---------------------------------------------------------------------*/
static int MaxIary(IARY *ary)
{
  int i,j;
  int maximum = ary->el[ary->lb1][ary->ub1];

  for(i=ary->lb1; i<=ary->ub1; i++) 
    for(j=ary->lb2; j<=ary->ub2; j++)
      if (ary->el[i][j] > maximum)
	maximum = ary->el[i][j];

  return maximum;
}

void* Iary2P2pgm(IARY * ary, char * filename)
{
  FILE * file = ImageOpen(filename,"w");
  int cols = ary->ub2 - ary->lb2+1;
  int rows = ary->ub1 - ary->lb1+1;
  int i,j;

  int max = MaxIary(ary);
  if (0 == max) {max++;}
  if (EOF == fprintf(file,"P2\n%d %d\n%d\n",cols,rows,MaxIary(ary)))
    ErrHandle("Bary2P5pgm: couldn't write image header",-1);

  for (i=0; i<rows; i++) {
    for (j=0; j<cols;j++ ) {
      fprintf(file,"%d ",ary->el[i][j]);
    }
    fprintf(file,"\n");
  }
  ImageClose(file);
  return ary;
}

/*---------------------------------------------------------------------*/
void * RGBary2P6ppmCm(RGBARY * image,char * filename, char * comment)
{
   FILE * file = ImageOpen(filename,"w");
   int cols = image->ub2 - image->lb2+1;
   int rows = image->ub1 - image->lb1+1;
   
   if (NULL == file) 
      return 0;
   if (EOF == fprintf(file,"P6\n%s%d %d\n255\n",comment,cols,rows))
     ErrHandle("RGBary2P6ppm: couldn't write image header",-1);

   ImageClose(file);
   ImageOpen(filename,"ab");  /* the file must be reopened as binary */
			      /* otherwise any newlines might be expanded */

   if (rows*cols != fwrite(
 	&(image->el[image->lb1][image->lb2]),sizeof(t_rgb),cols*rows,file))
     ErrHandle("Bary2P5pgm: couldn't write the whole image",-1);

   ImageClose(file);
   return image;
}
void * YUVary2P6ppm(YUVARY * image,char * filename)
  { return RGBary2P6ppm( (RGBARY*) image,filename); }

void * RGBary2P6ppm(RGBARY * image,char * filename)
  { return RGBary2P6ppmCm(image,filename,""); }

int XRGBary2P6ppm(XRGBARY * xrgbimage, char * filename)
{  
   RGBARY * rgbimage;
   rgbimage = XRGBary2RGBary(xrgbimage);
   RGBary2P6ppm(rgbimage, filename);
   return 1;
}

int RGBXary2P6ppm(RGBXARY * xrgbimage, char * filename)
{  
   RGBARY * rgbimage;
   rgbimage = RGBXary2RGBary(xrgbimage);
   RGBary2P6ppm(rgbimage, filename);
   return 1;
}

/*---------------------------------------------------------------------*/
void CutFary2P5pgm(FARY * image,char * filename)
{
   BARY * bary = Fary2Bary(image);
   Bary2P5pgm(bary,filename);
   free(bary);
}
/*---------------------------------------------------------------------*/
void ScaleFary2P5pgm(FARY * image,char * filename)
{
   BARY * bary = ScaleMinMaxFary2Bary(image);
   Bary2P5pgm(bary,filename);
   free(bary);
}

/*---------------------------------------------------------------------*/
int Ary2Px(ARY* ary,char * filename)
{
  switch (ary->ary_type)
	{	
    case BYTEMATRIX:
    case UBYTEMATRIX:   Bary2P5pgm((BARY*)ary,filename);          break;
    case INTEGERMATRIX: Iary2P2pgm((IARY*)ary,filename);          break;
    case FLOATMATRIX:   ScaleFary2P5pgm((FARY*)ary,filename);     break;
    case RGBMATRIX:     RGBary2P6ppm((RGBARY*)ary,filename) ;     break;
    case XRGBMATRIX:    XRGBary2P6ppm((XRGBARY*)ary,filename);    break;
    case YUVMATRIX:     YUVary2P6ppm((YUVARY*)ary,filename);      break;

    case RGBXMATRIX:    RGBXary2P6ppm((RGBXARY*)ary,filename);    break;
    case FRGBMATRIX:
    case RGIMATRIX:
    case SHORTINTMATRIX: 
    case DOUBLEMATRIX:
    case POINTERMATRIX: 
    default:
      return 1;
    /*     break; */   
                    /*  break commented out to stop warnings on sgi */
		}
  return 0;
}
/*---------------------------------------------------------------------*/
void printFary(FARY * ary)
{
  int i,j;

  fprintf(stderr,"          ");
  for(i=ary->lb2; i<=ary->ub2; i++) fprintf(stderr,"    %3d    ",i);
  fprintf(stderr,"\n");

  for(i=ary->lb1;i<=ary->ub1;i++)
  {
   fprintf(stderr,"  %2d   ",i);
    for(j=ary->lb2;j<=ary->ub2;j++)
      fprintf(stderr," %9f ",ary->el[i][j]);
    fprintf(stderr,"\n");
  }
}

/*---------------------------------------------------------------------*/
void printIary(IARY * ary)
{
  int i,j;

  fprintf(stderr,"          ");
  for(i=ary->lb2; i<=ary->ub2; i++) fprintf(stderr,"    %3d    ",i);
  fprintf(stderr,"\n");

  for(i=ary->lb1;i<=ary->ub1;i++)
  {
   fprintf(stderr,"  %2d   ",i);
    for(j=ary->lb2;j<=ary->ub2;j++)
      fprintf(stderr," %5d ",ary->el[i][j]);
    fprintf(stderr,"\n");
  }
}
/*---------------------------------------------------------------------*/
#include "aryio.h"

void* Iary2BMP(IARY * ary, char * filename)
{
  FILE * f = ImageOpen(filename,"wb");
  int cols = ary->ub2 - ary->lb2+1;
  int rows = ary->ub1 - ary->lb1+1;
  BARY * b = Iary2Bary(ary);

  Win3xBMP w = ConsBMPHeader(rows,cols);
  w.palette[0].st.b=255;

  if(EOF==fwrite(w.header.imageFileType,sizeof(w.header.imageFileType),1,f))
    ErrHandle("Iary2BMP: couldn't write header ",-1);

  fwrite(&w.header.fileSize,sizeof(w.header.fileSize),1,f);
  fwrite(&w.header.reserved1,sizeof(w.header.reserved1),1,f);
  fwrite(&w.header.reserved2,sizeof(w.header.reserved2),1,f);
  fwrite(&w.header.imageDataOffset,sizeof(w.header.imageDataOffset),1,f);

  fwrite(&w.info.headerSize,sizeof(w.info.headerSize),1,f);
  fwrite(&w.info.x,sizeof(w.info.x),1,f);
  fwrite(&w.info.imageWidth,sizeof(w.info.imageWidth),1,f);
  fwrite(&w.info.imageHeight,sizeof(w.info.imageHeight),1,f);
  fwrite(&w.info.numberOfImagePlanes,sizeof(w.info.numberOfImagePlanes),1,f);
  fwrite(&w.info.bitsPerPixel,sizeof(w.info.bitsPerPixel),1,f);
  fwrite(&w.info.compressionMethod,sizeof(w.info.compressionMethod),1,f);
  fwrite(&w.info.sizeOfBitmap,sizeof(w.info.sizeOfBitmap),1,f);
  fwrite(&w.info.horzResolution,sizeof(w.info.horzResolution),1,f);
  fwrite(&w.info.vertResolution,sizeof(w.info.vertResolution),1,f);
  fwrite(&w.info.numColorsUsed,sizeof(w.info.numColorsUsed),1,f);
 fwrite(&w.info.numSignificantColors,sizeof(w.info.numSignificantColors),1,f);
 
  {
  int i;
  for( i = 0; i<w.info.numColorsUsed; i++)
    fwrite(&w.palette[i].st,sizeof(w.palette[0].st),1,f);
  }

  {
  int i,j;
  int padding = colsAry(b)%4;
  for(i=b->ub1;i>=b->lb1;i--)
  {
    for(j=b->lb2;j<=b->ub2;j++)
      fwrite(&(b->el[i][j]),sizeof(char),1,f);
    if (padding>0)
    for(j=0;j<4-padding; j++) 
      fputc(0,f);
  }
  }
  
  ImageClose(f);
  destAry(b);
  return ary;
}


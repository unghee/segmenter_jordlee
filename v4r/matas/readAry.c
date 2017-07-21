/*---------------  write Ary into files of various formats ------------*/
/*
   G.Matas, 17-Jun-93
     created
     P4pbm2Bary is a modification of Andrew Elms's code
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)readAry.c	2.15	95/06/08 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ary.h"           /* for the ary types  */

/*--------- table used for fast unsigned char to float conversion -----*/
static float byte2f[256]=
{
 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0,
 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0,
 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0,
 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0,
 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0,
 70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0,
 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0,
 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0,
 100.0, 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0,
 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0, 117.0, 118.0, 119.0,
 120.0, 121.0, 122.0, 123.0, 124.0, 125.0, 126.0, 127.0, 128.0, 129.0,
 130.0, 131.0, 132.0, 133.0, 134.0, 135.0, 136.0, 137.0, 138.0, 139.0,
 140.0, 141.0, 142.0, 143.0, 144.0, 145.0, 146.0, 147.0, 148.0, 149.0,
 150.0, 151.0, 152.0, 153.0, 154.0, 155.0, 156.0, 157.0, 158.0, 159.0,
 160.0, 161.0, 162.0, 163.0, 164.0, 165.0, 166.0, 167.0, 168.0, 169.0,
 170.0, 171.0, 172.0, 173.0, 174.0, 175.0, 176.0, 177.0, 178.0, 179.0,
 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0,
 190.0, 191.0, 192.0, 193.0, 194.0, 195.0, 196.0, 197.0, 198.0, 199.0,
 200.0, 201.0, 202.0, 203.0, 204.0, 205.0, 206.0, 207.0, 208.0, 209.0,
 210.0, 211.0, 212.0, 213.0, 214.0, 215.0, 216.0, 217.0, 218.0, 219.0,
 220.0, 221.0, 222.0, 223.0, 224.0, 225.0, 226.0, 227.0, 228.0, 229.0,
 230.0, 231.0, 232.0, 233.0, 234.0, 235.0, 236.0, 237.0, 238.0, 239.0,
 240.0, 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0,
 250.0, 251.0, 252.0, 253.0, 254.0, 255.0
};

/*---- auxilliary functions for reading pgm/pbm headers ----------------*/
#define MAX_LINE 1000
static char * HeaderLine(FILE * file)
{
  static char  aryBuf[MAX_LINE];
  if (NULL == fgets(aryBuf,MAX_LINE,file))
    ErrHandle("HeaderLine: can't read header line (comment line long? ..)\n",-1);

  return aryBuf;
}

static FILE *  ImageOpen(char * filename)
{
  FILE * fileR;

  if (NULL == filename)
    ErrHandle("ImageOpen (read): filename is NULL",-1);
	 
  if      (filename[0]=='-' && filename[1]=='\0')  fileR = stdin;
  else if (NULL == (fileR = fopen(filename,"rb")))
         ErrHandle("ImageOpen: can't open file for reading",-1);

  return fileR;
}
static void ImageClose(FILE * file)
{
  if (file != stdin) fclose(file);
}

static int rowsLastPgm = -1;
static int colsLastPgm = -1;
static int levsLastPgm = -1;

void  LastPnmHeader (int * cols, int * rows, int * levels)
{
  *cols = colsLastPgm;
  *rows = rowsLastPgm;
  *levels = levsLastPgm;
}
/*---------------------------------------------------------------------*/

static FILE * ReadPgmHeader
  (char * filename, int * cols, int * rows,char ** pgmType, int levels)
{
   char * line;
   FILE * file = ImageOpen(filename);

   if(NULL==file) return NULL;

   line = HeaderLine(file);

   if (NULL == *pgmType)
   {         /* if pgmType==NULL then set it to whatever found in the file */
     static char pBuff[3]="  ";
     pBuff[0]=line[0];
     pBuff[1]=line[1];

     if(pBuff[0]!='P' || 
        (pBuff[1]!='2' && pBuff[1]!='5' && pBuff[1]!='4' && pBuff[1]!='1' &&
         pBuff[1]!='6')
       )
     ErrHandle1("ReadPgmHeader: unrecognised header format, file:",filename,-1);
     *pgmType = pBuff;
   }
   else if (strncmp(line,*pgmType,strlen(*pgmType)) != 0)
   {
      char msg[300];
      sprintf(msg,"ReadPgmHeader: not in expected %s format; file:",*pgmType);
      ErrHandle1(msg,filename,-1);
   }

   do                           /* skip any number of comments */
     line = HeaderLine(file);
   while('#' == line[0]);

   if (2 != sscanf(line,"%d %d",cols,rows))
 ErrHandle1("ReadPgmheader: can't find number of rows/cols;file",filename,-1);

   rowsLastPgm = *rows;
   colsLastPgm = *cols;

   if (strcmp(*pgmType,"P4") !=0  && strcmp(*pgmType,"P1")!=0 )
   {              /* P1 and P4 formats (pbm) don't store number of levels*/
     int inlevels;

     if (1 != fscanf(file,"%d",&inlevels))
       ErrHandle("ReadPgmheader: number of levels not found",-1);

     if (levels!=0 && levels != inlevels)
       ErrHandle("ReadPgmheader: number of levels incorrect",-1);
		  /* levels==0 indicates unknown number of levels */
		  /* eg. P2 has an unpredictable number of levels*/

     levsLastPgm = inlevels;
   
     {
       char c;
       if (1 != fscanf(file,"%c",&c))
	 ErrHandle("ReadPgmheader: can't find terminating char",-1);

       if (c!=' ' && c!='\t' && c!='\n')
	  ErrHandle("ReadPgmheader: header not terminated by a white space",-1);
     }
   }

   return file;
}
   
/*---------------------------------------------------------------------*/
FILE * ReadP5header(char * filename, int * cols, int * rows)
{
  char *pgmType  = "P5";
  return ReadPgmHeader(filename,cols,rows,&pgmType,255);
}

/*--------------------- I/O interface to the pbmplus package ---------------*/
static FARY * pgm2Fary(char * filename, char * pgm_Type)
{
  int cols, rows;
  char *pgmType  = pgm_Type;
  FILE * file = ReadPgmHeader(filename,&cols,&rows,&pgmType,255);
  if(file == NULL) return NULL;
  {

  FARY * fary = makFary(rows, cols); 
  int i,j;

  if ( '5' == pgmType[1]) 
  {
    unsigned char * lineBuff = malloc(cols);

    if (NULL == lineBuff)
      ErrHandle("pgm2Fary:Not enough memory for buffer",-1);
   
    for(i=0;i<rows;i++)
    {
      if (cols != fread(lineBuff, sizeof(char),cols,file))
        ErrHandle("pgm2Fary: Not enough data in image file",-1);

      for(j=0;j<cols;j++) fary->el[i][j]=byte2f[lineBuff[j]]; 
    }

    free(lineBuff);
  }
  else                          /* P2, ascii format */
  {
    for(i=0;i<rows;i++)  
      for(j=0;j<cols;j++)
        if( 1 != fscanf(file,"%f",&(fary->el[i][j])))
	  ErrHandle("pgm2Fary: Problems when reading data (short file?)",-1);
  }
  ImageClose(file);
  return fary;
  }
}

FARY * P5pgm2Fary(char * filename) {return pgm2Fary(filename,"P5"); }
FARY * Pgm2Fary(char * filename)   {return pgm2Fary(filename,NULL); }

/*---------------------------------------------------------------------*/
BARY * P5pgm2Bary(char * filename)
{
  int cols, rows;
  FILE * file = ReadP5header(filename,&cols,&rows);
  if(file == NULL) return NULL;
  {
  BARY * bary = makBary(rows, cols);

  if( cols*rows != fread(startAddrAry(bary),sizeof(char),cols*rows,file))
    ErrHandle("P5pgm2Bary: Not enough data in image file",-1);
  
  ImageClose(file);
  return bary;
  }
}
/*---------------------------------------------------------------------*/
IARY * MoveP2pgm2Iary(IARY * ary,char * filename)
{
  int cols, rows;
  char * pgmType = "P2";
  FILE * file = ReadPgmHeader(filename,&cols,&rows,&pgmType,0);
  if(file == NULL) return NULL;
  {

  IARY * iary;
  int i,j;
   
  if(NULL==ary) iary = makIary(rows, cols);
  else 
  {
    iary =ary;
    if(rowsAry(ary) != rows || colsAry(ary) != cols)
      ErrHandle("MoveP2pgmIary: file doesn't match ary size",-1);
  }

  for (i=0; i<rows; i++)
    for (j=0; j<cols; j++ )
    {
      if(1!=fscanf(file,"%d",&(iary->el[i][j])))
         ErrHandle("P2pbm2Iary: Not enough image data in input file",-1)
    }
  
  
  ImageClose(file);
  return iary;
  }
}

IARY * P2pgm2Iary(char * filename) { return MoveP2pgm2Iary(NULL,filename); }

/*---------------------------------------------------------------------*/
BARY * P4pbm2Bary(char * filename)
{

  int cols, rows , i, j, in;
  unsigned char mask;
  char * pgmType = "P4";
  FILE * file = ReadPgmHeader(filename,&cols,&rows,&pgmType,2);
  if(file == NULL) return NULL;
  {

  BARY * bary = makBary(rows, cols);

  for (i=0; i<rows; i++)
    for (j=0; j<cols; )
    {
      if(EOF == (in=getc(file))) 
         ErrHandle("P4pbm2Bary: Not enough image data in input file",-1)
			   
      for(mask=128; (mask>0) && j!=cols ; mask >>= 1)
	bary->el[i][j++]=((unsigned char)in & mask)?255:0;
    }
  
  ImageClose(file);
  return bary;
  }
}
/*---------------------------------------------------------------------*/
RGBARY * MoveP6ppm2RGBary(RGBARY * RGBary, char * filename)
{
  int cols, rows;
  char * pgmType = "P6";
  FILE * file = ReadPgmHeader(filename,&cols,&rows,&pgmType,255);
  if(file == NULL) return NULL;
  {
  if(rowsAry(RGBary) != rows || colsAry(RGBary) != cols)
    ErrHandle("MoveP6ppm2RGBary: file doesn't match ary size",-1);

  if(cols*rows!=fread(startAddrAry(RGBary),sizeof(t_rgb),cols*rows,file))
    ErrHandle("MoveP6ppm2RGBary: Not enough data in image file",-1);

  ImageClose(file);
  return RGBary;
  }
}
YUVARY * P6ppm2YUVary(char * filename)
{
   YUVARY * yuv = (YUVARY*)P6ppm2RGBary(filename); 
   yuv->ary_type    = YUVMATRIX;
   return yuv;
}

RGBARY * P6ppm2RGBary(char * filename)
{

  int cols, rows;
  char * pgmType = "P6";
  FILE * file = ReadPgmHeader(filename,&cols,&rows,&pgmType,255);

  if(file == NULL) return NULL;
  {
  RGBARY * RGBary = makRGBary(rows, cols);

  if(cols*rows!=fread(startAddrAry(RGBary),sizeof(t_rgb),cols*rows,file))
    ErrHandle("P6ppm2RGBary: Not enough data in image file",-1);
  
  ImageClose(file);
  return RGBary;
  }
}
 
/*---------------------------------------------------------------------*/
ARY *Px2Ary(char * filename)
{
#define IsType(type)  (strcmp(pgmType,type)==0)

   int dummy;
   char * pgmType = NULL;

   FILE* f = ReadPgmHeader(filename,&dummy,&dummy,&pgmType,0);
	 if (NULL==f) {return NULL;}
	 ImageClose(f);

   if     (IsType("P2")) return (ARY*) P2pgm2Iary(filename);
   else if(IsType("P4")) return (ARY*) P4pbm2Bary(filename);
   else if(IsType("P5")) return (ARY*) P5pgm2Bary(filename);
   else if(IsType("P6")) return (ARY*) P6ppm2RGBary(filename);
   else return NULL;
}
/*---------------------------------------------------------------------*/
static void reflectLeftSide(FARY *image, int xImagMin);	
static void reflectRightSide(FARY *image, int xImagMax);
static void reflectTop(FARY *image, int yImagMin);
static void reflectBot(FARY *image, int yImagMax);

#define MIN(x,y) ((x)<=(y) ? (x) : (y))
#define MAX(x,y) ((x)>=(y) ? (x) : (y))

/*---------------------------------------------------------------------*/
FARY * P5pgmRoi2Fary (char * filename, int * roi, int fullImage, int extend)
{
  int cols , rows; 

  FILE *file = ReadP5header(filename,&cols,&rows);
  return P5pgmRoi2Fary1(file,cols,rows, roi,fullImage,extend,NULL);
}

/*---------------------------------------------------------------------*/
FARY * P5pgmRoi2Fary1 (FILE * file, int cols, int rows, int * roi, 
                              int fullImage, int extend, void * mem)
{
   FARY * image;
   int xReadFrom, xReadTill, yReadFrom, yReadTill;
   int xRoiMin, xRoiMax, yRoiMin, yRoiMax;

   int xImagMin = 0;
   int yImagMin = 0;
   int xImagMax = cols-1;
   int yImagMax = rows-1;

   if (fullImage)
   {
     roi[0]=xImagMin;
     roi[1]=xImagMax;
     roi[2]=yImagMin;
     roi[3]=yImagMax;
   }

/*---- establish RoI size, image and RoI intersection --------*/
   xRoiMin = roi[0] - extend;
   xRoiMax = roi[1] + extend;
   yRoiMin = roi[2] - extend;
   yRoiMax = roi[3] + extend;

   xReadFrom = MAX(xRoiMin,xImagMin);       /* compute the intersection */
   xReadTill = MIN(xRoiMax,xImagMax);
   yReadFrom = MAX(yRoiMin,yImagMin);
   yReadTill = MIN(yRoiMax,yImagMax);

   
/*-- check if mirroring can fill  RoI pixels outside the image---*/
   {
     int  xRoiHalf = (xRoiMax - xRoiMin)/2;
     int  yRoiHalf = (yRoiMax - yRoiMin)/2;
     if ( (xReadFrom-xRoiMin) > xRoiHalf ||
	  (xRoiMax-xReadTill) > xRoiHalf ||
	  (yReadFrom-yRoiMin) > yRoiHalf ||
	  (yRoiMax-yReadTill) > yRoiHalf 
        )
     ErrHandle("Insuficient part of the Region of interest inside the image",-1);
   }

  image = (FARY *)makmemary(yRoiMin,yRoiMax,xRoiMin,xRoiMax,FLOATMATRIX,mem);

/*---- read the part of images that falls into RoI -----------*/
 {                  
   int charsPerLine = xReadTill-xReadFrom+1;
   unsigned char * lineBuff = malloc(charsPerLine);
   int imageCols = xImagMax-xImagMin+1;
   int i,j;

   if (NULL == lineBuff)
    ErrHandle("P5pgmRoi2Fary1:Not enough memory for buffer",-1);

   if (0 != fseek(file,yReadFrom * imageCols + xReadFrom,SEEK_CUR))
    ErrHandle("P5pgmRoi2Fary1:Not enough data in image file (fseek failed)",-1);

   for(j=yReadFrom;j<=yReadTill;j++)
   {
     if(charsPerLine != fread(lineBuff,sizeof(char),charsPerLine,file))
     ErrHandle("P5pgmRoi2Fary1: Not enough data in image file-fread failed",-1);

     for(i=xReadFrom;i<=xReadTill;i++)
       image->el[j][i]=byte2f[lineBuff[i-xReadFrom]]; 

     fseek(file,imageCols-(xReadTill-xReadFrom+1),SEEK_CUR);
   }

   free(lineBuff);
  }

/*------------- mirror ------------------------------------------*/
  if(xRoiMin<xImagMin) reflectLeftSide(image,xImagMin);
  if(xRoiMax>xImagMax) reflectRightSide(image,xImagMax);
  if(yRoiMin<yImagMin) reflectTop(image,yImagMin);
  if(yRoiMax>yImagMax) reflectBot(image,yImagMax);

  ImageClose(file);
  return image;
}



static void reflectLeftSide(FARY *image, int xImagMin)
{
  int i,j;

  for(i=image->lb1;i<=image->ub1;i++)
    for(j=image->lb2;j<xImagMin;j++)
      image->el[i][j] = image->el[(abs)(i)][(abs)(j)];
}

static void reflectRightSide(FARY *image, int xImagMax)
{
  int i,j;

  for(i=image->lb1;i<=image->ub1;i++)
    for(j=(xImagMax+1);j<=image->ub2;j++)
      image->el[i][j] = image->el[i][(2*(xImagMax+1))-2-j];
}

static void reflectTop(FARY *image, int yImagMin)
{
  int i,j;

  for(i=image->lb1;i<yImagMin;i++)
    for(j=image->lb2;j<=image->ub2;j++)
      image->el[i][j] = image->el[(abs)(i)][(abs)(j)];
}

static void reflectBot(FARY *image, int yImagMax)
{
  int i,j;

  for(i=(yImagMax+1);i<=image->ub1;i++)
    for(j=image->lb2;j<=image->ub2;j++)
      image->el[i][j] = image->el[(2*(yImagMax+1))-2-i][j];
}

   
/*---------- Read Ary from a format that can store ArY of any type --------*/
static void * ReadAry1(ARY *ary,char *filename)
{
  int lb1,ub1,lb2,ub2;
  t_aryEl type;
  int itype, elSize;

  FILE * file = ImageOpen(filename);
  char * line = HeaderLine(file);

  if (2 != sscanf(line,"ARY%d %d",&itype,&elSize))
    ErrHandle("ReadAry: not in UOS_ARY format",-1);
  type = itype;  /* avoids warnings with pointer type */

  if (4 != sscanf(HeaderLine(file),"%d %d %d %d",&lb2,&ub2,&lb1,&ub1))
    ErrHandle("ReadAry: ARY dimensions not found",-1);

  if(0==ary)   /* a new ary */
  {
    if (type != USER_DEFINED) ary = makary(lb1,ub1,lb2,ub2,type);
    else                      ary = consAry1(lb1,ub1,lb2,ub2,elSize,NULL);
  }
  else         /* read into existing */
  {
    if(ary->lb1!=lb1 || ary->ub1!=ub1 || ary->lb2!=lb2 || ary->ub2!=ub2 ||
       type!=ary->ary_type)
    ErrHandle("ReadAry: into existing: type/size mismatch",-1);
  }
  
  if (NULL == ary)
    ErrHandle("ReadAry: ARY allocation failed. (unknown type?)",-1);
 
  {
    int n_of_bytes = rowsAry(ary)*colsAry(ary)* elSize;   
    fread((char*)(ary->el[ary->lb1])+elSize*ary->lb2, 
                                  sizeof(char),n_of_bytes ,file); 
  }
  ImageClose(file);
  return(ary);
}

/*---------------------------------------------------------------------*/
void * ReadAry(char *filename)             {return ReadAry1(0,filename);}
void   ReadInAry(ARY * ary,char *filename) {       ReadAry1(ary,filename);}

#include <string.h>
#include "strGM.h"
/* #define ISWORD(str)   (strcasecmp(word,str) == 0)  */
#define ISWORD(str)   (stricmp(word,str) == 0) 
static int ReadCompressedBlock(IARY *fary,  FILE *f);
static int ReadUnCompressedBlock(IARY *fary,  FILE *f);

/*---------------------------------------------------------------------*/

IARY * SB7toIary(char *filename)
{
 IARY* iary;
 int   compressed = -1;    /* set 'undefined'  values */
 int   Fversion=0;
 int   Dversion = 0;
 int   rows=0;
 int   cols=0;

 FILE * f = ImageOpen(filename);

 while (1)          /* read the header: */
 {
	 char  lin[512];           /* buffers */
	 char  word[128];

   fgets(lin, 512, f);
   sscanf(lin, "%s", word);

   if      (ISWORD("end")) break;

   else if (ISWORD("File_version"))sscanf(lin, "%*s = %i",&Fversion);
   else if (ISWORD("Data_version"))sscanf(lin, "%*s = %i",&Dversion);
   else if (ISWORD("Height"))      sscanf(lin, "%*s = %i",&rows);
   else if (ISWORD("Width"))       sscanf(lin, "%*s = %i",&cols);
   else if (ISWORD("ST-7")){
     sscanf(lin, "ST-7 %s Image", word);
     compressed = ISWORD("Compressed");
   }
 }

 if (rows <= 0 || 
     cols <= 0 || 
     compressed == -1 || 
     Fversion != 3 || Dversion != 1)
  {
   fclose(f);
   ErrHandle("Can open SB7 image",2);
  }

 fseek(f, 2048L, SEEK_SET);
 iary = makIary(rows,cols);

 if (compressed)ReadCompressedBlock(iary, f);
 else           ReadUnCompressedBlock(iary,f);

 return iary;
}
    
/*-- Modified from 

   sbig2m.c: loads SBIG image file

   Documentation: CCD Camera Operating Manual for the Model ST-7 and ST-8,
   Santa Barbara Instrument Group, 1482 East Valley Rd., CA., First Revision,
   1994

  (c) Radim Sara (sara@cmp.felk.cvut.cz), October 27, 1997

 */


typedef unsigned short int PixelT ;

static int ReadCompressedBlock(IARY *iary,  FILE *f)
/* Reads a compressed block from SBIG image file. The file position indicator
   must be set to the beginning of the datablock. */
{
 int rows = rowsAry(iary);
 int cols = colsAry(iary);
 int r;

 for(r = 0; r < rows; r++) /* for all rows */
 {
	 PixelT pixel;
   int c = 0;
   unsigned short int llgth;
    /* read the line length in bytes */
   fread(&llgth, 2, 1, f);
   if (llgth >= cols*2) /* uncompressed */
   {

	/*fprintf(stderr,"Uncompressed row %d. length %d (>=%d)\n",r,llgth,cols*2);*/
     for(c=0; c<cols; c++) /* read as is */
     {
       fread(&pixel, 2, 1, f);
       iary->el[r][c] = (int) pixel;
     }
    }
   else /* compressed */
   {
     fread(&pixel, 2, 1, f);           /* read the first pixel */
     llgth -= 2;
     iary->el[r][c] = (int) pixel;

     for(c=1; c<cols; c++)
     {                        /* read the first byte of the next pixel */
	     unsigned char byte;
       fread(&byte, 1, 1, f);
       llgth--;
       if (byte == 0x80)
		   {
			   fread(&pixel, 2, 1, f); /* read a 2-byte pixel */
			   llgth -= 2;
			 }
			 else 
			 {
			   pixel +=    (signed char)byte;
			 } 	

       iary->el[r][c] = (int) pixel;
		 }

		 if (llgth != 0)
			{
			 fclose(f);
			 ErrHandle("Image line longer than expected, wrong format?",2);
			}
    }/* of  else compressed */
  }  /* of   for r */
  return 0;
}

static int ReadUnCompressedBlock(IARY *iary,  FILE *f)
{
  int rows = rowsAry(iary);
  int cols = colsAry(iary);
  int r,c;

 for(r = 0; r < rows; r++) /* for all rows */
  for(c = 0; c < cols; c++) /* for all rows */
	{
	  PixelT pixel;
    fread(&pixel, 2, 1, f);
    iary->el[r][c] = (int) pixel;
	}
			 /* printf("Reading uncompressed data not yet implemented. Sorry.\n");*/
  return 0;
}

#include "aryio.h"
/* short int  se(short int a) {return 256*a%256 + a/256;} */
Win3xBMP ConsBMPHeader(int rows, int cols)
{
  int palette_size = 256;
  Win3xBMP w;
  w.header.imageFileType[0]= 'B';
  w.header.imageFileType[1]= 'M';
  w.header.fileSize        = sizeof(w) + palette_size + rows*cols;
  w.header.reserved1 = 0;
  w.header.reserved2 = 0;
  w.header.imageDataOffset = sizeof(w) + palette_size; 

  w.info.headerSize  = sizeof(w.info);
  w.info.x           = 0; /*? */
  w.info.imageWidth  = cols;
  w.info.imageHeight = rows;
  w.info.numberOfImagePlanes = 1;
  w.info.bitsPerPixel =        8;
  w.info.compressionMethod =   0;
  w.info.sizeOfBitmap  =       rows*cols;
  w.info.horzResolution=        0;
  w.info.vertResolution=        0;
  w.info.numColorsUsed=         256;
  w.info.numSignificantColors = 0;

  if(NULL == (w.palette = malloc(sizeof(t_rgbx)*w.info.numColorsUsed)))
  {  fprintf(stderr,"ConsBMPHeader: can't alloc for palette\n");
     exit(-1);
  }
  {
  int i;
  for( i = 0; i<w.info.numColorsUsed; i++)
    w.palette[i].st.r= w.palette[i].st.g= w.palette[i].st.b=i;
	}
  
  return w;
}
Win3xBMP ReadBMPHeader(FILE * f)
{
  Win3xBMP w;
  fread(w.header.imageFileType,sizeof(w.header.imageFileType),1,f);
  fread(&w.header.fileSize,sizeof(w.header.fileSize),1,f);
  fread(&w.header.reserved1,sizeof(w.header.reserved1),1,f);
  fread(&w.header.reserved2,sizeof(w.header.reserved2),1,f);
  fread(&w.header.imageDataOffset,sizeof(w.header.imageDataOffset),1,f);

  fread(&w.info.headerSize,sizeof(w.info.headerSize),1,f);
  fread(&w.info.x,sizeof(w.info.x),1,f);
  fread(&w.info.imageWidth,sizeof(w.info.imageWidth),1,f);
  fread(&w.info.imageHeight,sizeof(w.info.imageHeight),1,f);
  fread(&w.info.numberOfImagePlanes,sizeof(w.info.numberOfImagePlanes),1,f);
  fread(&w.info.bitsPerPixel,sizeof(w.info.bitsPerPixel),1,f);
  fread(&w.info.compressionMethod,sizeof(w.info.compressionMethod),1,f);
  fread(&w.info.sizeOfBitmap,sizeof(w.info.sizeOfBitmap),1,f);
  fread(&w.info.horzResolution,sizeof(w.info.horzResolution),1,f);
  fread(&w.info.vertResolution,sizeof(w.info.vertResolution),1,f);
  fread(&w.info.numColorsUsed,sizeof(w.info.numColorsUsed),1,f);
  fread(&w.info.numSignificantColors,sizeof(w.info.numSignificantColors),1,f);

 if(w.info.numColorsUsed == 0) w.info.numColorsUsed = 1<< w.info.bitsPerPixel;

  if(NULL == (w.palette = malloc(sizeof(t_rgbx)*w.info.numColorsUsed)))
  {  fprintf(stderr,"ReadBMPHeader: can't alloc for palette\n");
     exit(-1);
  }

  fread(w.palette, sizeof(t_rgbx), w.info.numColorsUsed, f);

  return w;
} 

void PrintBMPHeader(Win3xBMP w)
{
   printf("Type:      %c%c\n",w.header.imageFileType[0],w.header.imageFileType[1]);
   printf("File size:   %d\n",w.header.fileSize);
   printf("Reserved:   %d %d\n",w.header.reserved1, w.header.reserved2);
   printf("Offset:    %d\n",w.header.imageDataOffset);
   printf("HeaderSize: %d \n",w.info.headerSize);
   printf("Planes: %d",w.info.numberOfImagePlanes);
   printf("Width x Height: %d x %d\n",w.info.imageWidth,w.info.imageHeight);
   printf("Bits/pixel    : %d\n",w.info.bitsPerPixel);
   printf("Compression   : %d\n",w.info.compressionMethod);
   printf("Colours   : %d\n",w.info.numColorsUsed);

   {
   int i;
   for(i=0;i<w.info.numColorsUsed;i++)
   { printf("c%d (rgb)   : %d %d %d\n",i,w.palette[i].st.r,w.palette[i].st.g,
                                    w.palette[i].st.b);
   }
   }
}  

static int look_up[256]; 
static int CheckBMPGray(Win3xBMP  w)
{
  if (w.info.bitsPerPixel != 8)     return 1;
  if (w.info.compressionMethod != 0)return 1 ;
  if (w.info.numColorsUsed > 256)  return 1; 
  {
    int i;
		/*
    for (i=0;i<w.info.numColorsUsed;i++)
      if(i != w.palette[i].st.r ||
         i != w.palette[i].st.g ||
         i != w.palette[i].st.b) return 1;
				 */
    for (i=0;i<w.info.numColorsUsed;i++)
      if( w.palette[i].st.r != w.palette[i].st.g ||
          w.palette[i].st.r != w.palette[i].st.b) return 1;
      else look_up[i]=w.palette[i].st.r;
  }
  return 0;
}


BARY * BMP2Bary(char * filename)
{
  FILE * f  = ImageOpen(filename);
	if(f  == NULL) return NULL;

  {
		Win3xBMP  w  = ReadBMPHeader(f);
    BARY *    b = makBary(w.info.imageHeight,w.info.imageWidth);
   
   if(CheckBMPGray(w)) 
      ErrHandle("BMP2Bary: can read only 8bit/pixel gray uncompr. BMPs",-1);
 {
  int i,j;
  int padding = colsAry(b)%4;
  for(i=b->ub1;i>=b->lb1;i--)
  {
    for(j=b->lb2;j<=b->ub2;j++)
      b->el[i][j] =  look_up[getc(f)];
  
    if (padding>0)
    for(j=0;j<4-padding; j++) 
      fgetc(f);
  }
  }
  
  ImageClose(f);
  return b;
  }
}

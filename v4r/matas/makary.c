/*----------------- dynamic 2-dimensional array lib -------------------*/
/*
  1-Sep-93, G. Matas
    - redefined using generic allocation consAry1


  1992 Written initially by Ron Cain at SRI, this version by Radu Horaud.
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)makary.c   2.7     95/05/23 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include "ary.h"
#include <stdio.h>

int  AryElemSize ( t_aryEl ary_type);

/*---------------------------------------------------------------------*/
ARY *makary(int firstrow,int lastrow, int firstcol, int lastcol, t_aryEl arytype)
{
  ARY *matrix =		
   consAry1(firstrow,lastrow,firstcol,lastcol,AryElemSize(arytype),NULL);

  if (NULL!=matrix) matrix->ary_type = arytype;
  return matrix;
}

/*---------------------------------------------------------------------*/
ARY *makmemary
 (int firstrow,int lastrow, int firstcol, int lastcol, t_aryEl arytype, void *addr)
{
  ARY *matrix =		
    consAry1(firstrow,lastrow,firstcol,lastcol,AryElemSize(arytype),addr);

  if (NULL!=matrix) matrix->ary_type = arytype;
  return matrix;
}


/*-------------- make various ary's   ------------------------*/
CARY *makCary (int rows,int  cols)
{ return ( (CARY*)makary(0, rows-1, 0, cols-1, BYTEMATRIX) ); }

BARY *makBary (int rows, int cols)
{ return ( (BARY*)makary(0, rows-1, 0, cols-1, UBYTEMATRIX) ); }

SARY *makSary (int rows, int cols)
{ return ( (SARY*)makary(0, rows-1, 0, cols-1, SHORTINTMATRIX) ); }

IARY *makIary (int rows, int cols)
{ return ( (IARY*)makary(0, rows-1, 0, cols-1, INTEGERMATRIX) ); }

FARY *makFary (int rows, int cols)
{ return ( (FARY*)makary(0, rows-1, 0, cols-1, FLOATMATRIX) ); }

DARY *makDary (int rows, int cols)
{ return ( (DARY*)makary(0, rows-1, 0, cols-1, DOUBLEMATRIX) ); }

PARY *makPary (int rows, int cols)
{ return ( (PARY*)makary(0, rows-1, 0, cols-1, POINTERMATRIX) ); }

RGBARY *makRGBary (int rows, int cols)
{ return ( (RGBARY*)makary(0, rows-1, 0, cols-1, RGBMATRIX) ); }

XRGBARY *makXRGBary (int rows, int cols)
{ return ( (XRGBARY*)makary(0, rows-1, 0, cols-1, XRGBMATRIX) ); }

RGBXARY *makRGBXary (int rows, int cols)
{ return ( (RGBXARY*)makary(0, rows-1, 0, cols-1, RGBXMATRIX) ); }

FRGBARY *makFRGBary (int rows, int cols)
{ return ( (FRGBARY*)makary(0, rows-1, 0, cols-1, FRGBMATRIX) ); }

YUVARY *makYUVary (int rows, int cols)
{ return ( (YUVARY*)makary(0, rows-1, 0, cols-1, YUVMATRIX) ); }

RGIARY *makRGIary (int rows, int cols)
{ return ( (RGIARY*)makary(0, rows-1, 0, cols-1, RGIMATRIX) ); }

USARY *makUSary (int rows, int cols)
{ return ( (USARY*)makary(0, rows-1, 0, cols-1, USHORTINTMATRIX) ); }


/*------------- return sizeof of ARY element --------------------*/
int  AryElemSize ( t_aryEl ary_type)
{
  int elSize;

  switch (ary_type)
  {
    case BYTEMATRIX:
    case UBYTEMATRIX:        elSize = sizeof(char);      break;
    case SHORTINTMATRIX:     elSize = sizeof(short int); break;
    case INTEGERMATRIX:      elSize = sizeof(int);       break;
    case FLOATMATRIX:        elSize = sizeof(float);     break;
    case DOUBLEMATRIX:       elSize = sizeof(double);    break;
    case POINTERMATRIX:      elSize = sizeof(void *);    break;
    case RGBMATRIX:          elSize = sizeof(t_rgb);     break;
    case XRGBMATRIX:         elSize = sizeof(t_xrgb);     break;
    case RGBXMATRIX:         elSize = sizeof(t_rgbx);     break;
    case FRGBMATRIX:         elSize = sizeof(t_frgb);     break;
    case RGIMATRIX:          elSize = sizeof(t_rgI);     break;
    case YUVMATRIX:          elSize = sizeof(t_yuv);     break;

    default:      
       fprintf(stderr,"AryElementSize: unknown ARY type\n");
       elSize=-1;
  }

  return elSize;
}

/*------------- return sizeof of ARY element --------------------*/
int  sizeAry(ARY*ary)
{
  return rowsAry(ary)*colsAry(ary)*AryElemSize(ary->ary_type);
}

/*------------- return sizeof of ARY element --------------------*/
char*   infoAry(ARY*ary)
{
  static char info[]="xxxxARY   123456 123456 123456 123456";
  sprintf(info,"%7s   %6d %6d %6d %6d",
	    nameAry(ary),ary->lb1,ary->ub1,ary->lb2,ary->ub2);
  return info;
}
/*------------- return sizeof of ARY element --------------------*/
char *  nameAry( ARY * ary)
{
  static char * name;

  switch (ary->ary_type)
  {
    case BYTEMATRIX:         name = "CARY";   break;
    case UBYTEMATRIX:        name = "BARY";   break;
    case SHORTINTMATRIX:     name = "SARY";   break;
    case INTEGERMATRIX:      name = "IARY";   break;
    case FLOATMATRIX:        name = "FARY";   break;
    case DOUBLEMATRIX:       name = "DARY";   break;
    case POINTERMATRIX:      name = "PARY";   break;
    case RGBMATRIX:          name = "RGBARY";   break;
    case XRGBMATRIX:         name = "XRBGARY";   break;
    case RGBXMATRIX:         name = "RGBXARY";   break;
    case FRGBMATRIX:         name = "FRGBARY";   break;
    case RGIMATRIX:          name = "RGIARY";   break;
    case YUVMATRIX:          name = "YUVARY";   break;

    default:                 name = "unknown"; break;
  }

  return name;
}

/*------------- return sizeof of ARY element --------------------*/
int isEqBoundsAry(ARY* a1, ARY * a2)
{
  return 
	  a1->lb1==a2->lb1 && a1->ub1==a2->ub1 &&
	  a1->lb2==a2->lb2 && a1->ub2==a2->ub2;
}

/*--------------------------------------------------------------*/
int     isInAry(ARY* ary,int x, int y)
{
   return 
	  x>=ary->lb1 && x<=ary->ub1 &&
	  y>=ary->lb2 && y<=ary->ub2;
}

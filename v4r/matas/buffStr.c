/*------ string library extensions ----------------------------------- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1993, George Matas.                                     | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)buffStr.c	1.5	95/02/01 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "strGM.h"

/*---------- constr a string ---------------------------*/

static char * buffStart = NULL;
static char * firstEmpty= NULL;
static int    buffSize  = 0;

void   ResBufStr(char * buffer, int size)
{
  buffStart = buffer;
  firstEmpty= buffer;
  buffSize  = size;
}
int  AppBufStr(char * format, ...)
{
  va_list args;

  if (NULL == buffStart) 
    {fprintf(stderr,"AppBufStr: NULL buffer");exit(-1);}
  if (firstEmpty-buffStart>=buffSize)
    {fprintf(stderr,"AppBufStr: buffer already full");exit(-1);}

  va_start(args,format);
  firstEmpty += vsprintf(firstEmpty,format,args);
  
  if (firstEmpty-buffStart>=buffSize)
    {fprintf(stderr,"AppBufStr: buffer overflow!");exit(-1);}

  va_end(args);

  return 0;
}

int    SizeBufStr(void)    {return firstEmpty-buffStart;}
/*char *     BufStr(void)  {return  buffStart;}*/

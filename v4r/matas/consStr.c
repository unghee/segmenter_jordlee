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
static char sccsid[]="@(#)consStr.c	1.5	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "strGM.h"

/*---------- constr a string ---------------------------*/
#define BUF_SIZE 100000

char * ConsStr(char * format, ...)
{
  va_list args;

  int converted;
  char buff[BUF_SIZE];  

  va_start(args,format);
  converted = vsprintf(buff,format,args);

  if (converted >= BUF_SIZE)
  {
    fprintf(stderr,"string to long in ConsStr\n");
    exit(-1);
  }

  va_end(args);

  return DupStr(buff);
}

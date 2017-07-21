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
static char sccsid[]="@(#)suffStr.c	1.1	94/10/26 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "strGM.h"

/*-------- check/ return string suffix ---------------  */
char * SuffStr(char * s)
{
   char * period ;
   if (NULL == ( period = strrchr(s,'.'))) return NULL;
   return period+1;
}

int    IsSuffStr(char * s, char * suff)
{
   char * suffStart = SuffStr(s);
   if(NULL == suffStart) return 0;

   return 0==strcmp(suffStart,suff);
}

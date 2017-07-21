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
static char sccsid[]="@(#)dupStr.c	1.7	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "strGM.h"

/*-------- duplicate a string into allocated memory ------------*/
char * DupStr(char * s)
{
   char * copy;
   if (NULL == s) { return NULL;}
   if (NULL == (copy = malloc(strlen(s) + 1)))
   {
     fprintf(stderr,"malloc returned NULL in strdup\n");
     exit(-1);
   }
   strcpy(copy,s);
   return copy;
}


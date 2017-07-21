/*------- LL Double linked list library: text file functions ----------- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1992, 1993, George Matas.                               | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+
*/

/*  based on a link library by Duane Morse                                  */

/* $Id: LLfile.c,v 1.4 1996/06/11 16:19:46 ees2gm Exp $ */

/* Modifications:
 * $Log: LLfile.c,v $
 * Revision 1.4  1996/06/11 16:19:46  ees2gm
 * LL2file recognises '-' as stdout
 *
 * Revision 1.3  1995/09/08  08:14:16  ees1rm
 * Comment bracket was fixed.
 *
 * Revision 1.2  1995/09/07  08:33:09  ees1rm
 * strlen() prototype added.
 *
*/

/*--------------------------------------------------------------------------*/
static char sccsid[]="@(#)LLfile.c	8.4	94/12/20 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdlib.h>
#include <stdio.h>
#include <string.h> /*strlen()*/
#include "LL.h"

/*--------------------------------------------------------------------------*/
#define MAX_LINE_LENGTH 20000
static t_LL AuxFile2LL(char * name, int exitOnErr)
{
  char  buffer[MAX_LINE_LENGTH];
  t_LL list  = ConsLL();
  FILE *file = fopen(name,"r");

  if (NULL==file)
  {
    if (!exitOnErr) return list;            /* return empty list */
    else             AbortLL_M("File2LL","can't open file for read");
  }

  while(fgets(buffer,MAX_LINE_LENGTH,file))
      InsLastLLf(list,strlen(buffer)+1,buffer);

  fclose(file);
  return list;
}

t_LL FileNoExit2LL(char * name)   {return AuxFile2LL(name,0);}
t_LL File2LL(char * name)         {return AuxFile2LL(name,1);}

/*--------------------------------------------------------------------------*/

void LL2File(t_LL list, char * name)
{
  FILE *file = (strcmp("-",name)) ? fopen(name,"w") : stdout ;
  char * str;

  if (NULL==file) AbortLL_M("LL2File","can't open file for write");

  ForeachLL_M(list,str)
    fputs(str,file);

  fclose(file);
}

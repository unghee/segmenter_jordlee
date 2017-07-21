/*------ Command line parser - function for list parameters --- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1993, George Matas.                                     | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+
*/

/*
    22-Feb-93, J. Matas
       - created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)94/07/21 g.matas@ee.surrey.ac.uk 2.1 optionList.c";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <string.h>

#include "LL.h"
#include "option.h"
#include "optionPriv.h"

t_LL  OptionLL(char *name,char * comment)
{
  char ** option;
  t_LL list = ConsLL();
  char * str;
  char buff[1000];
  int i;
   
  IsInitialized();

  if (NULL != (option=GetOption(name,-1)))
  {   
    for(i=1; NULL !=option[i] ;i++)
      InsLastLLf(list,strlen(option[i])+1,option[i]);
  }

  /* a more efficient version with %n instead of strlen didn't work ??gcc */

  sprintf(buff,"%s (",OptName(name)); 
  ForeachLL_M(list,str)
    sprintf(buff+strlen(buff),"%s ",str);
  sprintf(buff+strlen(buff),") [?] %s",comment);

  Usage(DupStr(buff));

  return list;
}

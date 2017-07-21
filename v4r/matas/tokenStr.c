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
static char sccsid[]="@(#)tokenStr.c	1.8	94/12/19 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "strGM.h"

/*------------------------------------------------------------------- */
/*         count the number of tokens in a string             */
/*         tokens are separated by a sequence of 'delimiters' */

int TokensInStr(char * str,char * delimiters)
{
  int i=0;
  char *Str= DupStr(str);
  char *token;


  token = strtok(Str,delimiters);
  while(NULL!=token){
    token = strtok(NULL,delimiters);
     i++;
   }
  free (Str);
  return i;
}

/*------------------------------------------------------------------- */
static char * tokenizeDels = " \t\n";
void  SetDelimitersStr(char * del)   {tokenizeDels =DupStr(del);}
char* GetDelimitersStr(void)         {return tokenizeDels;}

/*------------------------------------------------------------------- */
int TokenNumStr(char * str)
{
  int i;
  char *Str  = DupStr(str);
  char *token=  strtok(Str,tokenizeDels);

  for(i=0;NULL!=token;i++)
    token = strtok(NULL,tokenizeDels);

  free (Str);
  return i;
}

/*------------------------------------------------------------------- */
char* FindTokenStr(char * str, char * tok)
{
  char *Str  = DupStr(str);
  char *token=  strtok(Str,tokenizeDels);

  while(NULL!=token)
  {
    if(0==stricmp(tok,token)) break;
    token = strtok(NULL,tokenizeDels);
  }

  free (Str);

  if (NULL==token) return NULL;
  else             return str+(token-Str);
}

/*------------------------------------------------------------------- */
char* FindNthTokenStr(char * str,int tokenNo)
{
  char *Str   = DupStr(str);
  char *token = strtok(Str,tokenizeDels);

  for(tokenNo--;NULL!=token && tokenNo>0; tokenNo--)
     token = strtok(NULL,tokenizeDels);

  free (Str);

  if (NULL==token) return NULL;
  else             return str+(token-Str);
}

/*------------------------------------------------------------------- */
char *GetNthTokenStr(char * str, int tokenNo)
{
  char *Str   = DupStr(str);
  char *token = strtok(Str,tokenizeDels);

  for(tokenNo--;NULL!=token && tokenNo>0; tokenNo--)
     token = strtok(NULL,tokenizeDels);

  token = DupStr(token);
  free (Str);

  return token;
}
/*------------------------------------------------------------------- */
char ** TokenizeStr(char * str)
{
  int i=0;
  char *Str= DupStr(str);
  char ** arr = malloc(sizeof(char*)*(1+TokensInStr(str,tokenizeDels)));
  

  if (NULL==arr) {fprintf(stderr,"malloc failed in TokenizeStr\n");exit(-1);} 

  arr[i] = DupStr(strtok(Str,tokenizeDels));
  while(NULL != arr[i++])
    arr[i] = DupStr(strtok(NULL,tokenizeDels));

  free (Str);

  return arr;
}
/* int IsTokenPrefStr(char *token, char * str); place to a separate file
   to allow for its existnce in more than one library*/

/*------------------------------------------------------------------- */
int  SizeTokenizedStr(char ** tkn)
{
  int j=0;
  while(*tkn++) j++;
  return j;
}

/*------------------------------------------------------------------- */
void DestTokenizedStr(char ** tkn)
{
  char ** start = tkn;

  while(NULL != *tkn) 
    free(*tkn++);

  free(start);
}


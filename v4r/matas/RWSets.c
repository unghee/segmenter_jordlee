/*----------- Read/Write a LL list of gf sets ---------------------------*/
/* +-------------------------------------------------------------------+ */
/* | Copyright 1994, George (Jiri) Matas  (g.matas@ee.surrey.ac.uk)    | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
static const
char rcsid[] = "$Id: RWSets.c,v 1.4 1996/02/02 18:12:55 ees2gm Exp $";
typedef char _r_foo[sizeof(rcsid)]; 
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "LL.h"
#include "strGM.h"
#include "gfLLio.h"
#include "gfLL.h"
 

static t_LLSet ReadOneSet(char * SetName);         /* read-in one set */
/*-------------------------------------------------------------------------*/
/* if Setname == NULL read all set else read just the one specified */

static t_LL AuxRead(char *filename, char * Setname, int Pref)
{
  t_LL Sets = ConsLL();
  t_LLSet Set;
  char * line;

  if (OpenPGfileR(filename)) 
    MyPtrErr(ConsStr("ERR in AuxRead(%s): Can't open file",filename));

  while (NULL != (line=fgetline(NULL))){
    if (IsTokenPrefStr(SET_STA_TOKEN, line))
    {
      char * foundName;

      if (NULL != (foundName=GetNthTokenStr(line,2)))
      {
	if(Setname==NULL ||              /* if Setname NULL read every set */
	   (Pref==0 && !strcmp(foundName,Setname)) ||
		                	 /* if Pref==0 check the full name */
           (Pref==1 && !strncmp(foundName,Setname,strlen(Setname)))
		          /* else check if Setname is a prefix of foundName*/
	  )
	{
	  if (NULL != (Set=ReadOneSet(foundName)))
	    InsLastLL(Sets, Set);
	  else
	   MyPtrErr(ConsStr(
	   "ERR in AuxRead(%s): ReadSet(%s) returned NULL",filename,foundName));
	}

	free(foundName);     /* foundName not NULL here */
      }
    else
      MyPtrErr(ConsStr("ERR in AuxRead(%s): set without name",filename));
   }
  }
  ClosePGfileR();
  return Sets;
}

static t_LLSet AuxReadSet(char * filename, char * SetName, int Pref)
{
  t_LL Sets =AuxRead(filename, SetName,Pref);
  t_LLSet  *pSet;

  if (NULL == Sets) return NULL;
  pSet = FirstElmLL(Sets);
  if (!IsElmLL(pSet)) return NULL;     /* not a single set in the list*/

  DelElmLL(pSet);
  DestLLSets(Sets);                 /* Dest all set but the one returned*/

  return * pSet;
}

/*------------------------------------------------------------------------*/
t_LL ReadLLSets(char *filename)
{ return AuxRead(filename, NULL,0);}

t_LLSet ReadLLSet(char * filename,char * SetName)
{
  t_LLSet set=AuxReadSet(filename, SetName,0);
  if (NULL==set)
    MyPtrErr(ConsStr("ERR in ReadLLSet(%s): set/file not found",filename));
  return set;
}

t_LLSet ReadPrefLLSet(char * filename,char * SetName)
{ return AuxReadSet(filename, SetName,1); }

/*------------------------------------------------------------------------*/
t_LLSet ReadOneSet(char * SetName)
{
  char *line, * tmp_str ;
  t_LLSet Set;

/*   Set = (t_LLSet) malloc (sizeof(t_LLSet*)); */   /* didn't work on DEC */
/*   Set = (t_LLSet) malloc (sizeof(struct s_LLSet));  */
  Set = (t_LLSet) malloc (sizeof(*Set)); 
  Set->id = DupStr(SetName);

/*---  Read geometric structures and attribute descritions     --- */

  Set->GeomStruct = ConsLL();
  Set->Attributes = ConsLL();

  while (NULL != (line=fgetline(NULL)) && !IsTokenPrefStr(FORMAT_TOKEN,line))
  {                /* the section will be terminated by the format string */
    if (IsTokenPrefStr(GEOM_TOKEN,line))         /* geometric structure */
    {
      if (NULL != (tmp_str=FindNthTokenStr(line,2)))
	InsLastLLf(Set->GeomStruct,strlen(tmp_str)+1,tmp_str);
      else
	MyPtrErr(ConsStr(
	  "ERR in ReadOneSet(%s) when reading Geom struct string",SetName));
    }
    else if (IsTokenPrefStr(ATTRIB_TOKEN,line))   /* sttribute definition */ 
    {
      if (NULL != (tmp_str=FindNthTokenStr(line,2)))
	InsLastLLf(Set->Attributes,strlen(tmp_str)+1,tmp_str);
      else
	MyPtrErr(ConsStr(
	  "ERR in ReadOneSet(%s) when reading Geom struct string",SetName));
    }
    else                       /* neither geomStruct, attrib nor format */
      MyPtrErr(ConsStr("ERR in ReadOneSet(%s):"
		       "unknown keyword between @Set and Format",SetName));
  }
    
/*----------- Get the format string ------------------------------- */
  if (IsTokenPrefStr(FORMAT_TOKEN,line)&&
      (NULL != (tmp_str = FindNthTokenStr(line,2)))
     )
    Set->format = DupStr(tmp_str);
  else 
    MyPtrErr(ConsStr(
        "ERR in ReadOneSet(%s) when reading  format string",SetName));
  
/*------------- Get the set data ------------------------------------*/
  Set->data = ConsLL();
  {
  int l;
  while ((NULL != (line=fgetline(&l)) && !IsTokenPrefStr(SET_END_TOKEN,line)))
      InsLastLLf(Set->data,l+1,line);
  
  }
  return Set;
}

/*------------------------------------------------------------------------*/
void * WriteSet (void * Set);
int WriteLLSets(char * filename, t_LL Sets)
{
  if (OpenPGfileW(filename))
     MyErr(ConsStr("ERR in WriteSets(%s): Can't open file",filename));
  ApplyLL(Sets, WriteSet);

  ClosePGfileW();
  return 0;
}

int WriteLLSet(char * filename, t_LLSet Set)
{
  if (OpenPGfileW(filename))
     MyErr(ConsStr("ERR in WriteSets(%s): Can't open file",filename));

  WriteSet(&Set);

  ClosePGfileW();
  return 0;
}

/*------------------------------------------------------------------------*/
void *  WriteSet(void * vSet)
{
  char * GeomStruct, *data, *s, *Attribute;
  t_LLSet Set = *(t_LLSet*)vSet;

  fputline(s=ConsStr("%s %s",SET_STA_TOKEN,Set->id));
  free(s);

  ForeachLL_M(Set->GeomStruct, GeomStruct)
  {
    fputline(s=ConsStr("%s %s",GEOM_TOKEN,GeomStruct));
    free(s);
  }

  ForeachLL_M(Set->Attributes, Attribute)
  {
    fputline(s=ConsStr("%s %s",ATTRIB_TOKEN,Attribute));
    free(s);
  }

  fputline(s=ConsStr("%s %s",FORMAT_TOKEN,Set->format));
  free(s);

  ForeachLL_M(Set->data, data)
    fputline(data);

  fputline(SET_END_TOKEN);
  return NULL;
}

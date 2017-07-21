/*---------- Command line parser - core -------------------------------- */
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
/*
    23-Jun-93, J. Matas
       - put under SCCS control, versID string replaced by sccsID

    12-Mar-92, J. Matas
       - function calls that had sprintf(optBuf, ... ) as a parameters
	 replaced by ConsStr; 1. to orig. was not ANSI conformant 
	 (assuming that sprintf returns char *) 2. optBuf was made
	 public (unsage, difficult to check if not overwritten)

    1-Mar-93, J. Matas
       - function OptionIf() added

    18-Feb-93, J. Matas 
       - created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)93/06/23 g.matas@ee.surrey.ac.uk 1.2 option.c";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "option.h"
#include "optionPriv.h"

static int     optInitialized = 0;   /* has the parser been initialized ?*/
static char ** currentArgv = NULL;   /* array of unprocessed options and pars*/
static int   * pcurrentArgc= 0;      /* number of unprocessed opt.  */
static int     originalArgc= 0;      /* no. of option on the comm. line */

static char ** used;                 /* array of processed options and pars. */
static int     usedc;                /* no. of processed opt. and pars.  */

#define MAX_ERRC     50
static char * errv[MAX_ERRC];        /* array of error messages */
static int     errc = 0;             /* no. of error messages */

#define MAX_OPTIONS 100
static char *  usage[MAX_OPTIONS];   /* array containing complete help/usage */
static int     usagec;               /* no. of entries in the help      */

#define MAX_DEPEND 50
static char *  dep[MAX_DEPEND];      /* array of dependency information */
static int     depc;                 /* current no. of dependencies */


/*------------ Usage Registration --------------------------------------*/ 
void Usage(char * s)
   /* append a string into the usage array */
{
  if (usagec+1 >= MAX_OPTIONS)
  {
    fprintf(stderr,"Too many options! See ees2gm\n"); exit(-1);
  }
  usage[usagec]  =s;
  usagec++;
  usage[usagec]=NULL;
}


/*------------- Useful utilites ----------------------*/
#define EMPTY_LENGTH 6
static char emptyName[EMPTY_LENGTH] ="arg_1";
char * OptName(char * name)
  /* Fix option name, forced by the usage of "" options (options without */
  /* specifier);the empty string must be converted to something visible  */
{
  static char nameBuff[100];
  if (name[0] =='\0')              /* empty string, convert  */
  {
    strcpy(nameBuff,emptyName);
    emptyName[EMPTY_LENGTH-2]++;
  }
  else                             /* else put the '-' prefix */
    sprintf(nameBuff,"-%s",name);

  return DupStr(nameBuff);       /* should be freed it somewhere */
}


static int IsPar(char *s) 
  /* defines which comm. line arguments are option specifiers and 
   * which are consider to be parameters 
   * PARAMETERs must not start with -, unless it is a single character 
   * '-' (allowing typical definition of pipe in/out) or the '-' is followed
   * by a digit (assuming a negative number 
   * Consequently, OPTIONS start with a '-' followed by at least one  letter
  */
   
{
  if (s[0]!='-' || s[1]=='\0' || isdigit(s[1])) return 1;
  else                         return 0;
}

/*-- does string 's' match option 'name'(passed in without leading -) ?----*/
static int MatchOption(char *s, char * name)
{
    if ((!IsPar(s)) && (!strcmp(name,&s[1]))) return 1;
    else return 0;
}

/*--- get the number of pars following an option in position 'pos' ---*/
static int NumOfPars(char * name, int pos)
{
  int i;

  for (i=pos+1;i<*pcurrentArgc;i++)
    if (!IsPar(currentArgv[i])) break;
  
 return i-(pos+1);
}
/*------------ Error Checking and  Registration ----------------------------*/ 
void IsInitialized(void )
{
  if (optInitialized == 0)
  {
    fprintf(stderr,"Command line processing not initialized!\n");
    exit(-1);
  }
}
   
static void ErrRegister(char * s)    /* append string s to error array */
{
  if (errc >= MAX_ERRC)
  {
    fprintf(stderr,"Too many errors!\n"); exit(-1);
  }
  errv[errc++]=s;
}

/*-------------------------------------------------------------*/
static int MultipleRequest(char * name)
/* check if option name hasn't been already processed, ie.
 * if OptionXX name hasn't been called more than once for
*/
{
 int i;
 int name_length = strlen(name);

 if ('\0' == name[0]) return 0; /* <NO_OPT> can be multiply  requested */

 for(i=0;i<usagec;i++)      /* assumes usage always starts with -option */
   if(   !strncmp(&usage[i][1],name,name_length)
      && usage[i][name_length+1]==' ')
   {
     ErrRegister(ConsStr("Option %s processed more than once",OptName(name)));  
     return 1;
   }
 return 0;
}

/*-------------------------------------------------------------*/
static int NotEnoughPars(char * name, int expected, int pars)
{ 
  if (expected >pars)
  {
   ErrRegister(ConsStr("Not enough parameters for option %s",OptName(name)));
   return 1;
  }
  return 0;
}

/*-------------------------------------------------------------*/
static void ErrCompulsory(char * name)
{
 ErrRegister(ConsStr("Missing compulsory option %s",OptName(name)));
}

/*-------------------------------------------------------------*/
void ErrIncorrectType(char * name)
{
 ErrRegister(ConsStr(
	    "Incorrect type of parameters in option %s",OptName(name)));
}

/*-------------------------------------------------------------*/
static int MultipleSpec(char * name)
  /* check if this option hasn't appeared more then once on the comm. line */
{
  int i;

 if ('\0' == name[0]) return 0; /* <NO_OPT> can be multiply  spec. */

  for(i=0;i<usedc;i++)
    if (MatchOption(used[i],name))
    {
      ErrRegister(ConsStr( "Option %s used more then once",OptName(name)));
      return 1;
    }

  return 0;
}

/*------------ useful private functions --------------------------------*/
static int FindOption(char *name)
{
  int i;

  if (name[0] != '\0')
  {
    for(i=1;i<*pcurrentArgc; i++)
    if (MatchOption(currentArgv[i],name)) return i; 
  }
  else   /* for "" option any parameter is the value */
  {
    for(i=1;i<*pcurrentArgc; i++)
      if(IsPar(currentArgv[i])) return i-1; 
  }
  
  return -1;
}
static int FindUsedOption(char * name)
{
  int i;
  for(i=0;i<usedc;i++)
    if (MatchOption(used[i],name)) return i; 

  return -1;
}
  

/*-------------------------------------------------------------*/
static int numArgs = 0;   /* count the number of empty options */
static char **  MoveOption(int position,int pars,char * name)
{
  int i;
 
  if('\0' == name[0])                /* "" doesn't have a specifier, adjust*/
  {
    numArgs++;
    position++;
    pars--;
  }

 
  for(i=position;i<=position+pars;i++)  /* copy option + pars into used */
    used[usedc++] = currentArgv[i];
  used[usedc] = NULL;  /* NULL terminations enables to find out */
                       /* the number of args passed out (useful for list */

  for(i=position+pars+1	;i< *pcurrentArgc;i++)  /* shift option in input */
    currentArgv[i-pars-1]=currentArgv[i];
  
  * pcurrentArgc -= (pars+1)	;

  if('\0' == name[0]) pars++;       /* "" doesn't have a specifier, adjust */

  return &used[usedc-pars-1];
}

/*-------------------------------------------------------------*/
char ** GetOption(char * name,int expectedPars)
{
  int position = FindOption(name);
  int pars; 
 
  if (MultipleRequest(name))               return NULL;
  if (-1==position)                         return NULL;

  pars = NumOfPars(name,position);

  if (-1 == expectedPars ) expectedPars=pars;
    /* -1 pars means as many pars as can be found (useful for lists)*/

  if (NotEnoughPars(name,expectedPars,pars)  )
  {
     MoveOption(position,pars,name);
     return NULL; 
  }
  if (MultipleSpec(name))
  {
     MoveOption(position,expectedPars,name);
     return NULL; 
  }

  return MoveOption(position,expectedPars,name);
}

/*------------ Public funtions ---------------------------------------*/
void OptionInit(char ** orgv, int * orgc)
{

   currentArgv = orgv;
   pcurrentArgc= orgc;
   originalArgc = *orgc;

  if (optInitialized == 1)
  {
    fprintf(stderr,
         "Command line processing  re-initializition not permitted!\n");
    exit(-1);
  }
   if (NULL == (used = malloc(sizeof(char *) * originalArgc)))
     {fprintf(stderr,"Not enough memory in Init\n"); exit(-1);};

  optInitialized=1;
}

/*-------------------------------------------------------------*/
void OptionInitCopy(char ** orgv, int * orgc)
{
  int i;
  char ** copiedOrgv;
  static int copiedOrgc ;
 
  copiedOrgc = * orgc;

  if (NULL == (copiedOrgv = malloc(sizeof(char *) * (*orgc))))
  {fprintf(stderr,"Not enough memory in Init\n"); exit(-1);};
    
  for(i=0;i<*orgc;i++)
    copiedOrgv[i]=orgv[i];

  OptionInit(copiedOrgv,&copiedOrgc);
}

/*-------------------------------------------------------------*/
/* unprocessed stuff not considered an error */
static int optionLeftOK = 0;
void OptionLeftOK(void) { optionLeftOK = 1;}


void OptionCheck(void)
{
  IsInitialized();
  {
    int printUsage = 0;
    int i;
    int help = OptionToggle("help",0,"print out usage info");

    if (errc != 0)
    {
      fprintf(stderr,"\n");
      fprintf(stderr,"Errors detected during  option parsing:\n");
      for(i=0;i<errc;i++) fprintf(stderr,"    %s\n",errv[i]);

      printUsage=1;
    }

    if ((*pcurrentArgc > 1) && !optionLeftOK)
    {
      fprintf(stderr,"Unknown (unprocessed) options and parameters:\n  ");
      for(i=1;i<*pcurrentArgc;i++) fprintf(stderr,"%s ",currentArgv[i]);
      fprintf(stderr,"\n");

      printUsage=1; 
    }

    if (help || printUsage)
    {
      fprintf(stderr,"Usage: %s [options]\n",currentArgv[0]);
      for(i=0;i<usagec;i++)
	 fprintf(stderr,"    %s\n",usage[i]);
      fprintf(stderr,"Dependencies:\n");
      for(i=0;i<depc;i++)
	 fprintf(stderr,"    %s\n",dep[i]);
      fprintf(stderr,"\n");
      exit(-1);
    }
  }
}


/*-------------------------------------------------------------*/
char** OptionUsage(void)
{
  IsInitialized();
  return usage;
 }
/*------------------------------ dependency Check ------------------------*/ 
static void DepRegister(char * s)
{
  if (depc+1 >= MAX_OPTIONS)
  {
    fprintf(stderr,"Too many dependencies! See ees2gm\n"); exit(-1);
  }
  dep[depc]  =s;
  depc++;
  dep[depc]=NULL;
}

/*-------------------------------------------------------------*/
static int OptionNumbers(char * options)
{
  int i;
  int matches = 0;
  char optionUsed[100];
  char opt[200];


  sprintf(opt," %s ",options);
  for(i=0;i<usedc;i++)
    if(!IsPar(used[i]))
    {
      sprintf(optionUsed," %s ",&used[i][1]);      
      if(NULL != strstr(opt	,optionUsed)) matches++;
    }
   
  for(i=1;i<*pcurrentArgc; i++)
    if(!IsPar(currentArgv[i]))
    {
      sprintf(optionUsed,"%s ",&currentArgv[i][1]);      
      if(NULL != strstr(opt,optionUsed)) matches++;
    }

  return matches;
}

int  OptionOnCommLine(char * name)
{
  return ( OptionNumbers(name) > 0);
}

void OptionDependXor(char * xor_opt)
{
  int matches;

  IsInitialized();

  DepRegister(ConsStr("Options '%s' are mutually exclusive",xor_opt));
  

  matches = OptionNumbers(xor_opt);
  if(matches>1)
    ErrRegister(ConsStr("%d of mutally exclusive options '%s' specified",
               matches,xor_opt));
}
/*-------------------------------------------------------------*/
void OptionIf(int enableCond, char * depend, char * comment)
{
  IsInitialized();

  DepRegister(ConsStr( "Option %s can be used only if: %s",
              OptName(depend), comment)); 

  if (enableCond) return;

  if (-1 != FindUsedOption(depend) || -1 != FindOption(depend))
    ErrRegister(ConsStr( "Option %s can be used only if: %s",
         OptName(depend), comment));
} 

void OptionMultIf(int enableCond, char * depend, char * comment)
{
  int matches;

  IsInitialized();

  DepRegister(ConsStr("Options '%s' can be used only if: %s", depend, comment));

  if (enableCond) return;

  matches = OptionNumbers(depend);
  if(matches>0)
    ErrRegister(ConsStr( 
		"Options '%s' can be used only if: %s", depend, comment));
}

/*-------------------------------------------------------------*/
void OptionDependIf(char * cond, int enableVal, int val, char * depend)
{
  IsInitialized();

  DepRegister(ConsStr(
              "Option %s can be used only when option %s is %d('%c')",
              OptName(depend), OptName(cond), enableVal,enableVal)); 

  if (val == enableVal) return;

  if (-1 != FindUsedOption(depend) || -1 != FindOption(depend))
    ErrRegister(ConsStr( 
        "Option %s can be used only when option %s is %d(char: '%c')",
         OptName(depend), OptName(cond), enableVal,enableVal));
} 
/*-------------------------------------------------------------*/
void OptionCompulsory(char * name)
{
  IsInitialized();

  DepRegister(ConsStr( "Option %s is compulsory", OptName(name))); 

  if (-1 == FindUsedOption(name) && -1 == FindOption(name))
     ErrCompulsory(name);
}
/*-------------------------------------------------------------*/
void OptionCompulsoryArgs(int num)
{
  IsInitialized();

  DepRegister(ConsStr("At least %d argument(s) must be specified",num));

  if(num > numArgs)
    ErrRegister(ConsStr(
        "Only %d argument(s) were found on the command line",numArgs));
}

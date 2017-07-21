/*---------- Low level I/O for gf sets read/write  -----------------------*/
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
char rcsid[] = "$Id: PGio.c,v 1.5 1996/02/02 18:12:55 ees2gm Exp matas $";
typedef char _r_foo[sizeof(rcsid)];
/*-----------------------------------------------------------------------*/

#include <ctype.h>
#include "gfLL.h"
#include "gfLLio.h"

static  char buff[MAX_LINE_LENGHT];

/*---------------    Input    ----------------------------------*/
static  int FileOpenedR =0;
static  FILE * fileR;
int OpenPGfileR(char *fileRname)
{
 if (*fileRname=='-')  fileR = stdin;
 else
   if ((fileR  = fopen(fileRname, "r"))==NULL)
       MyErr("Can't open fileR for reading");

 FileOpenedR =1;
 return 0;
}
int ClosePGfileR(void)
{
 if (!FileOpenedR ) MyErr("Can't close fileR (fileR not opened)");
 if (fileR!= stdin)
   fclose(fileR);
 FileOpenedR = 0;
 return 0;
}

#define CONT_TABS 7

char * fgetline(int * length)
{
 char *line=buff;
 
  do
  { 
    do{    
      line = fgets(line ,MAX_LINE_LENGHT, fileR);
      if (NULL==line) return NULL;
    }
    while (('\n' == *line)||(buff[0]==';')) ;
			/* suppress empty lines and lines starting with a ';' */
    while (*line ) line++ ;                               /* find end of line */

    if ((line-buff)>MAX_LINE_LENGHT)
      MyPtrErr("ERR in fgetline - merged line too long");
    
    if (*(line-2) != '\\') break;         /* no continuation line, exit */

    line-=1;                                 /* step back over       \n */
    *(line-1)=' ';                      /* replace \ with a space       */
    *line = '\0';     /* terminate the string just in case there was no */
		      /* continuation line (should not happen)          */

    {
    int i;
      for(i=0;i<CONT_TABS;i++) /* swallow the continuation tabs         */
      {
        int c = getc(fileR);
        if(EOF==c)            /* no tabs after \? (shouldn't happen)    */
        {
  	  fprintf(stderr,"fgetline: No continuation tabs after \\!\n");
	  break;
	}
	if(!isspace(c))    /* not enough tabs? */
	{
	  ungetc(c,fileR);
	  break;
	}
      }
    } /* continuation tabs finished */
  }
  while(1);                          /* exit trough the break */

  /* in the gf package the terminal \n was repalaced by a ' ' */
  if(*(line-1)=='\n') *(--line)='\0';     /* get rid of terminal \n*/

  if (length != NULL)
    *length  = line - buff;
  return buff;  
}

/*---------------    Output    ----------------------------------*/
static  FILE * fileW;
static  int FileOpenedW =0;

int OpenPGfileW(char *filename)
{
 if (*filename=='-')  fileW = stdout;
 else
   if ((fileW  = fopen(filename, "w"))==NULL)
       MyErr("Can't open file for writting");

 FileOpenedW =1;
 return 0;
}
/*---------------------------------------------------------------*/
int ClosePGfileW()
{
 if (!FileOpenedW ) MyErr("Can't close file (file not opened)");
 if (fileW != stdout)
   fclose(fileW);
 FileOpenedW = 0;
 return 0;
}
/*---------------------------------------------------------------*/

static unsigned int break_length = 75;
void SetBreakLength(unsigned int l) { break_length = l; }

/*---------------------------------------------------------------*/
/*  
   fputline copies *s the the fileW and
     - all white space chars are converted to a space
     - strings longer then  break_length are broken into a number of
       lines
*/
/*---------------------------------------------------------------*/
char * fputline(char *s)
{
  char * outc   = s;
  char * lastnl = s;

  if (!FileOpenedW) MyPtrErr("Can't write  (file not open)");

  while ('\0' != *outc)
  {
    if((outc-lastnl> break_length) &&(isspace(*outc)))
    {         /* break a line if line long and current char is white */

      /*skip whites (don't break if ONLY whites remain in the string)*/
      while ('\0' != outc && isspace(*outc)) outc++;

      if('\0' != *outc)    
      {                      /* break a line, non-white chars remain */
	int i;
        fputs("\\",fileW);
        fputc('\n', fileW);
	for(i=0;i<CONT_TABS;i++)
	  fputc(' ',fileW);
	lastnl=outc-CONT_TABS;
      }
    }
    else                /* normal char ouput (ie. no line breaks) */
    {  
      if (isspace(*outc)) fputc(' ',fileW);    /* convert whites  */
      else              fputc(*outc,fileW);    /* output non-white*/
      outc++;
    }

  }  /* of while ('\0'... */

  
  fputc('\n', fileW);

  return s;
}  

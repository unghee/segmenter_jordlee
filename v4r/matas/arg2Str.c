/*------ string library extensions ----------------------------------- */
/* Copyright 1993, George Matas                   (g.matas@ee.surrey.ac.uk) */
/*
    27-Oct-93, J. Matas
       - created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)arg2Str.c	1.3	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "strGM.h"

/*-------- merge argv into a single string       ------------*/
char * Argv2Str(int argc, char **argv)
{
#define BUFF_SIZE 1000
   static char buffer[BUFF_SIZE];
   int length = 0;
   int i;

   for(i=0; i< argc; i++)
   {
     length += strlen(argv[i]);

     if(length >= BUFF_SIZE)
     {
       fprintf(stderr,"Argv2Str: out of buffer space\n");
       exit(-1);
     }
     strcat(buffer,argv[i]);
     strcat(buffer," ");
   }

   return buffer;
}


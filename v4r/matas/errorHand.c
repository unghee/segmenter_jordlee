/*---------------------------------------------------------------------*/
/*
  G. Matas, 1-Sep-93
      created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)errorHand.c	2.4	95/02/22 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include "ary.h"

int exitOnError = 0;
void ExitOnAryError(void )   { exitOnError=1; }
void ReturnOnAryError(void ) { exitOnError=0; }


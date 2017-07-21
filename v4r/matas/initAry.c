/*---------------  Initilise Ary to a given value -------------------------*/
/* Author:  George (Jiri) Matas                     g.matas@ee.surrey.ac.uk */
/*--------------------------------------------------------------------------*/
static const
char rscid[] = "$Id: initAry.c,v 1.4 1996/01/24 17:54:55 ees1rm Exp $";
   typedef char _s_foo[sizeof(rscid)];/*stop gcc warning: unused var rscid*/ 
/*----------------- created 17-Jun-93 --------------------------------------*/

#include "ary.h"

DEF_ARY_Initialise(Bary,BARY,unsigned char)
DEF_ARY_Initialise(Iary,IARY,int)
DEF_ARY_Initialise(USary,USARY,unsigned short int)

DEF_ARY_Initialise(Fary,FARY,float)
DEF_ARY_Initialise(Dary,DARY,double)

DEF_ARY_Initialise(Pary,PARY,void *)

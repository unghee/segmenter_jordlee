/*--- LL Double linked list library: checks internal consistency ------- */
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
/*--------------------------------------------------------------------------*/
static char sccsid[]="@(#)LLconsis.c	8.3	94/12/20 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include "LL.h"
#include "linkLL.h"

static t_LLsize maxSize = -2;

void ConsistentLL(t_LL list)
{
  l_list * link;
  l_list * head;

  if(NULL==list)
  {
    fprintf(stderr,"the t_LL pointer == NULL, list corrupted\n"); return;
  } 

  head = list2link(list);

  if(head->size != 0)       fprintf(stderr,"head->size not equal 0.\n");

  if(head->forward == NULL)
  {
    fprintf(stderr,"head->forward == NULL!, list corrupted \n");return;
  }

  if(head->backward == NULL)
  {
    fprintf(stderr,"head->backward == NULL!, list corrupted \n");return;
  }

  if(head->backward==head->forward && head->backward!=head)
  {
    fprintf(stderr,
       "head->forward == head->backward (true for empty lists only), but\n"
       "head->forward != head (should be == for an empty list)\n"
       "list corrupted \n");
    return;
  }

  {
    t_LLsize sizeF=0;
    t_LLsize sizeB=0;

    ForeachLink_M(head,link)     if(++sizeF > maxSize) break;
    ForeachLinkBack_M(head,link) if(++sizeB > maxSize) break;

    if(sizeB != sizeF)
    {
      fprintf(stderr,
      "different size for b/f directions. forward: %ld  backward: %ld\n"
      "list corrupted\n", sizeF,sizeB);
      return;
    }

    if(sizeF>maxSize) fprintf(stderr,"list size >maxSize(%ld)\n",maxSize);
  }
 
  {
    int i=0;
    ForeachLink_M(head,link)
    {
      i++;
      if (0 == link->size) 
	fprintf(stderr,"element no: %d has link->size == 0.\n",i);
    }
  }

  {
    int i=0;
    ForeachLink_M(head,link)
    {
      i++;
      if(link->forward->backward != link) 
        fprintf(stderr,"link->forward->backward != link at elem no: %d\n",i);
      if(link->backward->forward != link) 
        fprintf(stderr,"link->backward->forward != link at elem no: %d\n",i);
    }
     if(head->forward->backward != head) 
        fprintf(stderr,"head->forward->backward != head\n");
     if(head->backward->forward != head) 
        fprintf(stderr,"head->backward->forward != head\n");
  }
}

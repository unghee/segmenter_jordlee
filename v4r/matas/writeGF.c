#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "strGM.h"
#include "canny.h"

#ifndef NO_GF

/* static char X_Y_Def[]          ="pointset rep x y";
static char X_Y_mag_Def[]      ="pointset rep x y magnitude";
static char X_Y_angle_mag_Def[]="pointset rep x y angle magnitude";
*/
static char X_Y_angle_Def[]    ="pointset rep x y angle ";

#define OUT_BUF_SIZE 10000
char buff[OUT_BUF_SIZE];
void WriteStringsInGF(char * name,t_graphGR graph, FARY * grad_dir)
{
  WriteStringsSiInGF(name,graph,grad_dir,0);
}

void WriteStringsSiInGF(char * name,t_graphGR graph, FARY * grad_dir, int size)
{

  t_LL    AllSets   =ConsLL();
  t_LL    GeomStruct=ConsLL();
  t_LLSet Chains    =ConsLLSet("CannyLists",GeomStruct,"pointset");
  t_LLSet Juncts    =ConsLLSet("Junct_Pts",0,"Pointdot Rep IDCannyLists");

  if(size>0)
  {
    t_LLSet Decr = ConsLLSet("Description",ConsLL(),"Rep Text");

    sprintf(buff,"2 Dimension %d",size);
    InsLastLLf(Decr->data,strlen(buff)+1,buff);

    InsLastLL(AllSets,Decr);
  }

  if(grad_dir != NULL)
     InsLastLLf(GeomStruct,sizeof(X_Y_angle_Def),X_Y_angle_Def);
 /*
  if(!direct_out&&mag_out)
     InsLastLLf(GeomStruct,sizeof(X_Y_mag_Def),X_Y_mag_Def);
  if(direct_out&&!mag_out)
     InsLastLLf(GeomStruct,sizeof(X_Y_angle_Def),X_Y_angle_Def);
  if(direct_out&&mag_out)
     InsLastLLf(GeomStruct,sizeof(X_Y_angle_mag_Def),X_Y_angle_mag_Def);
  */

  {
    int i=0;
    t_edgeGR pEdge;

    ForeachLL_M(graph.edges,pEdge)
     pEdge->marker=++i;
  }
  
  {
    t_edgeGR pEdge;
    t_nodeGR pNode;
    t_edgeLink pL;

    ForeachLL_M(graph.nodes,pNode)
    {
      if(!not_all_junct ||
         ((SizeLL(pNode->out)>1) &&
	        ((SizeLL(pNode->out)>2 ||         /* if 2, not a loop */
	         (t_edgeLink) FirstElmLL(pNode->out) != 
	         (t_edgeLink) LastElmLL(pNode->out))
         )
	 )
	)
      {
	t_i2D * pPoint = FirstElmLL((t_LL)pNode->att);

	ResBufStr(buff,OUT_BUF_SIZE);
	AppBufStr("%d %d %ld ",pPoint->y ,pPoint->x,SizeLL(pNode->out));

  ForAdjEdgesGR_M(pNode,pL,pEdge) 
	   AppBufStr("%d ",pEdge->marker);


	InsLastLLf(Juncts->data,SizeBufStr()+1,buff);
      }
    }
  }
  {
    t_LL String;
    t_edgeGR pEdge;
    ForeachLL_M(graph.edges,pEdge)
    {
      t_i2D * pPoint;

      String=pEdge->att;

      ResBufStr(buff,OUT_BUF_SIZE);
      AppBufStr("%ld ",SizeLL(String));

      ForeachLL_M(String,pPoint)
      {
	      AppBufStr("%d %d ",pPoint->y,pPoint->x);
        if(grad_dir)  
          AppBufStr("%5.3f ",grad_dir->el[pPoint->x][pPoint->y]);
      } 
      InsLastLLf(Chains->data,SizeBufStr()+1,buff);
    }
  }

  InsLastLL(AllSets,Juncts);
  InsLastLL(AllSets,Chains);

  WriteLLSets(name,AllSets);
}
#endif


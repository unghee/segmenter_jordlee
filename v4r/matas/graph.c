#include <stdlib.h>
#include <string.h>
#include "graph.h"
#include "strGM.h"
/*-----------------------------------------------------------------------*/
/* Copyright 1994, G. Matas 
*/
/*-----------------------------------------------------------------------*/
static const char sccsid[]="@(#)94/05/13 g.matas@ee.surrey.ac.uk 2.2 graph.c";
typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

/*--------------------------------------------------------------------------*/
t_graphGR ConsGenGR(t_typeGR type)
{
  t_graphGR graph;

  graph.nodes = ConsLL();
  graph.edges = ConsLL();
  graph.type  = type;

  return graph;
}

/*--------------------------------------------------------------------------*/
t_graphGR ConsDirGR(void)   { return ConsGenGR(DIRECTED);}
t_graphGR ConsGR(void)      { return ConsGenGR(UNDIRECTED);}

/*--------------------------------------------------------------------------*/
t_nodeGR InsNodeIdGR(t_graphGR graph, char *id)
{
  t_nodeGR  pNode;

  if (graph.type == UNDIRECTED)
  {
    struct s_nodeUGR
    {
      struct s_nodeGR n;
      struct s_LL  listHead[1];
    }node;

    pNode = InsLastLL(graph.nodes,node);
    pNode->in  = InitLL(&(((struct s_nodeUGR*)pNode)->listHead[0]));
    pNode->out = pNode->in;
  }
  else
  {
    struct s_nodeDGR
    {
      struct s_nodeGR n;
      struct s_LL  listHead[2];
    }node;

    pNode = InsLastLL(graph.nodes,node);
    pNode->in = InitLL(&(((struct s_nodeDGR*)pNode)->listHead[0]));
    pNode->out= InitLL(&(((struct s_nodeDGR*)pNode)->listHead[1]));
  }

  pNode->id     = (NULL==id)?NULL:DupStr(id);

  return pNode;
}

t_nodeGR InsNodeGR(t_graphGR graph)      {return InsNodeIdGR(graph,NULL);}
/*--------------------------------------------------------------------------*/

#ifdef t_atNGR
t_nodeGR InsNodeIdAttGR(t_graphGR graph, char * id, void * att)
{ 
  t_nodeGR pNode =  InsNodeIdGR(graph,id); 
  pNode->att    = att;
  return pNode;
}

t_nodeGR InsNodeAttGR(t_graphGR graph,  void * att)
  { return InsNodeIdAttGR(graph,NULL,att); }
#endif

/*--------------------------------------------------------------------------*/
t_edgeGR InsEdgeIdGR(t_graphGR graph, t_nodeGR src, t_nodeGR targ,char * id)
{

  t_edgeGR pEdge;

  {
    struct s_edgeGR  edge;
    pEdge = InsLastLL(graph.edges,edge);
  }

  pEdge->id     = (NULL==id)?NULL:DupStr(id);

  pEdge->source.node = src;
  pEdge->target.node = targ;
  
  pEdge->source.me   = pEdge;
  pEdge->target.me   = pEdge;

  LinkInsLastLL(src->out,pEdge->target.node);   /* cross link */ 
  LinkInsLastLL(targ->in,pEdge->source.node);

  return pEdge ;
}

t_edgeGR InsEdgeGR(t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1)
  { return InsEdgeIdGR(graph,pN0,pN1,NULL);}

/*--------------------------------------------------------------------------*/
#ifdef t_atEGR
t_edgeGR InsEdgeIdAttGR(t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1,
			char * id, void * att)
{ 
  t_edgeGR pEdge = InsEdgeIdGR(graph,pN0,pN1,id);
  pEdge->att = att;
  return pEdge;
}

t_edgeGR InsEdgeAttGR(t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1, void * att)
  { return InsEdgeIdAttGR(graph,pN0,pN1,NULL,att);}
#endif

/*--------------------------------------------------------------------------*/
t_edgeGR DelEdgeGR(t_edgeGR pEdge)
{
  if (NULL != pEdge->id) free(pEdge->id);
  /* HACK, MZ, 2006-07-17: the attributes need to be deleted too!
   * I asume the attribute is a list. */
  DestLL(pEdge->att);
  /* HACK END */
  UnlinkLL(&pEdge->source.node);
  UnlinkLL(&pEdge->target.node);
  return DelElmPrLL(pEdge);
}

/*--------------------------------------------------------------------------*/
t_nodeGR DelNodeGR(t_nodeGR pNode)
{
  t_edgeLink pEdgeLink,pEdgePrev;

  if (NULL != pNode->id) free(pNode->id);
  /* HACK, MZ, 2006-07-17: the attributes need to be deleted too!
   * I asume the attribute is a list. */
  DestLL(pNode->att);;
  /* HACK END */

  ForeachLL_M(pNode->in,pEdgeLink)
  {
    pEdgePrev=PrevElmLL(pEdgeLink);
    DelEdgeGR(pEdgeLink->me); 
    pEdgeLink=pEdgePrev;
  }
  
  ForeachLL_M(pNode->out,pEdgeLink)
  {
    pEdgePrev=PrevElmLL(pEdgeLink);
    DelEdgeGR(pEdgeLink->me); 
    pEdgeLink=pEdgePrev;
  }

  return DelElmPrLL(pNode);
}

/*--------------------------------------------------------------------------*/
void DestGR(t_graphGR graph)  
{
  t_nodeGR pNode;

  ForeachLL_M(graph.nodes,pNode)
    pNode=DelNodeGR(pNode);

  DestLL(graph.nodes);   /* the list is empty, but the head must be freed*/
  DestLL(graph.edges);   /* dtto */
}
/*--------------------------------------------------------------------------*/
char *  EdgeIdGR(t_edgeGR  pEdge) { return pEdge->id;}
char *  NodeIdGR(t_nodeGR  pNode) { return pNode->id;}

/*--------------------------------------------------------------------------*/
#ifdef t_atNGR
t_atNGR   NodeAttGR(t_nodeGR pNode) { return pNode->att;}
t_atNGR * NodepAttGR(t_nodeGR pNode) { return &(pNode->att);}
#endif
#ifdef t_atEGR
t_atEGR   EdgeAttGR(t_edgeGR pEdge) { return pEdge->att;}
t_atEGR * EdgepAttGR(t_edgeGR pEdge) { return &(pEdge->att);}
#endif

/*--------------------------------------------------------------------------*/
t_nodeGR FindNodeGR(t_graphGR graph, char * id)
{
  t_nodeGR  pNode =NULL;
  ForeachLL_M(graph.nodes,pNode)
    if (strcmp(pNode->id,id)==0) break;
  
  return pNode;
}
/*--------------------------------------------------------------------------*/
t_edgeGR FindEdgeGR(t_graphGR graph, char * id)
{
  t_edgeGR pEdge =NULL;
  ForeachLL_M(graph.edges,pEdge)
    if (strcmp(pEdge->id,id)==0) break;
  
  return pEdge;
}
/*--------------------------------------------------------------------------*/
int NumNNConnectGR(t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1)
{
  int           num_connections=0;
  t_edgeLink    plink;
  t_nodeGR      pnode;

  ForAdjNodesGR_M(pN0, plink, pnode)
   {
    num_connections += ( pnode == pN1 );
  }
  return num_connections;
}



/*--------------------------------------------------------------------------*/
void FullPrintGR(t_graphGR graph)
  { PrintGR(graph,1,1);}

void PrintGR(t_graphGR graph, int nodes, int edges)
{
  t_nodeGR pNode;

  printf("Graph type: %s\n",graph.type==UNDIRECTED?"undirected":"directed");

  ForeachLL_M(graph.nodes,pNode)
  {
    t_edgeLink pLink;
    t_edgeGR pEdge;
    t_nodeGR pNeigh;

   printf("  Node: %s  at %ld\n",pNode->id==NULL?"no_id":pNode->id,(long)pNode);

    if(edges)
    {
      printf("    -- edges : \n");
      ForAdjEdgesGR_M(pNode,pLink,pEdge)
       printf("    %s  at %ld\n",pEdge->id==NULL?"no_id":pEdge->id,(long)pEdge);
    }

    if(nodes)
    {
      printf("    -- neighbouring nodes : \n");
      ForAdjNodesGR_M(pNode,pLink,pNeigh)
	printf("    %s  at %ld\n",
	  pNeigh->id==NULL?"no_id":pNeigh->id,(long)pNeigh);
    }

  }
  printf("----------\n");
}

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "strGM.h"
#include "gfPars.h"

/*-------------------------------------------------------------------------*/
void DestEntity(t_ParsLLSet set, void ** entity)
{
  int i;
  ForeachEnvGF_M(set,i)
    free(entity[i]);
  
  free(entity);
}

/*-------------------------------------------------------------------------*/
char ** TokenLine;
static void * Analyse(t_FormatItem FormatItem)
{
  if (NULL == *TokenLine)
     MyPtrErr("ERR in Str2Entity: Not enough tokens in the line");

  switch (FormatItem->Type)
  {
    case ID:
    {
      t_ID Id= (t_ID) malloc(sizeof(t_ID*));
      Id->num= (int*) malloc(sizeof(int));
      if (!stricmp(FormatItem->Def,VAR_TOKEN)){
	 char * number=strpbrk (*TokenLine,"0123456789");
	 *(Id->num)=atoi(number);
	 *number='\0';
	 Id->id =DupStr(*TokenLine++);
       }
      else{
	 *(Id->num)=atoi(*TokenLine++);
	 Id->id =FormatItem->Def;
      }
	 
      return  Id;
     }

    case TEXT:
       return  DupStr(*TokenLine++);

    case ATT:
    {
       float * att=malloc (sizeof(float));
       *att       =  atof(*TokenLine++);
       return att;
    }

    case REP:
    {
	VoidVector_t rep= ConstVoidVector(atoi(*TokenLine++));

	int i;
	for(i=0;i<rep->items;i++)
	  rep->data[i]=
	    Analyse((t_FormatItem)(FormatItem->Def));

	return rep;
    }

    case GEOMSTRUCT:
    {
      char * GeomStructItems=FindNthTokenStr(FormatItem->Def,2);
      int Correction = 0;
      int GeomRep    = 1;

      if (IsTokenPrefStr(REP_TOKEN,GeomStructItems)){
	Correction=1;
	GeomRep   =atoi(*TokenLine++);
      }
      {
	int NofFloats=GeomRep*(TokenNumStr(GeomStructItems)-Correction);
	FloatVector_t Vector=ConstFloatVector(NofFloats);
	int i ;

	for(i=0;(i<NofFloats)&&(NULL!=*TokenLine);i++)
	  Vector->data[i]=atof(*TokenLine++);

	if(i!=NofFloats)
	   MyPtrErr("ERR in Str2Entity-not enough tokens for GeomStr");
	 
	return(Vector);
      }
    }

    default:
      MyPtrErr("Err in AddEntitySet");
  } 
  return 0;
} 

/*-------------------------------------------------------------------------*/
void** Str2Entity(t_ParsLLSet set, char * line)
{
  int item;

  void ** Entity = malloc(set->entitySize* sizeof(void*));
  assert(Entity!=NULL);

  TokenLine = TokenizeStr(line);

  ForeachEnvGF_M(set,item)
   if (NULL == (Entity[item] = Analyse(set->format[item])))
    MyPtrErr("problem in Str2Entity") ;

  DestTokenizedStr(TokenLine);
  return Entity;
}

/*-------------------------------------------------------------------------*/
#define B_SIZE 100000
static char buffer[B_SIZE];

static void  AppendItem(void * Item, t_FormatItem formatItem)
{
  switch (formatItem->Type){

  case ID:
    if(!stricmp(VAR_TOKEN,formatItem->Def))
       AppBufStr("%s",((t_ID)(Item))->id);
    else
       AppBufStr("%d ",*((t_ID)(Item))->num);
  break;

  case TEXT: AppBufStr("%s ",Item);
  break;

  case ATT:  AppBufStr("%-.5g ",*((float*)Item));
  break;

  case REP:
  {
    int i;
    VoidVector_t vv = Item;
    AppBufStr("%d ",vv->items);
    for(i=0;i<vv->items;i++)
      AppendItem(vv->data[i],(t_FormatItem) formatItem->Def);
  }
  break;

  case GEOMSTRUCT:
  {
    int i;
    int  NoofGeomStruct= ((FloatVector_t)Item)->items/
	  (TokenNumStr(formatItem->Def)-2) ;
    if (IsTokenPrefStr(REP_TOKEN,FindNthTokenStr(formatItem->Def,2)))
      AppBufStr("%d ",NoofGeomStruct);

    for(i=0; i<((FloatVector_t)Item)->items; i++)
      AppBufStr("%-.5g ",((FloatVector_t)Item)->data[i]);
  }
  break;

  default: MyPtrErr("ERR in Entity2Str - unknown item type");
  break;
  } /* end of switch */
}
	
    
char * Entity2Str(t_ParsLLSet set, void** Entity)
{
  int  item;

  ResBufStr(buffer,B_SIZE);

  ForeachEnvGF_M(set,item)
    AppendItem(Entity[item],set->format[item]);

  return DupStr(buffer);
}

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "strGM.h"
#include "gfPars.h"

/*-------------------------------------------------------------------------*/
static char * IsInternalGeoStr(t_LLSet set, char * work)
{
  char * GeomDef;

  ForeachLL_M(set->GeomStruct,GeomDef)
    if (IsTokenPrefStr(work, GeomDef)) return GeomDef;

  return NULL;
}
/*-------------------------------------------------------------------------*/
static char * IsStdGeoStr( char * work)
{
  char ** GeomDef = StdGeomDef;
  while (NULL != *GeomDef){
    if (IsTokenPrefStr(work, *GeomDef)) return  *GeomDef;
  GeomDef++;
  }
  return NULL;
}
/*-------------------------------------------------------------------------*/
static void DestFormatItem(t_FormatItem item)
{
  switch (item->Type)
  {
    case ID:  case GEOMSTRUCT: case ATT:
      free(item->Def);
    break;

    case REP:  
      DestFormatItem(item->Def);
    break;

    case TEXT: /* nothing to free */
    break;
  }
  free(item);
}

/*-------------------------------------------------------------------------*/
void DestFormat(t_ParsLLSet set)
{
  int i;
  ForeachEnvGF_M(set,i)
    DestFormatItem(set->format[i]);
 
  free(set->format);
}  

/*--------------------------------------------------------------------------*/
void InitSetFormat(t_LLSet set, t_ParsLLSet parset )
{
  int i;
  int repNum = 0;
  char * work, * GeomFormat;

  char ** relatt = TokenizeStr(set->format);
  t_LL format    = ConsLL();

  for(i=0; NULL != (work=relatt[i]); i++)
  {
    t_FormatItem FormatItem = malloc (sizeof (t_FormatItem));
    assert(NULL != FormatItem);

    if (IsTokenPrefStr(ID_TOKEN, work))
    {
      FormatItem->Type = ID;
      FormatItem->Def  = DupStr(work + strlen(ID_TOKEN));
    }
    else if (IsTokenPrefStr(REP_TOKEN,work))
    {
	   repNum++;
	   FormatItem->Type = REP;
	   FormatItem->Def  = NULL;
    }
    else if (IsTokenPrefStr(TEXT_TOKEN, work))
    {
	   FormatItem->Type = TEXT;
	   FormatItem->Def  = NULL;
    }
    else if ((NULL !=(GeomFormat=IsInternalGeoStr(set,work)))||
	     (NULL !=(GeomFormat=IsStdGeoStr(work)))
	    )
    {
	   FormatItem->Type = GEOMSTRUCT;
	   FormatItem->Def  = DupStr(GeomFormat);
    }
    else/* if (IsTokenPrefStr(ATT_TOKEN,work))*/{
	   FormatItem->Type = ATT;
	   FormatItem->Def  = DupStr(work);
    }
    /* everything but ID, TEXT, GEOM, is uderstood to be ATT */
    /*
    else MyPtrErr("ERR in ConstSetFormat: unknown token type");
    */
    InsLastLL(format, FormatItem);
  }

  /*-------- move list of format items into an array --------*/
  {
    t_FormatItem *pItem;
    parset->entitySize = i-repNum;
    parset->format     = malloc(parset->entitySize * sizeof(t_FormatItem));

    assert(parset->format != NULL);

    i=0;
    ForeachLL_M(format, pItem)
    {
       parset->format[i] = *pItem; 
       if((*pItem)->Type == REP)
       {
          pItem = NextElmLL(pItem);
          parset->format[i]->Def = *pItem;
       }
       i++;
    }
  }

  DestTokenizedStr(relatt);
  DestLL(format);
}


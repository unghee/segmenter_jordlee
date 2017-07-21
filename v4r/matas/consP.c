#include "gfPars.h"
#include "strGM.h"

t_ParsLLSet ConsParsLLSet(t_LLSet set)
{
  t_ParsLLSet parsSet =(t_ParsLLSet)malloc (sizeof(*parsSet));
  char * str;

  parsSet->id         = DupStr(set->id);
  parsSet->GeomStruct = ConsCopyLL(set->GeomStruct);
  parsSet->Attributes = ConsCopyLL(set->Attributes);
  parsSet->data       = ConsLL();
  InitSetFormat(set,parsSet);

  ForeachLL_M(set->data,str)
  {
    void* ent = Str2Entity(parsSet,str);
    InsLastLL(parsSet->data,ent);
  }

  return parsSet;
}

void DestParsLLSet(t_ParsLLSet set)
{
  void ** pEntity;

  free(set->id);
  DestLL(set->GeomStruct);
  DestLL(set->Attributes);
  DestFormat(set);

  ForeachLL_M(set->data,pEntity)
   DestEntity(set,*pEntity);

  DestLL(set->data);

  free(set);
}



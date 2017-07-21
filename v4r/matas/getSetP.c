#include <stdlib.h>
#include "gfPars.h"

t_ParsLLSet GetParsSet(t_LL Sets, char *id)
{
  return (t_ParsLLSet) GetLLSet(Sets,id);
}

t_ParsLLSet GetPrefParsSet(t_LL Sets, char *id)
{
  return (t_ParsLLSet) GetPrefLLSet(Sets,id);
}

int GetParsSetNum(t_LL Sets, char *id)
{
  return GetLLSetNum(Sets,id);
}

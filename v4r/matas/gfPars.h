#ifndef  GF_PARS_H
#define GF_PARS_H 

#include <stdio.h>
#include <stdlib.h>

#include "gfLL.h"

enum s_Item {ID,GEOMSTRUCT,ATT,REP,TEXT}; /* types of Entry items */
typedef enum s_Item t_Item ; /* types of Entry items */

typedef struct{
	  t_Item  Type;
	  void *  Def;
	  }* t_FormatItem;

typedef struct{
	 char * id;
	 int  * num;
	 }*t_ID;

typedef struct{
          char * id;

	  unsigned int entitySize;	
	  t_FormatItem *format;
	   
	  t_LL GeomStruct;
	  t_LL Attributes;
	  t_LL data;
         }*t_ParsLLSet;


/* FloatVector is used for storing Geometric Structures */
typedef struct{
         int items;
         float data[2];
}* FloatVector_t;

typedef struct{
  int items;
  void* data[2];
}* VoidVector_t;


extern char *StdGeomDef[]; /* stdandard geometric feature definition */
extern int StdGeomNum;     /* numbur of the stand. geom. features */


/*--------  public ------------------------------------------------- */

t_ParsLLSet     ConsParsLLSet(t_LLSet);     
void            DestParsLLSet(t_ParsLLSet);

void** Str2Entity(t_ParsLLSet, char * line);       /* convert from strings */
char*  Entity2Str(t_ParsLLSet Set, void** Entity); /* convert into strings */

t_ParsLLSet GetParsSet(t_LL Sets, char *id);
t_ParsLLSet GetPrefParsSet(t_LL Sets, char *id);
int         GetParsSetNum(t_LL Sets, char *id) ;

#define ForeachEnvGF_M(set,it) for((it)=0; (it)<(set)->entitySize; (it)++)

/*--------- private  ------------------------------------*/
void InitSetFormat(t_LLSet set, t_ParsLLSet parset );
void DestFormat(t_ParsLLSet set);

void DestEntity(t_ParsLLSet set, void ** entity);


FloatVector_t   ConstFloatVector(int number);
VoidVector_t   ConstVoidVector(int number);

#endif

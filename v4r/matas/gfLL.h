#ifndef  GFLL
#define  GFLL 1
#include <stdio.h>
#include <stdlib.h>

#include "LL.h"

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {               /* can be used directly from C++ */
#endif

struct s_LLSet{
          char * id;
          char * format;
          t_LL GeomStruct;
	  t_LL Attributes;
          t_LL data;
         };
typedef struct s_LLSet *t_LLSet;


/*---- Read/Write sets from a .gf file   ----------------------------------*/
t_LL  ReadLLSets(char *filename);           /* Read/Write a LL list of sets */
t_LLSet ReadLLSet(char * filename,char * SetName);
t_LLSet ReadPrefLLSet(char * filename,char * SetName);

int   WriteLLSets(char *filename, t_LL AllSets);
int   WriteLLSet(char  *filename, t_LLSet Set);


t_LLSet ConsLLSet(char * SetName, t_LL GeomStruct, char * format);
void DestLLSet(t_LLSet Set);
void  DestLLSets(t_LL Sets);


int  GetLLSetNum(t_LL Sets, char *id) ;     /* Get a partic. set        */
t_LLSet GetPrefLLSet(t_LL Sets, char *id);
t_LLSet GetLLSet(t_LL Sets, char *id);
t_LLSet* GetPrefLLpSet(t_LL Sets, char *id);
t_LLSet* GetLLpSet(t_LL Sets, char *id);


/* error handling routines */
extern int  ErrStatus;
/*
  #define MyErr(A) {ErrStatus=1;fprintf(stderr,"%s \n",(A)); return -1;}
  #define MyPtrErr(A) {ErrStatus=1;fprintf(stderr,"%s \n",(A)); return NULL;}
*/
#define MyErr(A)    {ErrStatus=1;fprintf(stderr,"%s \n",(A)); exit(-1);}
#define MyPtrErr(A) {ErrStatus=1;fprintf(stderr,"%s \n",(A)); exit(-1);}
#define MAX_LINE_LENGHT 300000

/* predefinde strings */
/* strings defining beginning end end of a Set */
#define SET_STA_TOKEN "@Set"
#define SET_END_TOKEN "@"


/* strings used in Set header */
#define GEOM_TOKEN   "GeomStruct"   
#define ATTRIB_TOKEN "Attribute"     
#define FORMAT_TOKEN "Format"

/* strings used in format string */
#define REP_TOKEN    "Rep"
#define ID_TOKEN      "ID"
#define ATT_TOKEN     "Att"
#define TEXT_TOKEN    "Text"
#define VAR_TOKEN     "Var"

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif


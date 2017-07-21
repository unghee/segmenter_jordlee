#ifndef STRGM_h
#define STRGM_h

/*----------  string functions extending std. library -----------------------*/
/* author:  G. Matas                               g.matas@ee.surrey.ac.uk   */
/* version: @(#)%E%  %I% %M%                                   */
/*---------------------------------------------------------------------------*/
/* $Id: strGM.h,v 1.2 1995/09/08 08:56:09 ees1rm Exp $
 * Modifications:
 * $Log: strGM.h,v $
 * Revision 1.2  1995/09/08 08:56:09  ees1rm
 * RCS Id and Log added.
 *
*/

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {               /* can be used directly from C++ */
#endif

/*----------- replacements for non-ANSI SUN-specific functions --------------*/
char* DupStr(char * s);        /* duplicate string; (like strdup on SUNs)   */
int   stricmp(const char *s1, const char *s2);
			    /*  compare string s1 to s2 without regard to case
			     * based on code by Henry Spencer
			     * modified for ANSI by D'Arcy J.M. Cain
			     * (like strcasecmp on SUN) */
int   strnicmp(const char *s1, const char *s2,int num);
			   /*  as stricmp, compare at most num chars */
			   /*  (like strncasecmp) */

/*-------------- View the string as a sequence of tokens  ------------------*/
/* a token is a sequence of characters between delimiter chars              */

int   TokenNumStr(char * str);  /*   count the number of tokens in a string*/
char* FindTokenStr(char * str, char *tok);
				/* return ptr to the first occurence fo tok*/
char* FindNthTokenStr(char * str,int tokenNo);
                                /* return pointer to token number 'tokenNo'*/
char* GetNthTokenStr(char * str, int number);
                                           /* a copy of the number-th token*/
char** TokenizeStr(char * str);          
			   /* break a string into and array of ptrs        */
			   /* the array points to private copies of substr */
			   /* last element of the array contains NULL      */
int  SizeTokenizedStr(char ** tkn) ;
void DestTokenizedStr(char ** tkn);
			   /* free memory allocated by TokenizeStr */
int IsTokenPrefStr(char *token, char * str);   
                          
void  SetDelimitersStr(char * del);        /* define a new set of delimiter */
char* GetDelimitersStr(void);              /* read the current delimiters   */

int TokensInStr(char * str,char * delimiters); /* obsolete, use TokenNumStr */ 

/*------------------------- Miscellaneous ----------------------------------*/
char* ConsStr(char * format, ...);
   /* similar to  'sprintf(buffer,format,...)', the string is  created */
   /* in a dynamic (malloc) memory (instead of the static buffer of sprintf  */

void   ResBufStr(char * buffer, int size); /* use 'buffer' for AppBuffStr    */
int    AppBufStr(char * format, ...);        
			       /* append int buffer defined by ResBufStr     */
			       /* overflow checked using size form ResBufStr */
int    SizeBufStr(void);       /* return size of the current string buffer */
char * SuffStr(char * s);      /* return suffix (substr after leftmost period*/
int    IsSuffStr(char * s, char * suff);/* is 'suff' suffix of 's' ? */

/*------------------------- Miscellaneous -----------------------------------*/
char* Argv2Str(int argc, char **argv);
                    /*concatenate array of string ptrs intto a single string */

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif

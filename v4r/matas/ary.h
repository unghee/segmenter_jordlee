#ifndef INC_ARY
#define INC_ARY
/*----------------- dynamic 2-dimensional array lib -------------------*/
/*	

   22-Feb-95, G. Matas
     further addtions document under sccs 

   1-Sep-93, G.Matas
     consAry, consAry1 added     
     error handling functions added

   17-Jun-93, G. Matas
     pointer ARY added from G. Jones' ary lib
     redundant ary_sto field repalced with a macro 
     I/O functions added
     Initialization routines added
     ANSI-fied
     Miscellaneous functions added

   1992
     Written initially by Ron Cain at SRI, this version by Radu Horaud.
*/
/*---------------------------------------------------------------------*/
#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {               /* can be used directly from C++ */
#endif

#include <stdio.h>

enum e_aryEl { 
  BYTEMATRIX=1, SHORTINTMATRIX, INTEGERMATRIX, 
  FLOATMATRIX, DOUBLEMATRIX, UBYTEMATRIX,  POINTERMATRIX,
  RGBMATRIX, XRGBMATRIX, FRGBMATRIX, YUVMATRIX, RGIMATRIX,
  USHORTINTMATRIX, RGBXMATRIX,USER_DEFINED=255
};

typedef enum e_aryEl t_aryEl;
#define un_char   unsigned char 

#define DEF_ARY(NAME, ELEM_TYPE)       \
typedef struct s_ ## NAME{             \
        ELEM_TYPE  **el;               \
	int	   lb1, ub1, lb2, ub2; \
	int        el_size;            \
	t_aryEl    ary_type;           \
} NAME;

#define DEF_ARY_Initialise(SUFFIX, ARY_TYPE, ELEM_TYPE) \
void Initialise ## SUFFIX( ARY_TYPE * a, ELEM_TYPE value)\
{\
  int i,j;\
  for( i=a->lb1; i<= a->ub1; i++ ) for( j=a->lb2; j<=a->ub2; j++ )\
    a->el[i][j]=value;\
}

  

typedef union {un_char arr [3]; struct {un_char r,g,b;}  st; }   t_rgb  ;  
typedef union {float  arr [3]; struct {float r,g,b;}   st; }   t_frgb ;  
typedef union {un_char arr [4]; struct {un_char x,b,g,r;}st; }   t_xrgb ;  
typedef union {un_char arr [4]; struct {un_char r,g,b,x;}st; }   t_rgbx ;  
typedef union {float  arr [3]; struct {float r,g,I;}   st; }   t_rgI ;  
typedef union {float  arr [2]; struct {float r,g;}     st; }   t_rg ;  

typedef struct { unsigned char y; char u; char v;} t_yuv;  

/*--------------- definitions of ary variants for various types -----------*/
DEF_ARY( ARY, void)
DEF_ARY(CARY, char)
DEF_ARY(BARY, unsigned char)
DEF_ARY(SARY, short int)
DEF_ARY(USARY, unsigned short int)
DEF_ARY(IARY, int)
DEF_ARY(FARY, float)
DEF_ARY(RGBARY, t_rgb)
DEF_ARY(XRGBARY, t_xrgb)
DEF_ARY(RGBXARY, t_rgbx)
DEF_ARY(FRGBARY, t_frgb)
DEF_ARY(YUVARY, t_yuv)
DEF_ARY(RGIARY, t_rgI)
DEF_ARY(DARY, double)
DEF_ARY(PARY, void *)

/*----------------- useful macro -------------------------------------------*/
#define startAddrAry(a) ((void*)\
			 (((char*)((a)->el[(a)->lb1])) + (a)->el_size*(a)->lb2))
#define rowsAry(a)      ((a)->ub1 - (a)->lb1 + 1)
#define colsAry(a)      ((a)->ub2 - (a)->lb2 + 1)
int     sizeAry(ARY* ary);
char*   nameAry(ARY* ary);
char*   infoAry(ARY* ary); 
int     isEqBoundsAry(ARY* a1, ARY * a2);
int     isInAry(ARY* ary,int x, int y);


/*----------------- constructors and destructors ---------------------------*/
#define consAry(rows,cols,type)\
	        consAry1(0,(rows)-1,0,(cols)-1,sizeof(type),NULL) 

void *consAry1(int firstrow,int lastrow, int firstcol, int lastcol,
	       int elSize, void * mem);

void destAry(void * ary);              /* free memory used by ary */

ARY *makary(int fr, int lr, int fc, int lc, t_aryEl matrix_type); 
ARY *makmemary(int firstrow,int lastrow, int firstcol, int lastcol, 
	       t_aryEl arytype, void *addr);

/* variants of makary with lower bounds == 0 and proper matrix_typu subst. */
CARY *makCary(int rows, int cols);  
BARY *makBary(int rows, int cols); 
SARY *makSary(int rows, int cols);
IARY *makIary(int rows, int cols);
FARY *makFary(int rows, int cols);
DARY *makDary(int rows, int cols);
PARY *makPary(int rows, int cols);
USARY *makUSary(int rows, int cols);

RGBARY *makRGBary(int rows, int cols);
XRGBARY *makXRGBary (int rows, int cols);
RGBXARY *makRGBXary (int rows, int cols);
YUVARY *makYUVary (int rows, int cols);
FRGBARY *makFRGBary (int rows, int cols);
RGIARY *makRGIary (int rows, int cols) ;

/*----------------- initialisation routines  -------------------------------*/
void InitialiseBary(  BARY *array, unsigned char value  );
void InitialiseIary(  IARY *  array, int value );
void InitialiseUSary(  USARY *array, unsigned short int ptr );

void InitialiseFary(  FARY *array, float  value );
void InitialiseDary(  DARY *array, double value );

void InitialisePary(  PARY *array, void * ptr );

/*--------------------------- I/O   ----------------------------------------*/
ARY * Px2Ary(char * filename);         /* read in ANY Ary */
BARY* P5pgm2Bary(char * filename);    /* read in a 'raw' grey image */ 
BARY* P4pbm2Bary(char * filename);    /* read in a 'raw' bit map */

FARY* Pgm2Fary(char * filename);  
FARY* P5pgm2Fary(char * filename);    /* obsolete, use Pgm2Fary */
RGBARY* P6ppm2RGBary(char * filename);
RGBARY* MoveP6ppm2RGBary(RGBARY * RGBary, char * filename);
YUVARY* P6ppm2YUVary(char * filename);

FILE* ReadP5header(char * filename, int * cols, int * rows);


BARY * BMP2Bary(char * filename);
IARY * SB7toIary(char *filename);

int   Ary2Px(ARY* ary,char * filename);
void* Bary2P5pgm(BARY * image, char * filename);
void* Bary2P4pbm(BARY * image, char * filename);
void* RGBary2P6ppm(RGBARY * image,char * filename);
int   XRGBary2P6ppm(XRGBARY * image,char * filename);
int   RGBXary2P6ppm(RGBXARY * image,char * filename);
void* YUVary2P6ppm(YUVARY * image,char * filename);

void* Bary2P5pgmCm(BARY * image,char * filename,char * comment) ;
void* Bary2P4pbmCm(BARY * image, char * filename,char * comment);
void* RGBary2P6ppmCm(RGBARY * image,char * filename,char * comment);

IARY* P2pgm2Iary(char * filename);
void* Iary2P2pgm(IARY * ary, char * filename);
void* Iary2BMP(IARY * ary, char * filename);
IARY* MoveP2pgm2Iary(IARY * ary, char * filename);


void CutFary2P5pgm(FARY * image,char * filename);
void ScaleFary2P5pgm(FARY * image,char * filename) ;

void ScaleMinMaxFary(FARY * image, float min, float max);

BARY *  ScaleFary2Bary(FARY *fary);
BARY *  ScaleMinMaxFary2Bary(FARY *fary);


FARY* P5pgmRoi2Fary(char * filename, int * roi, int fullImage, int extend);
FARY* P5pgmRoi2Fary1 (FILE * file, int cols, int rows, int * roi,
			      int fullImage, int extend, void * mem);

const void* WriteAry(const void *ary,char * filename);
void* ReadAry(char *filename);
void  ReadInAry(ARY *,char *filename);

void printFary(FARY * ary);    /* prints a matrix of floats to stdout */
void printIary(IARY * ary);    /* prints a matrix of int to stdout */

/*-------------------- miscellaneous ----------------------------------------*/
ARY * CopyAry(ARY * ary);
void  ScaleFillBary(BARY *dest, BARY * src, float f_zoom1, float f_zoom2) ;
void InitBorderFary(FARY * image, int border, int pixel);
FARY * ExtendFary(FARY * in_image, int border);

float MinFary(FARY *ary);
float MinIary(IARY *ary);
float MaxFary(FARY *ary);

void ScaleMaxFary(FARY *ary,int requested_max) ;
/*FARY *    ScaleFary2Fary(FARY * image); */     /* by Rupert Young */

int SubtractIary(IARY *ary, IARY *subtract);
int AbsSubtractIary(IARY *ary,IARY *sub);
int AddToIary(IARY *ary, int value);
/*------------------- trasformations ---------------------------------------*/
void SetYoff(int off);
t_frgb  Yuv2Frgb(t_yuv p) ;

t_rgb   Yuv2Rgb(t_yuv p);
t_rgI   Yuv2Rgi(t_yuv yuv);

t_rgI   Frgb2Rgi(t_frgb frgb);
t_rgI   Rgb2Rgi(t_rgb rgb);
t_rgI   Yuv2Rgi(t_yuv yuv);

t_rgb  Frgb2Rgb(t_frgb frgb);
t_frgb Rgb2Frgb(t_rgb frgb);
t_rg   Frgb2Rg(t_frgb frgb);

RGBARY*  YUVary2RGBary(YUVARY * ary) ;
FRGBARY* YUVary2FRGBary(YUVARY * ary);
RGIARY*  YUVary2RGIary(YUVARY * ary) ;

BARY *    Fary2Bary(FARY *fary);
BARY *    Iary2Bary(IARY *in);
IARY *    Bary2Iary(BARY *in);

RGBARY *  XRGBary2RGBary(XRGBARY * image) ;
RGBARY *  XBGRary2RGBary(XRGBARY *xrgb) ;
RGBARY *  RGBXary2RGBary(RGBXARY *rgbx);
void RGBXaryToRGBary(RGBXARY *rgbx,RGBARY *rgb);


/* by Rupert Young */
void RGBary2Fary3(RGBARY * col_image, FARY * out[3]);
void RGBary2FaryArr3(RGBARY * col_image, FARY * red, FARY * green, FARY * blue);

RGBARY * Fary3_2RGBary(FARY * out[3]);
RGBARY * FaryArr3_2RGBary(FARY * red, FARY * green, FARY * blue);

FARY * RGBary2Y(RGBARY * image);


SARY *  Bary2Sary(BARY *bary);
BARY *  Sary2Bary(SARY *sary);

BARY *  USary2Bary(USARY *usary);
void    USaryToBary(USARY *usary, BARY * bary);

void  LastPnmHeader (int * cols, int * rows, int * levels);

/*-------------------- internal ---------------------------------------------*/
extern int exitOnError;
void ExitOnAryError(void );
void ReturnOnAryError(void );

#define ErrHandle(message,code)\
 { if(exitOnError){fprintf(stderr,"%s \n",message);exit(code);}\
   else  return 0; }

#define ErrHandle1(message,context,code)\
 { if(exitOnError){fprintf(stderr,"%s %s \n",message,context);exit(code);}\
   else return 0;}

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif

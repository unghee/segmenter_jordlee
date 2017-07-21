#include "ary.h"

typedef short int int16;
typedef int       int32;

typedef struct  Win3xBMPHeader{
	char   imageFileType[2]; 
  int32  fileSize;
  int16  reserved1;
  int16  reserved2;
  int32  imageDataOffset;
} Win3xBMPHeader;

typedef struct Win3xBMPInfoHeader{
  int16   headerSize;
  int16   x ;
  int32   imageWidth;
  int32   imageHeight;
  int16   numberOfImagePlanes; /* always 1*/
  int16   bitsPerPixel;      /* 1, 4, 8 or 24*/
  int32   compressionMethod; /* 0,1, or 2 */
  int32   sizeOfBitmap;
  int32   horzResolution;
  int32   vertResolution;
  int32   numColorsUsed;
  int32   numSignificantColors; 
} Win3xBMPInfoHeader;

typedef struct Win3xBMP{
  struct Win3xBMPHeader     header;
  struct Win3xBMPInfoHeader info;
  t_rgbx *palette; 
}Win3xBMP;

Win3xBMP ConsBMPHeader(int rows, int cols);
Win3xBMP ReadBMPHeader(FILE * f);
void     PrintBMPHeader(Win3xBMP h);

/*--------------------------------------------------------------------------*/
/* Author:  George (Jiri) Matas                     g.matas@ee.surrey.ac.uk */
/*--------------------------------------------------------------------------*/
/* $Id: colorquant.h,v 1.5 1996/09/24 08:21:00 ees2gm Exp $ */
/* Modifications:
 * $Log: colorquant.h,v $
 * Revision 1.5  1996/09/24  08:21:00  ees2gm
 * *** empty log message ***
 *
 * Revision 1.4  1996/07/19  13:52:47  ees2gm
 * *** empty log message ***
 *
 * Revision 1.3  1996/02/21  15:55:51  ees2gm
 * *** empty log message ***
 *
 * Revision 1.2  1996/02/13  10:17:50  ees2gm
 * t_grayQuant,t_rgbQuant defined plus a number of conversion functions
 *
 * Revision 1.1  1995/09/07  10:07:37  ees1rm
 * Initial revision
 *
*/

#ifndef COLORQUANT_H
#define COLORQUANT_H
#include "ary.h"

/*--------------------------------------------------------------------------*/
typedef unsigned char t_colormap_ind;  /* X11 definition */
typedef unsigned char t_color;

typedef struct s_rgbQuant{
   int             bits_r;
   int             bits_g;
   int             bits_b;
   t_colormap_ind  *rgbMap;
   int             rgbSize;

   t_color        *colorMap[3];
   int            colors   ;
} *t_rgbQuant;

typedef struct   s_grayQuant{
   t_colormap_ind   grayMap[256];

   t_color  *colorMap[3];
   int      colors;
} *t_grayQuant;

/*--------------------------------------------------------------------------*/
typedef struct s_rgbQuantPars { 
  int colors; int bits; int fast;
} t_rgbQuantPars;

t_rgbQuant      rgbQuant(RGBARY * ary, int colors, int bits, int fast);
t_rgbQuant      rgbxQuant(RGBXARY * ary, int colors, int bits, int fast);
t_rgbQuant      rgbQuantP(RGBARY * ary, t_rgbQuantPars pars);
t_rgbQuant      rgbxQuantP(RGBXARY * ary, t_rgbQuantPars pars);
t_rgbQuantPars consRgbQuantPars( int colors, int bits, int fast);
  

typedef struct s_rgbQuantFixedPars{ 
  int bits_r; int bits_g; int bits_b;
} t_rgbQuantFixedPars;

t_rgbQuant rgbQuantFixed(RGBARY * ary, int bits_r, int bits_g, int bits_b);
t_rgbQuant rgbxQuantFixed(RGBXARY * ary, int bits_r, int bits_g, int bits_b);
t_rgbQuant rgbQuantFixedP(RGBARY * ary, t_rgbQuantFixedPars pars);
t_rgbQuant rgbxQuantFixedP(RGBXARY * ary, t_rgbQuantFixedPars pars);
t_rgbQuantFixedPars consRgbQuantFixedPars(int bits_r,int bits_g, int bits_b);


void       DestRgbQuant(t_rgbQuant);

void RGBary2Bary(RGBARY *ary, BARY * ind, t_rgbQuant quant);
void RGBXary2Bary(RGBXARY *ary, BARY * ind, t_rgbQuant quant);
void Bary2RGBary(BARY * ind, RGBARY* ary, t_rgbQuant quant);

int
colorquant(t_color * red, t_color * green, t_color *blue,
           unsigned long pixels, t_color * colormap[3],
           int colors, int bits, t_color *rgbmap, int fast);

/*------------------------------------------------------------------------*/
t_grayQuant grayQuant(int levels);
t_grayQuant grayQuantRain(int levels);
t_grayQuant grayQuantFile(int levels,char * file, int mapType);
void        DestGrayQuant(t_grayQuant q);

void Bary2QuantBary(BARY *ary, BARY * ind, t_grayQuant quant);
void Fary2QuantBary(FARY *fary, BARY * qbary,float gama, int scale,
                     t_grayQuant quant);


void checkRgbQuant(t_rgbQuant q);
#endif

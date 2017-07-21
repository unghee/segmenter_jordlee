/*--------------------------------------------------------------------------*/
/* Author:  George (Jiri) Matas                     g.matas@ee.surrey.ac.uk */
/*--------------------------------------------------------------------------*/
static const
char rcsid[] = "$Id: quantAry.c,v 1.8 1996/09/24 08:21:00 ees2gm Exp $";
typedef char _r_foo[sizeof(rcsid)];

#include <stdlib.h>
#include "ary.h"
#include "colorquant.h"

void checkRgbQuant(t_rgbQuant q)
{
  int   colorNum[256];
  int   i;

  printf("colors: %d \n",q->colors);
  for(i=0;i<256;i++) colorNum[i]=0;

  for(i=0;i<q->rgbSize;i++) colorNum[q->rgbMap[i]]++;    

  for(i=0;i<256;i++) printf("%3d %d\n",i,colorNum[i]);
}


static t_rgbQuant consRgbQuant(void)
{
  t_rgbQuant x = malloc(sizeof(*x));
  if(x==NULL)
    { fprintf(stderr,"malloc returned 0 in consRgbQuant\n");exit(-1);}

  return x;
}

t_rgbQuantPars consRgbQuantPars(int colors, int bits, int fast)
{
  t_rgbQuantPars pars;
  pars.colors = colors;
  pars.bits = bits;
  pars.fast = fast;
  return  pars;
}

t_rgbQuant rgbQuantP(RGBARY * ary, t_rgbQuantPars pars)
{
 return   rgbQuant(ary, pars.colors, pars.bits, pars.fast);
}

t_rgbQuant rgbxQuantP(RGBXARY * ary, t_rgbQuantPars pars)
{
 return   rgbxQuant(ary, pars.colors, pars.bits, pars.fast);
}


t_rgbQuant rgbQuant(RGBARY * ary, int colors, int bits, int fast)
{
   t_rgbQuant quant = consRgbQuant();

   unsigned long npix = rowsAry(ary)*colsAry(ary);

   t_color* red  ; 
   t_color* green;
   t_color* blue ;

   int i,j;
   int shift = 8 - bits;


   if (    NULL==(red   = malloc(npix)) 
        || NULL==(green = malloc(npix))
        || NULL==(blue  = malloc(npix)))
    { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}

   npix = 0;
   for(i=ary->lb1;i<=ary->ub1;i++) 
     for(j=ary->lb2;j<=ary->ub2;j++) 
     {
       red[npix]  = (ary->el[i][j].st.r)>>shift;
       green[npix]= (ary->el[i][j].st.g)>>shift;
       blue[npix] = (ary->el[i][j].st.b)>>shift;
       npix++;
     }

   quant->bits_r = bits;
   quant->bits_g = bits;
   quant->bits_b = bits;

   quant->rgbSize = 1<<(bits*3);
   quant->rgbMap  = malloc(sizeof(t_colormap_ind)*quant->rgbSize);

   if (NULL==quant->rgbMap)
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}

   for(i=0;i<3;i++)
   {
     quant->colorMap[i] = malloc(colors);
     if (NULL==quant->colorMap[i])
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}
   }


   quant->colors = colorquant(red,green,blue,npix,
                              quant->colorMap,colors,bits,quant->rgbMap,fast);

   free(red);
   free(blue);
   free(green);

   return quant; 
}

t_rgbQuant rgbxQuant(RGBXARY * ary, int colors, int bits, int fast)
{
   t_rgbQuant quant = consRgbQuant();

   unsigned long npix = rowsAry(ary)*colsAry(ary);

   t_color* red  ; 
   t_color* green;
   t_color* blue ;

   int i,j;
   int shift = 8 - bits;


   if (    NULL==(red   = malloc(npix)) 
        || NULL==(green = malloc(npix))
        || NULL==(blue  = malloc(npix)))
    { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}

   npix = 0;
   for(i=ary->lb1;i<=ary->ub1;i++) 
     for(j=ary->lb2;j<=ary->ub2;j++) 
     {
       red[npix]  = (ary->el[i][j].st.r)>>shift;
       green[npix]= (ary->el[i][j].st.g)>>shift;
       blue[npix] = (ary->el[i][j].st.b)>>shift;
       npix++;
     }

   quant->bits_r = bits;
   quant->bits_g = bits;
   quant->bits_b = bits;

   quant->rgbSize = 1<<(bits*3);
   quant->rgbMap  = malloc(sizeof(t_colormap_ind)*quant->rgbSize);

   if (NULL==quant->rgbMap)
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}

   for(i=0;i<3;i++)
   {
     quant->colorMap[i] = malloc(colors);
     if (NULL==quant->colorMap[i])
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}
   }


   quant->colors = colorquant(red,green,blue,npix,
                              quant->colorMap,colors,bits,quant->rgbMap,fast);

   free(red);
   free(blue);
   free(green);

   return quant; 
}

/*--------------------------------------------------------------------------*/
t_rgbQuantFixedPars consRgbQuantFixedPars(int bits_r, int bits_g, int bits_b)
{
  t_rgbQuantFixedPars pars;
  pars.bits_r = bits_r;
  pars.bits_g = bits_g;
  pars.bits_b = bits_b;
  return  pars;
}

t_rgbQuant rgbQuantFixedP(RGBARY * ary,t_rgbQuantFixedPars pars)
{
   return rgbQuantFixed(ary, pars.bits_r, pars.bits_g, pars.bits_b);
}

t_rgbQuant rgbxQuantFixedP(RGBXARY * ary,t_rgbQuantFixedPars pars)
{
   return rgbxQuantFixed(ary, pars.bits_r, pars.bits_g, pars.bits_b);
}


t_rgbQuant rgbQuantFixed(RGBARY * ary, int bits_r, int bits_g, int bits_b)
{
   t_rgbQuant quant = consRgbQuant();
   int i,j,k,index;

   int factor_r = 1<<(8-bits_r);
   int factor_g = 1<<(8-bits_g);
   int factor_b = 1<<(8-bits_b);

   quant->bits_r = bits_r;
   quant->bits_g = bits_g;
   quant->bits_b = bits_b;

   quant->rgbSize = 1<<(bits_r + bits_g + bits_b);
   quant->colors  = quant->rgbSize;
   quant->rgbMap  = malloc(sizeof(t_colormap_ind)*quant->rgbSize);
                  
   if (NULL==quant->rgbMap)
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}

   for(i=0;i<3;i++)
   {
     quant->colorMap[i] = malloc(quant->colors);
     if (NULL==quant->colorMap[i])
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}
   }

   
   index=0;
   for(i=0;i< 1<<bits_r;i++)
   for(j=0;j< 1<<bits_g;j++)
   for(k=0;k< 1<<bits_b;k++)
   {
      quant->rgbMap[(((i<<bits_g)| j)<<bits_b) |k ]=index;
      quant->colorMap[0][index]=i*factor_r;
      quant->colorMap[1][index]=j*factor_g;
      quant->colorMap[2][index]=k*factor_b;
      index++;
   }

   return quant; 
}

t_rgbQuant rgbxQuantFixed(RGBXARY * ary, int bits_r, int bits_g, int bits_b)
{
   t_rgbQuant quant = consRgbQuant();
   int i,j,k,index;

   int factor_r = 1<<(8-bits_r);
   int factor_g = 1<<(8-bits_g);
   int factor_b = 1<<(8-bits_b);

   quant->bits_r = bits_r;
   quant->bits_g = bits_g;
   quant->bits_b = bits_b;

   quant->rgbSize = 1<<(bits_r + bits_g + bits_b);
   quant->colors  = quant->rgbSize;
   quant->rgbMap  = malloc(sizeof(t_colormap_ind)*quant->rgbSize);
                  
   if (NULL==quant->rgbMap)
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}

   for(i=0;i<3;i++)
   {
     quant->colorMap[i] = malloc(quant->colors);
     if (NULL==quant->colorMap[i])
      { fprintf(stderr,"malloc returned 0 in colorQuant\n");exit(-1);}
   }

   
   index=0;
   for(i=0;i< 1<<bits_r;i++)
   for(j=0;j< 1<<bits_g;j++)
   for(k=0;k< 1<<bits_b;k++)
   {
      quant->rgbMap[(((i<<bits_g)| j)<<bits_b) |k ]=index;
      quant->colorMap[0][index]=i*factor_r;
      quant->colorMap[1][index]=j*factor_g;
      quant->colorMap[2][index]=k*factor_b;
      index++;
   }

   return quant; 
}

void DestRgbQuant(t_rgbQuant quant)
{
   int i;
   free(quant->rgbMap);
   for(i=0;i<3;i++) free(quant->colorMap[i]);
   free(quant);
}


void RGBary2Bary(RGBARY *ary, BARY * ind, t_rgbQuant quant)
{
  int i,j;
  int shift_r = 8 - quant->bits_r;
  int shift_g = 8 - quant->bits_g;
  int shift_b = 8 - quant->bits_b;

  for(i=0;i<=ary->ub1;i++) 
  for(j=0;j<=ary->ub2;j++) 
  {
    t_color r   = (ary->el[i][j].st.r)>>shift_r;
    t_color g   = (ary->el[i][j].st.g)>>shift_g;
    t_color b   = (ary->el[i][j].st.b)>>shift_b;
    
    ind->el[i][j]= quant->rgbMap[(((r<<quant->bits_g) | g)<<quant->bits_b)|b]; 
  }
}
void RGBXary2Bary(RGBXARY *ary, BARY * ind, t_rgbQuant quant)
{
  int i,j;
  int shift_r = 8 - quant->bits_r;
  int shift_g = 8 - quant->bits_g;
  int shift_b = 8 - quant->bits_b;

  for(i=0;i<=ary->ub1;i++) 
  for(j=0;j<=ary->ub2;j++) 
  {
    t_color r   = (ary->el[i][j].st.r)>>shift_r;
    t_color g   = (ary->el[i][j].st.g)>>shift_g;
    t_color b   = (ary->el[i][j].st.b)>>shift_b;
    
    ind->el[i][j]= quant->rgbMap[(((r<<quant->bits_g) | g)<<quant->bits_b)|b]; 
  }
}

void Bary2RGBary(BARY * ind, RGBARY* ary, t_rgbQuant quant)
{
  int i,j,k;
  for(i=0;i<=ary->ub1;i++) 
  for(j=0;j<=ary->ub2;j++) 
  for(k=0;k<=2;k++) 
    ary->el[i][j].arr[k] = quant->colorMap[k][ind->el[i][j]];
}

/*------------------------------------------------------------------------*/
static t_grayQuant consGrayQuant(int levels, int mapType)
{
  t_grayQuant quant = malloc(sizeof(*quant));
  int i;
  if(quant==NULL)
    { fprintf(stderr,"malloc returned 0 in consGrayQuant\n");exit(-1);}

  quant->colors = levels;

  for(i=0;i<256;i++)
    if (mapType == 0) quant->grayMap[i]=i % levels    ;
    else              quant->grayMap[i]=levels/256.0*i;

	for(i=0;i<3;i++)
   {
     quant->colorMap[i] = malloc(levels);
     if (NULL==quant->colorMap[i])
      { fprintf(stderr,"malloc returned 0 in grayQuant\n");exit(-1);}
   }
  return quant;
}

t_grayQuant grayQuant(int levels)
{
  t_grayQuant quant = consGrayQuant(levels,1);

  int i;
  for(i=0;i<levels;i++)
	{
   quant->colorMap[0][i] =
   quant->colorMap[1][i] =
   quant->colorMap[2][i] = (i+0.5)*256.0/levels;
  }
  return quant;
}

t_grayQuant grayQuantFile(int levels, char * file, int mapType)
{
  t_grayQuant quant = consGrayQuant(levels,mapType);
  RGBARY       *rgb = P6ppm2RGBary(file);

  int m;

  for(m=0;m<256;m++)
  {
    int l =  quant->grayMap[m];
    int color = (mapType==1)  ? m : l;
    int j =  color % (rgb->ub2+1) ;
    int i =  color / (rgb->ub2+1) ;

    quant->colorMap[0][l] = rgb->el[i][j].st.r;
    quant->colorMap[1][l] = rgb->el[i][j].st.g;
    quant->colorMap[2][l] = rgb->el[i][j].st.b;
  }
  return quant;
}

t_grayQuant grayQuantRain(int levels)
{
  return grayQuantFile(levels,
       "/user/eevsspsr/ees2gm/Soft/color/colorMaps/rainbow1.ppm",1);
}

void DestGrayQuant(t_grayQuant quant)
{
   int i;
   for(i=0;i<3;i++) free(quant->colorMap[i]);
   free(quant); 
}

void Bary2QuantBary(BARY *ary, BARY * qbary, t_grayQuant quant)
{
  int i,j;
  for(i=0;i<=ary->ub1;i++) 
  for(j=0;j<=ary->ub2;j++) 
    qbary->el[i][j] = quant->grayMap[ary->el[i][j]];
    
}

#include <math.h>
void Fary2QuantBary(FARY *fary, BARY * qbary,float gama, int scale,
                     t_grayQuant quant)
{
  int i,j;
  if(gama!=0.0)
  {
  for(i=0;i<=fary->ub1;i++) for(j=0;j<=fary->ub2;j++) 
	  if     (gama>0.0)  fary->el[i][j] = exp((1.0/gama)*log(fary->el[i][j]+1));
	  else               fary->el[i][j] = log(fary->el[i][j]+1);
  }

  if (scale!=0) ScaleMaxFary(fary,255);


  for(i=0;i<=fary->ub1;i++) for(j=0;j<=fary->ub2;j++) 
  {
    int val = (int)  fary->el[i][j];
    if     (val > 255)  val = 255; 
    else if(val < 0  )  val = 0; 
    qbary->el[i][j] = quant->grayMap[val];
  }
}

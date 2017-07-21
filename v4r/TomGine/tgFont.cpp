#include "tgFont.h"
#include "headers.h"
#include <stdexcept>
#include <stdio.h>

using namespace TomGine;

tgFont::tgFont()
{
#ifdef USE_FTGL_FONT
  m_font = new FTPixmapFont(TTF_FONT); // FTPixmapFont(TTF_FONT) -> better, but buggy
  if(m_font->Error())
  {
    char err[64];
    sprintf(err, "[tgFont::tgFont()] Error cannot create font '%s'", TTF_FONT);
    throw std::runtime_error(err);
  }
#endif
}

tgFont::~tgFont()
{
#ifdef USE_FTGL_FONT
  delete(m_font);
#endif
}

void tgFont::Set(tgFont::Type type)
{
#ifdef USE_FTGL_FONT
  delete(m_font);
  switch(type)
  {
    case BITMAP_FONT:
    m_font = new FTBitmapFont(TTF_FONT);
    break;

    case PIXMAP_FONT:
    m_font = new FTPixmapFont(TTF_FONT);
    break;

    default:
    m_font = new FTBitmapFont(TTF_FONT);
    break;
  }
  if(m_font->Error())
  {
    char err[64];
    sprintf(err, "[tgFont::Set()] Error cannot create font '%s'", TTF_FONT);
    throw std::runtime_error(err);
  }
#endif
}

void tgFont::Print(const char* text, int size, int x, int y, float r, float g, float b, float a) const
{
#ifdef USE_FTGL_FONT
  glColor4f(r,g,b,a);
  GLfloat raster_pos[4];
  glGetFloatv(GL_CURRENT_RASTER_POSITION, raster_pos);
  //  printf("[tgFont::Print] %f %f %f %f\n", raster_pos[0], raster_pos[1], raster_pos[2], raster_pos[3]);
  //  glRasterPos4f(raster_pos[0], raster_pos[1], raster_pos[2], raster_pos[3]);  // must be called for color to take effect
  glRasterPos4f(0.0f, 0.0f, 0.0f, 1.0f); // must be called for color to take effect
  m_font->FaceSize(size);
  m_font->Render(text, -1, FTPoint(x,y));
#endif
#ifdef GLX_FONT
  if (!glIsList(m_font_base))
  throw std::runtime_error("[tgFont::Print]: Bad display list. - Exiting.\n");

  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);

  glColor4f(r, g, b, a);
  glRasterPos2i(x, y);

  glPushAttrib(GL_LIST_BIT);
  glListBase(m_font_base);
  glCallLists(strlen(text), GL_UNSIGNED_BYTE, (GLubyte*)text);
  glPopAttrib();
#endif
}

void tgFont::Print(const tgLabel2D &label) const
{
#ifdef USE_FTGL_FONT
  glColor4f(label.rgba.x,label.rgba.y,label.rgba.z,label.rgba.w);
  GLfloat raster_pos[4];
  glGetFloatv(GL_CURRENT_RASTER_POSITION, raster_pos);
  glRasterPos4f(raster_pos[0], raster_pos[1], raster_pos[2], raster_pos[3]); // must be called for color to take effect
  m_font->FaceSize(label.size);
  m_font->Render(label.text.c_str(), -1, FTPoint(label.x,label.y));
#endif
#ifdef GLX_FONT
  Print(label.text.c_str(), 20, label.x, label.y, label.rgba.x, label.rgba.y, label.rgba.z, label.rgba.w);
#endif
}

#include "tgNurbsSurface.h"
#include "tgError.h"
#include <string>

using namespace TomGine;

tgNurbsSurface::tgNurbsSurface(TomGine::tgShader *nurbsSurfaceShader)
{
  m_NurbsSurfaceShader = nurbsSurfaceShader;
}

tgNurbsSurface::tgNurbsSurface(const tgNurbsSurfacePatch &data, TomGine::tgShader *nurbsSurfaceShader)
{
  m_NurbsSurfaceShader = nurbsSurfaceShader;
  Set(data);
}

void tgNurbsSurface::SetShader()
{
  // setup NURBS shader
  m_NurbsSurfaceShader->bind();
  m_NurbsSurfaceShader->setUniform("orderU", (int) nurbsData.orderU);
  m_NurbsSurfaceShader->setUniform("orderV", (int) nurbsData.orderV);
  m_NurbsSurfaceShader->setUniform("nknotsU", (int) nurbsData.knotsU.size());
  m_NurbsSurfaceShader->setUniform("nknotsV", (int) nurbsData.knotsV.size());
  m_NurbsSurfaceShader->setUniform("knotsU", 0);
  m_NurbsSurfaceShader->setUniform("knotsV", 1);
  m_NurbsSurfaceShader->setUniform("cps", 2);
  m_NurbsSurfaceShader->unbind();
#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::SetShader()] B");
#endif
}

tgShader* tgNurbsSurface::LoadShader()
{
  std::string nurbs_vert = std::string(TOMGINE_SHADER) + "nurbssurface.vert";
  std::string cox_head = std::string(TOMGINE_SHADER) + "coxdeboor.c";
  std::string color_frag = std::string(TOMGINE_SHADER) + "color.frag";
  TomGine::tgShader* shader = new tgShader(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());
  return shader;
}

void tgNurbsSurface::Set(const tgNurbsSurfacePatch &data)
{
  nurbsData = data;

  texKnotsU.Load(&nurbsData.knotsU[0], nurbsData.knotsU.size(), GL_R32F, GL_RED, GL_FLOAT);
  texKnotsV.Load(&nurbsData.knotsV[0], nurbsData.knotsV.size(), GL_R32F, GL_RED, GL_FLOAT);
  texCP.Load(&nurbsData.cps[0].x, nurbsData.ncpsU, nurbsData.ncpsV, GL_RGBA32F, GL_RGBA, GL_FLOAT);

  //	m_glunurb = gluNewNurbsRenderer();
  //	gluNurbsProperty( m_glunurb, GLU_SAMPLING_TOLERANCE, 25.0 );
  //	gluNurbsProperty( m_glunurb, GLU_DISPLAY_MODE, GLU_FILL );
  //	gluNurbsCallback(m_glunurb, GLU_ERROR, (GLvoid(*)()) nurbsError);

  Remesh(nurbsData.resU, nurbsData.resV);
#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::Set(const tgNurbsSurfacePatch)]");
#endif
}

void tgNurbsSurface::Remesh(unsigned resU, unsigned resV)
{
  float x0 = nurbsData.knotsU[0];
  float y0 = nurbsData.knotsV[0];
  float w = nurbsData.knotsU[nurbsData.knotsU.size() - 1] - x0;
  float h = nurbsData.knotsV[nurbsData.knotsV.size() - 1] - y0;

  m_model.Clear();
  tgShapeCreator::CreatePlaneXY(m_model, x0, y0, 0.0, w, h, resU, resV);

  float dx = w / resU;
  float dy = h / resV;
  for( unsigned i = 0; i < resU; i++ ) {
    TomGine::tgLine line;
    line.start = vec3(x0 + i * dx, 0.0f, 0.0f);
    line.end = vec3(x0 + (i + 1) * dx, 0.0f, 0.0f);
    m_boundary.push_back(line);
    line.start = vec3(x0 + i * dx, y0 + h, 0.0f);
    line.end = vec3(x0 + (i + 1) * dx, y0 + h, 0.0f);
    m_boundary.push_back(line);
  }
  for( unsigned i = 0; i < resV; i++ ) {
    TomGine::tgLine line;
    line.start = vec3(0.0f, y0 + i * dy, 0.0f);
    line.end = vec3(0.0f, y0 + (i + 1) * dy, 0.0f);
    m_boundary.push_back(line);
    line.start = vec3(x0 + w, y0 + i * dy, 0.0f);
    line.end = vec3(x0 + w, y0 + (i + 1) * dy, 0.0f);
    m_boundary.push_back(line);
  }
}

void tgNurbsSurface::SetCP(unsigned i, unsigned j, const vec4 &cpv)
{
  unsigned idx = j * nurbsData.ncpsU + i;
  SetCP(idx, cpv);
}
void tgNurbsSurface::SetCP(unsigned i, const vec4 &cpv)
{
  if( i >= nurbsData.cps.size() || i < 0 ) {
    printf("[tgNurbsSurface::UpdateCV] Warning: index out of bounds.\n");
    return;
  }
  nurbsData.cps[i] = cpv;
  texCP.Load(&nurbsData.cps[0].x, nurbsData.ncpsU, nurbsData.ncpsV, GL_RGBA32F, GL_RGBA, GL_FLOAT);
}

vec4 tgNurbsSurface::GetCP(unsigned i, unsigned j)
{
  unsigned idx = j * nurbsData.ncpsU + i;
  if( idx >= nurbsData.cps.size() || idx < 0 ) {
    printf("[tgNurbsSurface::GetCV] Warning: index out of bounds.\n");
    return vec4(0.0, 0.0, 0.0, 1.0);
  }
  return nurbsData.cps[idx];
}

vec4 tgNurbsSurface::GetCP(unsigned i)
{
  if( i >= nurbsData.cps.size() || i < 0 ) {
    printf("[tgNurbsSurface::GetCV] Warning: index out of bounds.\n");
    return vec4(0.0, 0.0, 0.0, 1.0);
  }
  return nurbsData.cps[i];
}

void tgNurbsSurface::DrawVertices()
{
  SetShader();
  // draw B-Spline surface
  m_NurbsSurfaceShader->bind();

  texKnotsU.Bind(0);
  texKnotsV.Bind(1);
  texCP.Bind(2);

  m_model.DrawVertices();

  m_NurbsSurfaceShader->unbind();
  glDisable(GL_TEXTURE_1D);
  glDisable(GL_TEXTURE_2D);
#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::DrawVertices()]");
#endif
}

void tgNurbsSurface::DrawFaces()
{
  SetShader();
  // draw B-Spline surface
  m_NurbsSurfaceShader->bind();

  texKnotsU.Bind(0);
  texKnotsV.Bind(1);
  texCP.Bind(2);

  m_model.DrawFaces();

  m_NurbsSurfaceShader->unbind();
  glDisable(GL_TEXTURE_1D);
  glDisable(GL_TEXTURE_2D);

#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::DrawFaces()]");
#endif

  //	glEnable(GL_LIGHTING);
  //	gluBeginSurface(m_glunurb);
  //	gluNurbsSurface(m_glunurb,
  //				nurbsData.knotsU.size(), &nurbsData.knotsU[0],
  //				nurbsData.knotsV.size(), &nurbsData.knotsV[0],
  //				nurbsData.ncpsU * 4, 4,
  //				&nurbsData.cps[0].x, nurbsData.orderU+1, nurbsData.orderV+1, GL_MAP2_VERTEX_4);
  //
  //	gluEndSurface(m_glunurb);
}

void tgNurbsSurface::DrawFaces(float r, float g, float b)
{
  SetShader();
  // draw B-Spline surface
  m_NurbsSurfaceShader->bind();
  m_NurbsSurfaceShader->setUniform("user_color", true);
  m_NurbsSurfaceShader->setUniform("ucolor", vec4(r, g, b, 0.0));

  texKnotsU.Bind(0);
  texKnotsV.Bind(1);
  texCP.Bind(2);

  m_model.DrawFaces();
  m_NurbsSurfaceShader->setUniform("user_color", false);
  m_NurbsSurfaceShader->unbind();
  glDisable(GL_TEXTURE_1D);
  glDisable(GL_TEXTURE_2D);

#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::DrawFaces(float r, float g, float b)]");
#endif
}

void tgNurbsSurface::DrawBoundary(float r, float g, float b, float linewidth)
{
  SetShader();
  m_NurbsSurfaceShader->bind();
  m_NurbsSurfaceShader->setUniform("user_color", true);
  m_NurbsSurfaceShader->setUniform("ucolor", vec4(r, g, b, 0.0));

  texKnotsU.Bind(0);
  texKnotsV.Bind(1);
  texCP.Bind(2);

  glBegin(GL_LINES);
  for( unsigned i = 0; i < m_boundary.size(); i++ ) {
    glVertex3f(m_boundary[i].start.x, m_boundary[i].start.y, m_boundary[i].start.z);
    glVertex3f(m_boundary[i].end.x, m_boundary[i].end.y, m_boundary[i].end.z);
  }
  glEnd();

  m_NurbsSurfaceShader->setUniform("user_color", false);
  m_NurbsSurfaceShader->unbind();
  glDisable(GL_TEXTURE_1D);
  glDisable(GL_TEXTURE_2D);

#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::DrawBoundary(float r, float g, float b, float linewidth)]");
#endif
}

void tgNurbsSurface::DrawCPs()
{
  glDisable(GL_LIGHTING);

  glBegin(GL_POINTS);
  for( unsigned i = 0; i < nurbsData.cps.size(); i++ )
    glVertex3f(nurbsData.cps[i].x, nurbsData.cps[i].y, nurbsData.cps[i].z);
  glEnd();

  glEnable(GL_LIGHTING);

#ifdef DEBUG
  tgCheckError("[tgNurbsSurface::DrawCPs()]");
#endif
}

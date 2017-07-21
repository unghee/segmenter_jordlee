#include "tgNurbsCurve.h"
#include <string>

using namespace TomGine;

tgNurbsCurve::tgNurbsCurve()
{
  std::string nurbs_vert = std::string(TOMGINE_SHADER) + "nurbscurve.vert";
  std::string cox_head = std::string(TOMGINE_SHADER) + "coxdeboor.c";
  std::string color_frag = std::string(TOMGINE_SHADER) + "color.frag";
  shNurbsCurve = tgNurbsCurve::LoadShader();
}

tgNurbsCurve::tgNurbsCurve(const tgNurbsCurvePatch &data)
{
  // setup NURBS shader
  std::string nurbs_vert = std::string(TOMGINE_SHADER) + "nurbscurve.vert";
  std::string cox_head = std::string(TOMGINE_SHADER) + "coxdeboor.c";
  std::string color_frag = std::string(TOMGINE_SHADER) + "color.frag";
  shNurbsCurve = tgNurbsCurve::LoadShader();

  Set(data);
}

tgNurbsCurve::~tgNurbsCurve()
{
  delete shNurbsCurve;
}

void tgNurbsCurve::Set(const tgNurbsCurvePatch &data)
{
  nurbsData = data;

  texKnots.Load(&nurbsData.knotsU[0], nurbsData.knotsU.size(), GL_R32F, GL_RED, GL_FLOAT);
  texCP.Load(&nurbsData.cps[0].x, nurbsData.cps.size(), GL_RGBA32F, GL_RGBA, GL_FLOAT);

  // setup NURBS shader
  shNurbsCurve->bind();
  shNurbsCurve->setUniform("order", (int) nurbsData.orderU);
  shNurbsCurve->setUniform("g_nknots", (int) nurbsData.knotsU.size());
  shNurbsCurve->setUniform("g_knots", 0);
  shNurbsCurve->setUniform("g_cps", 1);
  shNurbsCurve->unbind();

  Remesh(nurbsData.resU);
}

tgShader* tgNurbsCurve::LoadShader()
{
  std::string nurbs_vert = std::string(TOMGINE_SHADER) + "nurbscurve.vert";
  std::string cox_head = std::string(TOMGINE_SHADER) + "coxdeboor.c";
  std::string color_frag = std::string(TOMGINE_SHADER) + "color.frag";
  TomGine::tgShader* shader = new tgShader(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());
  return shader;
}

void tgNurbsCurve::Remesh(unsigned res)
{
  float x0 = nurbsData.knotsU[0];
  float w = nurbsData.knotsU[nurbsData.knotsU.size() - 1] - x0;
  float dx = w / (res - 1);
  tgVertex v;
  for( unsigned i = 0; i < res; i++ ) {
    v.pos.x = x0 + dx * i;
    printf("x: %f\n", v.pos.x);
    m_points.push_back(v);
  }
}

void tgNurbsCurve::SetCP(unsigned i, const vec4 &cpv)
{
  if( i >= nurbsData.cps.size() || i < 0 ) {
    printf("[tgNurbsCurve::SetCP] Warning: index out of bounds.\n");
    return;
  }
  nurbsData.cps[i] = cpv;
  texCP.Load(&nurbsData.cps[0].x, nurbsData.cps.size(), GL_RGBA32F, GL_RGBA, GL_FLOAT);
}

vec4 tgNurbsCurve::GetCP(unsigned i)
{
  if( i >= nurbsData.cps.size() || i < 0 ) {
    printf("[tgNurbsCurve::SetCP] Warning: index out of bounds.\n");
    return vec4(0.0, 0.0, 0.0, 0.0);
  }
  return nurbsData.cps[i];
}

void tgNurbsCurve::DrawVertices()
{
  // draw B-Spline surface
  shNurbsCurve->bind();

  texKnots.Bind(0);
  texCP.Bind(1);

  glBegin(GL_POINTS);
  for( unsigned i = 0; i < m_points.size(); i++ ) {
    glTexCoord2f(m_points[i].texCoord.x, m_points[i].texCoord.y);
    glVertex3f(m_points[i].pos.x, m_points[i].pos.y, m_points[i].pos.z);

  }
  glEnd();

  shNurbsCurve->unbind();
  glDisable(GL_TEXTURE_1D);
}

void tgNurbsCurve::DrawLines()
{
  // draw B-Spline surface
  shNurbsCurve->bind();

  texKnots.Bind(0);
  texCP.Bind(1);

  glBegin(GL_LINE_STRIP);
  for( unsigned i = 0; i < m_points.size(); i++ ) {
    glVertex3f(m_points[i].pos.x, m_points[i].pos.y, m_points[i].pos.z);
  }
  glEnd();

  shNurbsCurve->unbind();
  glDisable(GL_TEXTURE_1D);
}

void tgNurbsCurve::DrawCPs()
{
  glDisable(GL_LIGHTING);

  glBegin(GL_POINTS);
  for( unsigned i = 0; i < nurbsData.cps.size(); i++ )
    glVertex3f(nurbsData.cps[i].x, nurbsData.cps[i].y, nurbsData.cps[i].z);
  glEnd();

  glEnable(GL_LIGHTING);
}

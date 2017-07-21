#include "tgNurbsVolume.h"
#include <string>

using namespace TomGine;

tgNurbsVolume::tgNurbsVolume()
{
  std::string nurbs_vert = std::string(TOMGINE_SHADER) + "nurbsvolume.vert";
  std::string cox_head = std::string(TOMGINE_SHADER) + "coxdeboor.c";
  std::string color_frag = std::string(TOMGINE_SHADER) + "color.frag";
  shNurbsSurface = new tgShader(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());
}

tgNurbsVolume::tgNurbsVolume(const tgNurbsVolumePatch &data)
{
  // setup NURBS shader
  std::string nurbs_vert = std::string(TOMGINE_SHADER) + "nurbsvolume.vert";
  std::string cox_head = std::string(TOMGINE_SHADER) + "coxdeboor.c";
  std::string color_frag = std::string(TOMGINE_SHADER) + "color.frag";
  shNurbsSurface = new tgShader(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());

  Set(data);
}

tgNurbsVolume::~tgNurbsVolume()
{
  delete shNurbsSurface;
}

void tgNurbsVolume::Set(const tgNurbsVolumePatch &data)
{
  nurbsData = data;

  texKnotsU.Load(&nurbsData.knotsU[0], nurbsData.knotsU.size(), GL_R32F, GL_RED, GL_FLOAT);
  texKnotsV.Load(&nurbsData.knotsV[0], nurbsData.knotsV.size(), GL_R32F, GL_RED, GL_FLOAT);
  texKnotsW.Load(&nurbsData.knotsW[0], nurbsData.knotsW.size(), GL_R32F, GL_RED, GL_FLOAT);
  texCP.Load(&nurbsData.cps[0].x, nurbsData.ncpsU, nurbsData.ncpsV, nurbsData.ncpsW, GL_RGBA32F, GL_RGBA, GL_FLOAT);

  // setup NURBS shader
  shNurbsSurface->bind();
  shNurbsSurface->setUniform("orderU", (int) nurbsData.orderU);
  shNurbsSurface->setUniform("orderV", (int) nurbsData.orderV);
  shNurbsSurface->setUniform("orderW", (int) nurbsData.orderW);
  shNurbsSurface->setUniform("nknotsU", (int) nurbsData.knotsU.size());
  shNurbsSurface->setUniform("nknotsV", (int) nurbsData.knotsV.size());
  shNurbsSurface->setUniform("nknotsW", (int) nurbsData.knotsW.size());
  shNurbsSurface->setUniform("knotsU", 0);
  shNurbsSurface->setUniform("knotsV", 1);
  shNurbsSurface->setUniform("knotsW", 2);
  shNurbsSurface->setUniform("cps", 3);
  shNurbsSurface->unbind();

  Remesh(nurbsData.resU, nurbsData.resV, nurbsData.resW);
}

void tgNurbsVolume::Remesh(unsigned resU, unsigned resV, unsigned resW)
{
  float x0 = nurbsData.knotsU[0];
  float y0 = nurbsData.knotsV[0];
  float z0 = nurbsData.knotsW[0];
  float x = nurbsData.knotsU[nurbsData.knotsU.size() - 1] - x0;
  float y = nurbsData.knotsV[nurbsData.knotsV.size() - 1] - y0;
  float z = nurbsData.knotsW[nurbsData.knotsW.size() - 1] - z0;

  m_model.Clear();
  tgShapeCreator::CreatePlaneXY(m_model, x0, y0, z0, x, y, nurbsData.resU, nurbsData.resV);
  tgShapeCreator::CreatePlaneXY(m_model, x0, y0, z0 + z, x, y, nurbsData.resU, nurbsData.resV);
  tgShapeCreator::CreatePlaneYZ(m_model, x0, y0, z0, y, z, nurbsData.resV, nurbsData.resW);
  tgShapeCreator::CreatePlaneYZ(m_model, x0 + x, y0, z0, y, z, nurbsData.resV, nurbsData.resW);
  tgShapeCreator::CreatePlaneZX(m_model, x0, y0, z0, z, x, nurbsData.resW, nurbsData.resU);
  tgShapeCreator::CreatePlaneZX(m_model, x0, y0 + y, z0, z, x, nurbsData.resW, nurbsData.resU);
}

void tgNurbsVolume::SetCP(unsigned i, unsigned j, unsigned k, const vec4 &cpv)
{
  unsigned idx = k * nurbsData.ncpsU * nurbsData.ncpsV + j * nurbsData.ncpsU + i;
  SetCP(idx, cpv);
}
void tgNurbsVolume::SetCP(unsigned i, const vec4 &cpv)
{
  if( i >= nurbsData.cps.size() || i < 0 ) {
    printf("[tgNurbsVolume::UpdateCV] Warning: index out of bounds.\n");
    return;
  }
  nurbsData.cps[i] = cpv;
  texCP.Load(&nurbsData.cps[0].x, nurbsData.ncpsU, nurbsData.ncpsV, nurbsData.ncpsW, GL_RGBA32F, GL_RGBA, GL_FLOAT);
}

vec4 tgNurbsVolume::GetCP(unsigned i, unsigned j, unsigned k)
{
  unsigned idx = k * nurbsData.ncpsU * nurbsData.ncpsV + j * nurbsData.ncpsU + i;
  if( idx >= nurbsData.cps.size() || idx < 0 ) {
    printf("[tgNurbsVolume::GetCV] Warning: index out of bounds.\n");
    return vec4(0.0, 0.0, 0.0, 1.0);
  }
  return nurbsData.cps[idx];
}
vec4 tgNurbsVolume::GetCP(unsigned i)
{
  if( i >= nurbsData.cps.size() || i < 0 ) {
    printf("[tgNurbsVolume::GetCV] Warning: index out of bounds.\n");
    return vec4(0.0, 0.0, 0.0, 1.0);
  }
  return nurbsData.cps[i];
}

void tgNurbsVolume::DrawVertices()
{
  // draw NURBS volume
  shNurbsSurface->bind();

  texKnotsU.Bind(0);
  texKnotsV.Bind(1);
  texKnotsW.Bind(2);
  texCP.Bind(3);

  m_model.DrawVertices();

  shNurbsSurface->unbind();
  glDisable(GL_TEXTURE_1D);
  glDisable(GL_TEXTURE_3D);
}

void tgNurbsVolume::DrawFaces()
{
  // draw NURBS volume
  shNurbsSurface->bind();

  texKnotsU.Bind(0);
  texKnotsV.Bind(1);
  texKnotsW.Bind(2);
  texCP.Bind(3);

  m_model.DrawFaces();

  shNurbsSurface->unbind();
  glDisable(GL_TEXTURE_1D);
  glDisable(GL_TEXTURE_3D);
}

void tgNurbsVolume::DrawCPs()
{
  glDisable(GL_LIGHTING);

  glBegin(GL_POINTS);
  for( unsigned i = 0; i < nurbsData.cps.size(); i++ )
    glVertex3f(nurbsData.cps[i].x, nurbsData.cps[i].y, nurbsData.cps[i].z);
  glEnd();

  glEnable(GL_LIGHTING);
}

void tgNurbsVolume::CreateBox()
{

  nurbsData.orderU = 2;
  nurbsData.orderV = 2;
  nurbsData.orderW = 2;

  nurbsData.knotsU.push_back(0.0);
  nurbsData.knotsU.push_back(0.0);
  nurbsData.knotsU.push_back(0.0);
  nurbsData.knotsU.push_back(1.0);
  nurbsData.knotsU.push_back(1.0);
  nurbsData.knotsU.push_back(1.0);

  nurbsData.knotsV.push_back(0.0);
  nurbsData.knotsV.push_back(0.0);
  nurbsData.knotsV.push_back(0.0);
  nurbsData.knotsV.push_back(1.0);
  nurbsData.knotsV.push_back(1.0);
  nurbsData.knotsV.push_back(1.0);

  nurbsData.knotsW.push_back(0.0);
  nurbsData.knotsW.push_back(0.0);
  nurbsData.knotsW.push_back(0.0);
  nurbsData.knotsW.push_back(1.0);
  nurbsData.knotsW.push_back(1.0);
  nurbsData.knotsW.push_back(1.0);

  float y, z;
  y = -1.0;
  z = -1.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));
  y = 0.0;
  z = -1.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));
  y = 1.0;
  z = -1.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));

  y = -1.0;
  z = 0.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));
  y = 0.0;
  z = 0.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));
  y = 1.0;
  z = 0.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));

  y = -1.0;
  z = 1.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));
  y = 0.0;
  z = 1.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));
  y = 1.0;
  z = 1.0;
  nurbsData.cps.push_back(vec4(-1.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(0.0, y, z, 1.0));
  nurbsData.cps.push_back(vec4(1.0, y, z, 1.0));

  Set(nurbsData);
}

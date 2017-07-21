/**
 * $Id$
 *
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */


#include "ZAdaptiveNormals.hh"

namespace pclA 
{

using namespace std;

float ZAdaptiveNormals::NaN  = std::numeric_limits<float>::quiet_NaN(); 


/********************** ZAdaptiveNormals ************************
 * Constructor/Destructor
 */
ZAdaptiveNormals::ZAdaptiveNormals(Parameter p)
{
  setParameter(p);
  param.kernel_radius[0] = 2;
  param.kernel_radius[1] = 2;
  param.kernel_radius[2] = 2;
  param.kernel_radius[3] = 3;
  param.kernel_radius[4] = 4;
  param.kernel_radius[5] = 5;
  param.kernel_radius[6] = 6;
  param.kernel_radius[7] = 7;
}

ZAdaptiveNormals::~ZAdaptiveNormals()
{
}


/************************** PRIVATE ************************/


/**
 * ComputeCovarianceMatrix
 */
void ZAdaptiveNormals::ComputeCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
      const std::vector<int> &indices, const Eigen::Vector4f &mean, Eigen::Matrix3f &cov)
{
  cov.setZero ();

  for (unsigned i = 0; i < indices.size (); ++i)
  {
    Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - mean;

    cov(1,1) += pt.y () * pt.y ();
    cov(1,2) += pt.y () * pt.z ();

    cov(2,2) += pt.z () * pt.z ();

    pt *= pt.x ();
    cov(0,0) += pt.x ();
    cov(0,1) += pt.y ();
    cov(0,2) += pt.z ();
  }

  cov(1,0) = cov(0,1);
  cov(2,0) = cov(0,2);
  cov(2,1) = cov(1,2);
}

/**
 * GetIndices
 */
void ZAdaptiveNormals::GetIndices(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                  int u, int v, int kernel, 
                                  std::vector<int> &indices)
{
  int idx;
  indices.clear();

  pcl::PointXYZRGB &pt = cloud.points[GetIdx(u,v)]; 

  if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
    return;

  for (int y=v-kernel; y<=v+kernel; y++) {
    for (int x=u-kernel; x<=u+kernel; x++) {
      if (x>0 && y>0 && x<width && y<height) {
        idx = GetIdx(x,y);
        pcl::PointXYZRGB &pt1 = cloud.points[idx];
        if(!isnan(pt1.z)) {
          float new_sqr_radius = sqr_radius;
          if(param.adaptive) {
            int dist = (int) (pt1.z*2); // *2 => every 0.5 meter another kernel radius
            float val = param.kappa*param.kernel_radius[dist]*pt1.z + param.d;
            new_sqr_radius = val*val;
          }
        
          if ((pt.getVector3fMap()-pt1.getVector3fMap()).squaredNorm() < new_sqr_radius)
            indices.push_back(idx);
        }
      }
    }
  }
}

/**
 * ComputeNormal
 */
float ZAdaptiveNormals::ComputeNormal(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                      std::vector<int> &indices, 
                                      Eigen::Vector4f &mean, 
                                      Eigen::Matrix3f &cov, 
                                      Eigen::Vector3f &eigen_values, 
                                      Eigen::Matrix3f &eigen_vectors, 
                                      float &eigsum)
{
  if (indices.size()<4)
    return NaN;

  mean.setZero();
  for (unsigned j=0; j<indices.size(); j++)
    mean += cloud.points[indices[j]].getVector4fMap();
  mean /= (float)indices.size();
  mean[3]=0.;

  ComputeCovarianceMatrix (cloud, indices, mean, cov);

  pcl::eigen33 (cov, eigen_vectors, eigen_values);
  eigsum = eigen_values.sum();
  if (eigsum != 0)
    return fabs (eigen_values[0] / eigsum );
  
  return NaN;
}


/**
 * EstimateNormals
 */
void ZAdaptiveNormals::EstimateNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                       pcl::PointCloud<pcl::Normal> &normals)
{
  Eigen::Vector3f n0(0.,0.,1.);
  Eigen::Vector4f mean;
  EIGEN_ALIGN16 Eigen::Matrix3f cov;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  std::vector< int > indices;
  float eigsum;
  bool havenan=false;

  #pragma omp parallel for private(mean,cov,eigen_values,eigen_vectors,indices,eigsum)
  for (int v=0; v<height; v++) {
    for (int u=0; u<width; u++) {
      GetIndices(cloud, u,v, param.kernel, indices);

      int idx = GetIdx(u,v);
      pcl::PointXYZRGB &pt = cloud.points[idx];
      pcl::Normal &n = normals.points[idx];

      if (indices.size()<4)
      {
        havenan=true;
        n.normal[0] = NaN;
        pt.x = NaN;
        continue;
      }

      n.curvature = ComputeNormal(cloud, indices, mean, cov, eigen_values, eigen_vectors, eigsum);

      n.normal[0] = eigen_vectors (0,0);
      n.normal[1] = eigen_vectors (1,0);
      n.normal[2] = eigen_vectors (2,0);


      if (n.getNormalVector3fMap().dot(pt.getVector3fMap()) > 0) {
        n.getNormalVector4fMap() *= -1;
        n.getNormalVector4fMap()[3] = 0;
        n.getNormalVector4fMap()[3] = -1 * n.getNormalVector4fMap().dot(pt.getVector4fMap());
      }
    }
  }

  if (havenan)
  {
    cloud.is_dense=false;
    normals.is_dense=false;
  }
}




/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void ZAdaptiveNormals::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error ("[ZAdaptiveNormals::compute] Need an organized point cloud!");

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  normals.reset(new pcl::PointCloud<pcl::Normal>);
  normals->points.resize(cloud->points.size());
  normals->width = cloud->width;
  normals->height = cloud->height;
  normals->is_dense = cloud->is_dense;
}

/**
 * compute the normals
 */
void ZAdaptiveNormals::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[ZAdaptiveNormals::compute] No point cloud available!");
  EstimateNormals(*cloud, *normals);
}

/**
 * compute the normals using a mask
 */
void ZAdaptiveNormals::compute(const vector<int> &mask)
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[ZAdaptiveNormals::compute] No point cloud available!");
  cout<<"[ZAdaptiveNormals::compute] Error: Not yet implemented."<<endl;
  exit(0);
}


/**
 * getNormals
 */
void ZAdaptiveNormals::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  _normals = normals;
}

/**
 * setParameter
 */
void ZAdaptiveNormals::setParameter(Parameter p)
{
  param = p;
  sqr_radius = p.radius*p.radius;
}


} //-- THE END --


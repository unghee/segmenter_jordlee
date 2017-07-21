/**
 * $Id$
 */

#ifndef RELATION_CONTOUR_NORMALS_DISTANCE_HH
#define RELATION_CONTOUR_NORMALS_DISTANCE_HH

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree.h>

#include "v4r/SurfaceClustering/PPlane.h"
#include "v4r/SurfaceUtils/SurfaceModel.hpp"

namespace surface 
{


class ContourNormalsDistance
{
public:
  class Parameter
  {
  public:
    float pcntContourPoints;  // percentage of nearest contour points used 
    Parameter(float _pcntContourPoints=.2)
     : pcntContourPoints(_pcntContourPoints) {} 
  };

private:
  int width, height;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  std::vector<int> contour1, contour2;
  std::vector<std::pair<int, float> > nnIdxDist12, nnIdxDist21;

  static unsigned idcnt;
  std::vector<unsigned> idMap;

  void GetContourPoints(const std::vector<int> &indices, std::vector<int> &contour);
  float ComputeNearestNeighbours(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
        const std::vector<int> &ptIndices1, const std::vector<int> &ptIndices2,
        std::vector<int> &contour1, std::vector<int> &contour2,
        std::vector<std::pair<int, float> > &nnIdxDist);
  void ComputeNormalsDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
         const surface::SurfaceModel &in1, const surface::SurfaceModel &in2, 
         std::vector<std::pair<int, float> > &nnIdxDist12, 
         float &cosDeltaAngle, float &distNormal, float minDist);


  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Parameter param;

  ContourNormalsDistance(Parameter p=Parameter());
  ~ContourNormalsDistance();

  void setParameter(Parameter &p);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  
  bool compute(const surface::SurfaceModel &in1,
               const surface::SurfaceModel &in2,
               float &cosDeltaAngle,
               float &distNormal,
               float &minDist);

  bool computeOctree(const surface::SurfaceModel &in1,
                     const surface::SurfaceModel &in2,
                     float &cosDeltaAngle,
                     float &distNormal,
                     float &minDist);

};



/*********************** INLINE METHODES **************************/

inline int ContourNormalsDistance::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short ContourNormalsDistance::X(int idx)
{
  return idx%width;
}

inline short ContourNormalsDistance::Y(int idx)
{
  return idx/width;
}



}

#endif


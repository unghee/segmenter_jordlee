/**
 * $Id$
 */


#include "ContourNormalsDistance.hh"

namespace surface 
{

//using namespace std;

float CONTOUR_MAX_DISTANCE = 1.15;   // maximum distance compared to min-distance
unsigned ContourNormalsDistance::idcnt = 0;

/********************** ContourNormalsDistance ************************
 * Constructor/Destructor
 */
ContourNormalsDistance::ContourNormalsDistance(Parameter p)
 : width(0), height(0)
{
  setParameter(p);
}

ContourNormalsDistance::~ContourNormalsDistance()
{
}



/************************** PRIVATE ************************/

/**
 * GetContourPoints
 * Attention! 
 * Returns indices from indices!
 */
void ContourNormalsDistance::GetContourPoints(const std::vector<int> &indices, std::vector<int> &contour)
{
  int x, y;

  contour.clear();

  for( unsigned i = 0; i < indices.size(); i++ ) {
    x = X(indices[i]);
    y = Y(indices[i]);

    if( x==0 || idMap[GetIdx(x - 1, y)] != ContourNormalsDistance::idcnt )
      contour.push_back(i);
    else if( x==width-1 || idMap[GetIdx(x + 1, y)] != ContourNormalsDistance::idcnt )
      contour.push_back(i);
    else if( y==0 || idMap[GetIdx(x, y - 1)] != ContourNormalsDistance::idcnt )
      contour.push_back(i);
    else if( y==height-1 || idMap[GetIdx(x, y + 1)] != ContourNormalsDistance::idcnt)
      contour.push_back(i);
  }
}

/**
 * ComputeNearestNeighbours
 */
float ContourNormalsDistance::ComputeNearestNeighbours(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
      const std::vector<int> &ptIndices1, const std::vector<int> &ptIndices2,
      std::vector<int> &contour1, std::vector<int> &contour2, 
      std::vector<std::pair<int, float> > &nnIdxDist)
{
  int idx=0;
  float dist, minDist, minDist2=FLT_MAX;
  nnIdxDist.resize(contour1.size());

  for (unsigned i=0; i<contour1.size(); i++)
  {
    minDist=FLT_MAX;
    pcl::PointXYZRGB &pt1 = cloud.points[ ptIndices1[contour1[i]] ];

    for (unsigned j=0; j<contour2.size(); j++)
    {
      dist = (pt1.getVector3fMap() - cloud.points[ ptIndices2[contour2[j]] ].getVector3fMap()).norm();
      if (dist<minDist)
      {
        minDist = dist;
        idx = j;
      }
    }

    nnIdxDist[i].first = contour2[idx];
    nnIdxDist[i].second = minDist;

    if (minDist<minDist2)
      minDist2 = minDist;
  }

  return minDist2;
}

/**
 * ComputeNormalsDistance
 */
void ContourNormalsDistance::ComputeNormalsDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
      const surface::SurfaceModel &in1, const surface::SurfaceModel &in2, 
      std::vector<std::pair<int, float> > &nnIdxDist12, 
      float &cosDeltaAngle, float &distNormal, float minDist)
{
  unsigned cnt=0;
  float cosAlpha;

  minDist = minDist*(1.+param.pcntContourPoints);
  cosDeltaAngle=0;
  distNormal=0;

  for (unsigned i=0; i<nnIdxDist12.size(); i++)
  {
    if (nnIdxDist12[i].second < minDist)
    {
      int idx = nnIdxDist12[i].first;
      cosAlpha = in1.normals[i].dot(in2.normals[idx]);
      cosDeltaAngle += (cosAlpha<0?1.+cosAlpha:cosAlpha);
      distNormal += fabs( Plane::NormalPointDist(&cloud.points[in1.indices[i]].x, 
                                                 &in1.normals[i][0], 
                                                 &cloud.points[in2.indices[idx]].x));      
      cnt++;
    }
  }

  if (cnt>0)
  {
    cosDeltaAngle /= (float)cnt;
    distNormal /= (float)cnt;
  }
}





/************************** PUBLIC *************************/

bool ContourNormalsDistance::computeOctree(const surface::SurfaceModel &in1, 
                                           const surface::SurfaceModel &in2, 
                                           float &cosDeltaAngle, 
                                           float &distNormal, 
                                           float &minDist)
{
  if(in1.contour.size() == 0 || in2.contour.size() == 0) {
    printf("[ContourNormalsDistance::computeOctree] Warning: Input surface contour is empty: Return false.\n");
    return false;
  }

  distNormal = 0.0f;
  cosDeltaAngle = 0.0f;
  int distNormalCnt = 0;
    
  // Take the one as octree with more contour pixels!!!
  surface::SurfaceModel surf1;   // => octree
  surface::SurfaceModel surf2;   // => search points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr octreeCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(in1.contour.size() > in2.contour.size()){
    surf1 = in1;
    surf2 = in2;
  }
  else {
    surf1 = in2;
    surf2 = in1;
  }
  pcl::copyPointCloud(*cloud, surf1.contour, *octreeCloud);
  
  // create octree from octreecloud
  float resolution = 4.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);
  octree.setInputCloud(octreeCloud);
  octree.addPointsFromInputCloud();

  // find indices of nearest points
  int surf1NP = -1;
  int surf2NP = -1;
  minDist = 10.0f;
  for(unsigned i=0; i<surf2.contour.size(); i++) {
    pcl::PointXYZRGB searchPoint;
    searchPoint = cloud->points[surf2.contour[i]];
    
    // K-nearest neighbour search
    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    if(sqrt(pointNKNSquaredDistance[0]) < minDist) {
      minDist = sqrt(pointNKNSquaredDistance[0]);
      surf1NP = pointIdxNKNSearch[0];
      surf2NP = i;
    }
  }
  
  // estimate direction
  bool equal = true;
  int n1l, n1r, n2l, n2r;
  n1l = surf1NP;
  n1r = surf1NP;
  n2l = surf2NP;
  n2r = surf2NP;
  float normal, cross;
    
  int maxNr = surf2.contour.size()/2;
  while(equal) {
    maxNr--;
    n1l--;
    n1r++;
    n2l--;
    n2r++;

    // check validity of points
    if(n1l < 0)
      n1l = surf1.contour.size()-1;
    if(n1r >= (int) surf1.contour.size())
      n1r = 0;
    if(n2l < 0)
      n2l = surf2.contour.size()-1;
    if(n2r >= (int) surf2.contour.size())
      n2r = 0;
    
    Eigen::Vector3f dist_normal_l = cloud->points[surf1.contour[n1l]].getVector3fMap() - cloud->points[surf2.contour[n2l]].getVector3fMap();
    Eigen::Vector3f dist_normal_r = cloud->points[surf1.contour[n1r]].getVector3fMap() - cloud->points[surf2.contour[n2r]].getVector3fMap();
    Eigen::Vector3f dist_cross_l = cloud->points[surf1.contour[n1l]].getVector3fMap() - cloud->points[surf2.contour[n2r]].getVector3fMap();
    Eigen::Vector3f dist_cross_r = cloud->points[surf1.contour[n1r]].getVector3fMap() - cloud->points[surf2.contour[n2l]].getVector3fMap();
    
    normal = dist_normal_l.norm() + dist_normal_r.norm();
    cross = dist_cross_l.norm() + dist_cross_r.norm();
    
    if(fabs(normal - cross) > 0.001 || maxNr < 0)
      equal = false;
  }
  
  // create all relations between neighbors
  int dir;
  if(normal < cross)
    dir = 1;
  else
    dir = -1;

  // left side (with minimum point)
  bool left_run = true;
  maxNr = surf2.contour.size()/2;
  n1l = surf1NP+=dir;
  n2l = surf2NP+=dir;
  while(left_run) {
    maxNr--;
    n1l-=dir;
    n2l-=dir;
    
    // check validity of points
    if(n1l < 0)
      n1l = surf1.contour.size()-1;
    if(n1l >= (int) surf1.contour.size())
      n1l = 0;
    if(n2l < 0)
      n2l = surf2.contour.size()-1;
    if(n2l >= (int) surf2.contour.size())
      n2l = 0;
       
    float distance_l = (cloud->points[surf1.contour[n1l]].getVector3fMap() - cloud->points[surf2.contour[n2l]].getVector3fMap()).norm();

    distNormalCnt++;
    distNormal += fabs( Plane::NormalPointDist(&cloud->points[surf1.indices[n1l]].x, 
                                               &surf1.normals[n1l][0], 
                                               &cloud->points[surf2.indices[n2l]].x));      
    float cosAlpha = surf1.normals[n1l].dot(surf2.normals[n2l]);
    cosDeltaAngle += (cosAlpha<0?1.+cosAlpha:cosAlpha);    
    

    if(distance_l > minDist*CONTOUR_MAX_DISTANCE || maxNr < 0)
      left_run=false;
  } 
  
  // calculate right side relations
  n1r = surf1NP;
  n2r = surf2NP;
  maxNr = surf2.contour.size();
  bool right_run = true;
  while(right_run) {
    maxNr--;
    n1r+=dir;
    n2r+=dir;
    
    // check validity of points
    if(n1r < 0)
      n1r = surf1.contour.size()-1;
    if(n1r >= (int) surf1.contour.size())
      n1r = 0;
    if(n2r < 0)
      n2r = surf2.contour.size()-1;
    if(n2r >= (int) surf2.contour.size())
      n2r = 0;

    float distance_r = (cloud->points[surf1.contour[n1r]].getVector3fMap() - cloud->points[surf2.contour[n2r]].getVector3fMap()).norm();
    distNormalCnt++;
    distNormal += fabs(Plane::NormalPointDist(&cloud->points[surf1.indices[n1r]].x, 
                                              &surf1.normals[n1r][0], 
                                              &cloud->points[surf2.indices[n2r]].x));      
    float cosAlpha = surf1.normals[n1r].dot(surf2.normals[n2r]);
      cosDeltaAngle += (cosAlpha<0?1.+cosAlpha:cosAlpha);    

    if(distance_r > minDist*CONTOUR_MAX_DISTANCE || maxNr < 0)
      right_run=false;
  } 

  distNormal/= (float) distNormalCnt;
  cosDeltaAngle /= (float) distNormalCnt;
  return true;
}



/**
 * compute
 */
bool ContourNormalsDistance::compute(const surface::SurfaceModel &in1, 
                                     const surface::SurfaceModel &in2, 
                                     float &cosDeltaAngle, 
                                     float &distNormal, 
                                     float &minDist)
{
  if (cloud.get()==0 || width==0 || height==0) 
    throw std::runtime_error ("[ContourNormalsDistance::compute] Input point cloud not set!");

  bool calculate_octree_version = true;
  if(calculate_octree_version)
    computeOctree(in1, in2, cosDeltaAngle, distNormal, minDist);
  else {
    ContourNormalsDistance::idcnt++;
    GetContourPoints(in1.indices, contour1);
    GetContourPoints(in2.indices, contour2);

    float minDist12 = ComputeNearestNeighbours(*cloud, in1.indices, in2.indices, contour1, contour2, nnIdxDist12);
    float minDist21 = ComputeNearestNeighbours(*cloud, in2.indices, in1.indices, contour2, contour1, nnIdxDist21);

    float cosDeltaAngle1, cosDeltaAngle2;
    float distNormal1, distNormal2;

    ComputeNormalsDistance(*cloud, in1, in2, nnIdxDist12, cosDeltaAngle1, distNormal1, minDist12);
    ComputeNormalsDistance(*cloud, in2, in1, nnIdxDist21, cosDeltaAngle2, distNormal2, minDist21);

    cosDeltaAngle = (cosDeltaAngle1 + cosDeltaAngle2) / 2.;
    distNormal = (distNormal1 + distNormal2) / 2.;
    minDist = fmin(minDist12, minDist21);
    
    if(cosDeltaAngle != cosDeltaAngle || distNormal != distNormal)
      return false;
  }
  return true;
}

/**
 * setInputCloud
 */
void ContourNormalsDistance::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  idMap.clear();
  idMap.resize(width*height,0);
  ContourNormalsDistance::idcnt=0;
}

/**
 * setParameter
 */
void ContourNormalsDistance::setParameter(Parameter &p)
{
  param = p;
}



} //-- THE END --


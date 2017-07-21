/**
 * @file GraphLink.h
 * @author Richtsfeld Andreas
 * @date March 2011
 * @version 0.1
 * @brief Datastructure for links in the object representation.
 */

#ifndef Z_GRAPH_LINK_H
#define Z_GRAPH_LINK_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace Z
{

  
/**
 * @brief Link between two nodes with a probabibility value.
 */
class GraphLink
{
public:
  cv::Point3d node[2];
  double probability;
};

}
#endif


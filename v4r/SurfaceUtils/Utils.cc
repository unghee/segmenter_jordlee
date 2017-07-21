/**
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


#include "Utils.hh"


namespace surface 
{

using namespace std;

/********************** Utils ************************
 * Constructor/Destructor
 */
Utils::Utils()
{
}

Utils::~Utils()
{
}

/************************** PRIVATE ************************/




/************************** PUBLIC *************************/

/**
 * ConvertCloud2Image
 */
void Utils::ConvertCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = cloud.width;
  unsigned pcHeight = cloud.height;
  unsigned position = 0;
  RGBValue color;

  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for( unsigned row = 0; row < pcHeight; row++ ) 
  {
    for( unsigned col = 0; col < pcWidth; col++ ) 
  {
      cv::Vec3b &cvp = image(row, col);
      const pcl::PointXYZRGB &pt = cloud(col,row);

      if( pt.rgb != pt.rgb ) 
      {
        cvp[0] = 0;
        cvp[1] = 0;
        cvp[2] = 0;
      } 
      else 
      {
        color.float_value = pt.rgb;
        cvp[2] = color.r;
        cvp[1] = color.g;
        cvp[0] = color.b;
      }
    }
  }
}



} //-- THE END --


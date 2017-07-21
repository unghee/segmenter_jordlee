/**
 * @file VisionCore.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2006, 2010
 * @version 0.1
 * @brief Vision Core for perceptual grouping.
 *
 *
 * TODO - for each principle and/or gestalt have a SetupParameters() function
 * TODO   called after getting the image
 * TODO - more gestalt principles, color!
 * TODO - feedback
 * TODO - different gestalt principles produce the same groups
 */

#ifndef Z_VISION_CORE_HH
#define Z_VISION_CORE_HH

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "Namespace.hh"
#include "Array.hh"
#include "Config.hh"
#include "Color.hh"
#include "Vector.hh"
#include "Gestalt.hh"
#include "GestaltPrinciple.hh"
#include "VoteImage.hh"

namespace Z
{

using namespace VEC;

/**
 * @class VisionCore Representing the perceptual grouping system.
 * @brief Representing the perceptual grouping system.
 */
class VisionCore
{
private:
  Config config;              ///< configuration of the perceptual grouping
  const IplImage *img;        ///< iplImage
  double realRuntime;         ///< Estimated runtime
  VoteImage *vote_img;        ///< Vote image for all Gestalt principles.


  GestaltPrinciple* principles[GestaltPrinciple::MAX_TYPE];   ///< principle list 
  Array<Gestalt*> gestalts[Gestalt::MAX_TYPE];                ///< gestalt list 
  Array<Gestalt*> ranked_gestalts[Gestalt::MAX_TYPE];         ///< ranked_gestalt list

  void InitGestaltPrinciples();

public:

  double p_e;                                   ///< probability of an edgel 				TODO das sollte hier weg => für sig-Berechnung
  double p_ee;                                  ///< probability of an edgel given another edgel

  bool use_masking;                             ///< use masking of gestalts for stereo processing. 	TODO sollte für alle Gestalts eigens def. werden.
  Vector2 roi_center;                           ///< center of the region of interest			TODO überprüfen was das macht? Wieso public?
  double roi_sigma;                             ///< sigma of the region of interest			TODO überprüfen was das macht? Wieso public?


public:
  VisionCore(const string &config_name = "");
  ~VisionCore();
  void Configure(const string &config_name);
  Config& GetConfig() {return config;}                                                  ///< Return configuration
  void NewImage(const IplImage *new_img);
  bool HaveImage() {return img != 0;}                                                   ///< Return true, if have image.
  const IplImage* GetImage() {return img;}                                              ///< Returns image (openCV ipl-image)
  int ImageArea() {return img->width*img->height;}                                      ///< Return image area (width*height)
  void EnableGestaltPrinciple(GestaltPrinciple::Type p);
  void DisableGestaltPrinciple(GestaltPrinciple::Type p);
  bool IsEnabledGestaltPrinciple(GestaltPrinciple::Type p);
  RGBColor Pixel(int x, int y)                                                          /// TODO Sollte das hier nicht zu den Draw-Sachen? geht das?
  {
    char *p = &img->imageData[y*img->widthStep + x*img->nChannels];
    return RGBColor(p[0], p[1], p[2]);
  }
  void ProcessImage();
  void ProcessImage(int runtime_ms, float ca, float co);
  void ClearGestalts();
  void Draw(int detail = 0);
  void DrawImage();
  void DrawAllGestalts(int detail = 0);
  void DrawGestalts(Gestalt::Type type, int detail = 0, bool masking = false);
  void DrawGestalt(Gestalt::Type type, unsigned num, int detail = 0, bool masking = false);
  void DrawGestaltInfo(Gestalt::Type type, unsigned num);
  void DrawPrinciple(GestaltPrinciple::Type type, int detail = 0);
  const char* GetInfo(Gestalt::Type type, int id);
  const char* GetGestaltTypeName(Gestalt::Type type);
  const char* GetGestaltListInfo();
  unsigned PickGestaltAt(Gestalt::Type type, int x, int y, unsigned start_at, bool reject_masked = false);
  double RunTime();
  void PrintRunTime();
  void SetROI(const Vector2 center, double sigma);
  void NewGestalt(GestaltPrinciple::Type type, Z::Gestalt* g, bool inform = true);
  void InformNewGestalt(Gestalt::Type type, unsigned id);
  void InformNewIntersection(unsigned sline, Array<VoteImage::Elem> iscts);

  unsigned IW() {return img->width;}                                                                    ///< Returns the width of the image in pixel (unsigned).
  unsigned IH() {return img->height;}                                                                   ///< Returns the height of the image in pixel (integer).
  Array<Gestalt*>* Gestalts() {return gestalts;}                                                        ///< Return Gestalt array
  Array<Gestalt*>& Gestalts(Gestalt::Type type) {return gestalts[type];}                                ///< Returns Gestalt array of "type"
  Gestalt* Gestalts(Gestalt::Type type, unsigned id) {return gestalts[type][id];}                       ///< Returns Gestalt of "type" and "id"
  Array<Gestalt*>& RankedGestalts(Gestalt::Type type) {return ranked_gestalts[type];}                   ///< Returns ranked Gestalt array of "type"
  Gestalt* RankedGestalts(Gestalt::Type type, unsigned id) {return ranked_gestalts[type][id];}          ///< Returns ranked Gestat of "type" and "id"
  unsigned NumGestalts(Gestalt::Type type) {return gestalts[type].Size();}                              ///< Ruturns number of Gestalts of "type"
  GestaltPrinciple* Principles(GestaltPrinciple::Type type) {return principles[type];}                  ///< Returns Gestalt principle of "type"
  VoteImage* VI() {return vote_img;}
  
  
  /// For object segmentation
  void SetBoundarySegments(int b_idx, std::vector<int> &boundary);
  void ProcessBoundarySegments(int runtime_ms);
  void GetResults();
};

}

#endif


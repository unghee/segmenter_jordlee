/**
 * @file GestaltPrinciple.hh
 * @author Michael Zillich, Anddreas Richtsfeld
 * @date 2006, 2010
 * @version 0.1
 * @brief Gestalt principle header
 */

#ifndef Z_GESTALT_PRINCIPLE_HH
#define Z_GESTALT_PRINCIPLE_HH

#include "Config.hh"
#include "Gestalt.hh"
#include "VoteImage.hh"

namespace Z
{

// class VisionCore;

// When creating rectangles, we use a method, where we delete short lines to create quadrangles.
// After meassuring the maximum line length, we delete all lines which are LENGTH_THR_FACTOR*maxLineLength
const static double LENGTH_THR_FACTOR = 0.25;

// When creating rectangles we use a threshold for the minimum parallelity of opposing edges
// to prune really accidential results
const static double MIN_PARALLELITY = 0.5;



/**
 * @brief Class GestaltPrinciple
 */
class GestaltPrinciple
{
public:
  enum Type						///< Type of Gestalt principle
  {
    FORM_SEGMENTS,
    FORM_LINES,
    FORM_E_JUNCTIONS,					// TODO not fail save: must run, before ellipse creation.
    FORM_ARCS,
    FORM_ARC_JUNCTIONS,
    FORM_CONVEX_ARC_GROUPS,
    FORM_ELLIPSES,
    FORM_CIRCLES,
    FORM_EXT_ELLIPSES,
    FORM_CYLINDERS,
    FORM_CONES,
    FORM_JUNCTIONS,
    FORM_CORNERS,
    FORM_CLOSURES,
    FORM_RECTANGLES,
    FORM_FLAPS,
    FORM_FLAPS_ARI,
    FORM_CUBES,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

private:
  double runtime;   					///< processing runtime in [s]
  struct timespec startTime;				///< start time [timespec]
  struct timespec endTime;				///< end time [timespec]

protected:
  VisionCore *core;					///< Vision core
  Array<unsigned> next_principles;			///< Next principles, to be informed of new gestalts		TODO used?

  /**
   * @brief Rank Gestalts according to their significance value
   * @param type Type of Gestalt
   * @param compar Compare function
   */
  void RankGestalts(Gestalt::Type type, int(*compar)(const void *, const void *));

public:

	/**
	 * @brief Returns the name of a given Gestalt principle type.
	 * @param t Type of Gestalt principle
	 * @return Name of Gestalt principle
	 */
  static const char* TypeName(Type t);

	/**
	 * @brief Return the enum type of a given Gestalt principle type name.
	 * @param type_name Type name of the Gestalt principle
	 * @return Gestalt principle type
	 */
  static Type EnumType(const char *type_name);

  /**
   * @brief Constructor of class GestaltPrinciple.
   * @param vc Vision core
   */
  GestaltPrinciple(VisionCore *vc);

  /**
   * @brief Destructor of class Gestalt Principle
   */
  virtual ~GestaltPrinciple() {}

  /**
   * @brief Reset the Gestalt principle for new calculation.
   */
  virtual void Reset() {};

	  /**
   * @brief Operate the Gestalt principle before processing starts.
   */
  virtual void PreOperate() {};
	
  /**
   * @brief Operate the Gestalt principle.
   * @param incremental Operate it incremental
   */
  virtual void Operate(bool incremental) {};

  /**
   * @brief Operate the Gestalt principle after processing ends.
   */
  virtual void PostOperate() {};

  /**
   * @brief Returns false, when no new processing is needed.
   * @return Returns false, when no new processing is needed.
   */
  virtual bool NeedsOperate() {return true;}

  /**
   * @brief Inform Gestalt principle about new lower-level Gestalts
   * @param type Type of Gestalt
   * @param idx Index of Gestalt
   */
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx) {}

  /**
   * @brief Inform Gestalt principle about new lower-level Gestalts
   * @param type Type of Gestalt
   * @param idx Index of Gestalt
   */
  virtual void CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts) {}

  /**
   * @brief Draw Gestalt principle
   * @param detail Degree of detail
   */
  virtual void Draw(int detail = 0) {}

	/**
	 * @brief Set Canny parameters for Gestalt principle FORM_SEGMENTS
	 * @param a Alpha value
	 * @param o Omega value
	 */
	virtual void SetCanny(float a, float o) {}

  /**
   * @brief Return runtime of Gestalt principle
   * @return Runtime of processing
   */
  double RunTime() {return runtime;}

  /**
   * @brief Reset processing runtime.
   */
  void ResetRunTime() {runtime = 0.;}

  /**
   * @brief Set runtime.
   * @param t Runtime in seconds
   */
  void SetRunTime(double t) {runtime = t;}

  /**
   * @brief Add runtime.
   * @param t Runtime to add in seconds.
   */
  void AddRunTime(double t) {runtime += t;}
  
  void StartRunTime();

  void StopRunTime();
};

}

#endif


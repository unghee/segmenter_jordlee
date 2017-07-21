/**
 * @file FormEJunctions.hh
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormEJunctions (ellipse junction).
 **/

#ifndef Z_FORM_E_JUNCTIONS_HH
#define Z_FORM_E_JUNCTIONS_HH

#include "Line.hh"
#include "Ellipse.hh"
#include "EJunction.hh"
#include "GestaltPrinciple.hh"

namespace Z
{


/**
 * @brief Class FormEJunctions
 */
class FormEJunctions : public GestaltPrinciple
{
private:
  Array<VoteImage::Elem> iscts;
	bool init;
	Array<Gestalt::Type> typ;
	Array<unsigned> id;
	bool initialized;
	
	void PrintCreateJunctions(unsigned sline, Array<VoteImage::Elem> &iscts);		/// TODO only for debugging
	bool NoEJunctionYet(Line *line, Ellipse *ellipse);
	void Mask();
	void CreateJunction(unsigned sline, VoteImage::Elem &isct);

public:

	FormEJunctions(VisionCore *vc);
  virtual ~FormEJunctions();
	virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
	void Initialize();
  virtual void Reset();
  virtual void PostOperate();
	virtual void CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts);

	void UpdateEJunctions(unsigned idx);
};

}

#endif



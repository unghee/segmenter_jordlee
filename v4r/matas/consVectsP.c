#include "gfPars.h"

FloatVector_t ConstFloatVector(int number)
{
   FloatVector_t FloatVector;
   
   FloatVector= (FloatVector_t)
        malloc(sizeof( FloatVector->items)+number*sizeof(float));
   FloatVector->items=number;

   return FloatVector;

}
VoidVector_t ConstVoidVector(int number)
{
   VoidVector_t VoidVector;
   
   VoidVector= (VoidVector_t)
        malloc(sizeof( VoidVector->items)+number*sizeof(void*));
   VoidVector->items=number;

   return VoidVector;

}

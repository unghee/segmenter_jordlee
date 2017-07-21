/**
 * @file Array.hh
 * @author Michael Zillich
 * @date 2007
 * @version 0.1
 * @brief Own Array class.
 **/

#ifndef Z_ARRAY_HH
#define Z_ARRAY_HH

#include <stdarg.h>
#include <stdexcept>
#include <stdio.h>

namespace Z
{

/**
 * @brief My own array class.\n
 * STL vectors are just too complicated to debug and sorting is a nightmare,
 * Note: Although I loathe templates, I am using them here, hoping that they
 * won't make too many troubles later on...
 */
template <class Elem> class Array
{
private:
  static const unsigned DEFAULT_SIZE = 10;
 
  Elem *array;        ///< actual array
  unsigned size;      ///< used size of array
  unsigned capacity;  ///< allocated size of array

  void CheckIndex(unsigned i) const throw (std::runtime_error);
  void EnsureCapacity(unsigned need_size) throw (std::runtime_error);

public:
  Array(unsigned s = 0) throw (std::runtime_error);
  Array(const Array &a);
  ~Array();
  Array& operator=(const Array &a) {DeepCopy(a); return *this;}
  Elem& operator[](unsigned i);
  const Elem& operator[](unsigned i) const;
  bool Empty() {return size == 0;}
  Elem& First() throw (std::runtime_error);
  const Elem& First() const throw (std::runtime_error);
  Elem& Last() throw (std::runtime_error);
  const Elem& Last() const throw (std::runtime_error);
  void PushFront(const Elem &el);
  void PushBack(const Elem &el);
  void InsertBefore(unsigned i, const Elem &el);
  void InsertAfter(unsigned i, const Elem &el);
  void InsertSorted(const Elem &el, int(*compar)(const void *, const void *));
  unsigned Size() const {return size;}
  void Resize(unsigned new_size);
  void Sort(int(*compar)(const void *, const void *));
  bool Contains(const Elem &el) const;
  bool ContainsBackwards(const Elem &el) const;
  unsigned Find(const Elem &el);
  unsigned Find(unsigned start, const Elem &el);
  unsigned FindBackwards(const Elem &el);
  void Swap(unsigned i, unsigned j);
  void Set(const Elem &el);
  void Clear() {Resize(0);}
  void DeepCopy(const Array &a);
  unsigned CircularNext(unsigned i) const throw (std::runtime_error);
  unsigned CircularPrev(unsigned i) const throw (std::runtime_error);
  void Erase(unsigned i);
  void EraseFirst();
  void EraseLast();
  void Reverse();
  bool Intersect(const Array &a);
};

}

#include "Array.ic"

#endif


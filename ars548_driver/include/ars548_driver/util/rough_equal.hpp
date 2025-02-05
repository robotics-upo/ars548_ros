#pragma once

#include <limits>

template <class T>
bool rough_eq(T lhs, T rhs, T epsilon = std::numeric_limits<T>::epsilon()) // operator==
{ 
  return fabs(lhs - rhs) < epsilon;
}

#pragma once

#include <cmath>
#include <limits>

template <class T>
bool rough_eq(T lhs, T rhs, T epsilon = std::numeric_limits<T>::epsilon()) // operator==
{ 
  return std::fabs(lhs - rhs) < epsilon;
}

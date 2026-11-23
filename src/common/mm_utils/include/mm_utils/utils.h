#ifndef __MM_UTILS_H
#define __MM_UTILS_H

#include "mm_utils/ros2_utils.hpp"
#include <stdexcept>

#include <mm_utils/converters.h>
#include <mm_utils/geometry_utils.h>

namespace mm_utils
{

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(T value, const T2& low, const T2& high)
{
  if (!(low < high)) throw std::invalid_argument("lower bound must be below upper bound");
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(T value, const T2& limit)
{
  if (!(limit > 0)) throw std::invalid_argument("limit must be positive");
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  if (!(low < high)) throw std::invalid_argument("lower bound must be below upper bound");
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  if (!(limit > 0)) throw std::invalid_argument("limit must be positive");
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace mm_utils

#endif

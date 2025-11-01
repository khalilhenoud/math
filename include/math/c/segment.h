/**
 * @file segment.h
 * @author khalilhenoud@gmail.com
 * @brief 
 * @version 0.1
 * @date 2023-06-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef SEGMENT_DEFINITION_H
#define SEGMENT_DEFINITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math/c/vector3f.h>


typedef 
struct segment_t {
  point3f points[2];
} segment_t;

typedef segment_t line_t;

inline
point3f 
closest_point_on_segment(
  const point3f* point,
  const segment_t* target);

inline
point3f 
closest_point_on_segment_loose(
  const point3f* point, 
  const point3f* a,
  const point3f* b);

inline
float
get_point_distance_to_line(
  const point3f* point,
  const line_t* target);

#include "segment.inl"

#ifdef __cplusplus
}
#endif

#endif
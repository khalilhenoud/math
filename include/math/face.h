/**
 * @file face.h
 * @author khalilhenoud@gmail.com
 * @brief
 * @version 0.1
 * @date 2023-06-10
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef FACE_DEFINITION_H
#define FACE_DEFINITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math/vector3f.h>


typedef
struct face_t {
  point3f points[3];
} face_t;

// IMPORTANT: normals are assumed unitary in all these functions.
inline
face_t
get_extended_face(
  const face_t *face,
  float radius);

inline
void
get_faces_normals(
  const face_t *faces,
  const uint32_t count,
  vector3f *normals);

// NOTE: distance < 0 if the point is in the face's negative halfspace.
inline
float
get_point_distance(
  const face_t *face,
  const vector3f *normal,
  const point3f *point);

inline
point3f
get_point_projection(
  const face_t *face,
  const vector3f *normal,
  const point3f *point,
  float *distance);

#include "face.impl"

#ifdef __cplusplus
}
#endif

#endif
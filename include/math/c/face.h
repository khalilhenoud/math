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
#ifndef FACE_COLLISION
#define FACE_COLLISION

#ifdef __cplusplus
extern "C" {
#endif

#include <math/c/vector3f.h>


typedef struct sphere_t sphere_t;
typedef struct segment_t segment_t;
typedef struct capsule_t capsule_t;

typedef
struct face_t {
  point3f points[3];
} face_t;

// IMPORTANT: normals are assumed unitary in all these functions.
inline
face_t
get_extended_face(
  const face_t* face,
  float radius);

inline
void
get_faces_normals(
  const face_t* faces,
  const uint32_t count,
  vector3f* normals);

// NOTE: distance < 0 if the point is in the face's negative halfspace.
inline
float
get_point_distance(
  const face_t* face,
  const vector3f* normal,
  const point3f* point);

inline
point3f
get_point_projection(
  const face_t* face,
  const vector3f* normal,
  const point3f* point,
  float* distance);

typedef
enum segment_plane_classification_t {
  SEGMENT_PLANE_PARALLEL,
  SEGMENT_PLANE_COPLANAR,
  SEGMENT_PLANE_INTERSECT_ON_SEGMENT,
  SEGMENT_PLANE_INTERSECT_OFF_SEGMENT,
  SEGMENT_PLANE_COUNT
} segment_plane_classification_t;

inline
segment_plane_classification_t
classify_segment_face(
  const face_t *face, 
  const vector3f *normal, 
  const segment_t *segment,
  point3f *intersection,
  float *t);

typedef
enum coplanar_point_classification_t {
  COPLANAR_POINT_OUTSIDE,
  COPLANAR_POINT_ON_OR_INSIDE,
  COPLANAR_POINT_COUNT
} coplanar_point_classification_t;

// NOTE: The point is assumed coplanar, this function will not do any checks as
// these are prone to floating point imprecision error.
inline
coplanar_point_classification_t
classify_coplanar_point_face(
  const face_t* face,
  const vector3f* normal,
  const point3f* point,
  point3f* closest);

typedef
enum point_halfspace_classification_t {
  POINT_ON_PLANE,
  POINT_IN_POSITIVE_HALFSPACE,
  POINT_IN_NEGATIVE_HALFSPACE,
  POINT_COUNT
} point_halfspace_classification_t;

inline
point_halfspace_classification_t
classify_point_halfspace(
  const face_t* face,
  const vector3f* normal,
  const point3f* point);

typedef
enum sphere_face_classification_t {
  SPHERE_FACE_COLLIDES,
  SPHERE_FACE_COLLIDES_SPHERE_CENTER_ON_FACE,
  SPHERE_FACE_NO_COLLISION,
  SPHERE_FACE_COUNT
} sphere_face_classification_t;

// NOTE: as_plane affects how the penetration is calculated.
inline
sphere_face_classification_t
classify_sphere_face(
  const sphere_t* sphere,
  const face_t* face,
  const vector3f* normal,
  const int32_t as_plane,
  vector3f* penetration);

inline
float
get_sphere_face_distance(
  const sphere_t* sphere, 
  const face_t* face, 
  const vector3f* normal);

typedef
enum capsule_face_classification_t {
  CAPSULE_FACE_COLLIDES,
  CAPSULE_FACE_COLLIDES_CAPSULE_AXIS_INTERSECTS_FACE,
  CAPSULE_FACE_COLLIDES_CAPSULE_AXIS_COPLANAR_FACE,
  CAPSULE_FACE_NO_COLLISION,
  CAPSULE_FACE_COUNT
} capsule_face_classification_t;

inline
void
extrude_capsule_along_face_normal(
  const capsule_t* capsule,
  const face_t* face,
  const vector3f* normal,
  const segment_t* segment,
  const point3f* intersection,
  vector3f* penetration);

// NOTE: as_plane affects how the penetration is calculated.
inline
capsule_face_classification_t
classify_capsule_face(
  const capsule_t* capsule,
  const face_t* face,
  const vector3f* normal,
  const int32_t as_plane,
  vector3f* penetration,
  point3f* sphere_center);

inline
float
get_capsule_face_distance(
  const capsule_t* capsule, 
  const face_t* face, 
  const vector3f* normal);

// call this function to find the initial toi. the sphere must be in non-solid
// at t=0 and colliding at t=1. the function will return the toi between 0 and 1
inline
float
find_sphere_face_intersection_time(
  sphere_t sphere, 
  const face_t* face,
  const vector3f* normal,
  const vector3f displacement,
  const uint32_t max_iteration,
  const float limit);

// call this function to find the initial toi. the capsule must be in non-solid
// at t=0 and colliding at t=1. the function will return the toi between 0 and 1
inline
float
find_capsule_face_intersection_time(
  capsule_t capsule,
  const face_t* face, 
  const vector3f* normal,
  const vector3f displacement,
  const uint32_t max_iteration,
  const float limit);

#include "face.inl"

#ifdef __cplusplus
}
#endif

#endif
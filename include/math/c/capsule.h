/**
 * @file capsule.h
 * @author khalilhenoud@gmail.com
 * @brief
 * @version 0.1
 * @date 2023-06-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CAPSULE_DEFINITION_H
#define CAPSULE_DEFINITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math/c/vector3f.h>


typedef struct segment_t segment_t;

// direction is (0, 1, 0) in all cases, a transform is required to modify this.
// NOTE: make a version of these functions that take transform into account;
typedef
struct capsule_t {
  point3f center;
  float half_height;
  float radius;
} capsule_t;

// TODO: Provide a variant with transform.
inline
void
get_capsule_segment(
  const capsule_t *source,
  segment_t *segment);

// TODO: Provide a variant with transform.
inline
void
get_capsule_segment_loose(
  const capsule_t *source,
  point3f *a,
  point3f *b);

#include "capsule.impl"

#ifdef __cplusplus
}
#endif

#endif
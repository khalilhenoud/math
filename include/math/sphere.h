/**
 * @file sphere.h
 * @author khalilhenoud@gmail.com
 * @brief
 * @version 0.1
 * @date 2023-06-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef SPHERE_DEFINITION_H
#define SPHERE_DEFINITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math/vector3f.h>


typedef
struct sphere_t {
  point3f center;
  float radius;
} sphere_t;

#ifdef __cplusplus
}
#endif

#endif
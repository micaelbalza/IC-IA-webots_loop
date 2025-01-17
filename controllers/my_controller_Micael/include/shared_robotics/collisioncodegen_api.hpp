// Copyright 2019-2021 The MathWorks, Inc.
/**
 * @file
 * @brief This files serves as a wrapper of the
 * collisioncodegen_checkCollision_api.hpp. This file is ANSI-C compliant.
 */

#ifndef COLLISIONCODEGEN_API_HPP
#define COLLISIONCODEGEN_API_HPP
#include "collisioncodegen_util.hpp"

typedef void* CollisionGeometryPtr;

/**
 * @brief Create a collision geometry as a box primitive (the caller is responsible for freeing the
 * memory pointed by the returned pointer)
 * @param x The side length of the box along x-axis
 * @param y The side length of the box along y-axis
 * @param z The side length of the box along z-axis
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryPtr collisioncodegen_makeBox(double x, double y, double z);

/**
 * @brief Create a collision geometry as a sphere primitive (the caller is responsible for freeing
 * the memory pointed by the returned pointer)
 * @param r The radius of the sphere
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryPtr collisioncodegen_makeSphere(double r);

/**
 * @brief Create a collision geometry as a box primitive (the caller is responsible for freeing the
 * memory pointed by the returned pointer)
 * @param r The radius of the cylinder
 * @param h The length of the cylinder
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryPtr collisioncodegen_makeCylinder(double r, double h);

/**
 * @brief Create a full-mesh primitive with faces (the caller is responsible for freeing
 * the memory pointed by the returned pointer)
 * @param vertices The vertices of the mesh
 * @param numVertices The number of vertices of the mesh
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryPtr collisioncodegen_makeMesh(double *vertices,
                                                                              double numVertices);



/**
 * @brief Check collision between @p obj1 and @p obj2 if they are not in collision, also computes
 * the minimal distance and witness points if requested.
 * @param[in] obj1 Pointer to @c CollisionGeometry object 1
 * @param[in] obj2 Pointer to @c CollisionGeometry object 2
 * @param[in] computeDistance An integer indicating whether to compute minimal distance when
 * checking collision. 1 - yes, 0 - no
 * @param[out] p1Vec Pointer to witness point on @p obj1
 * @param[out] p2Vec Pointer to witness point on @p obj2
 * @param[out] distance Minimal distance between @p obj1 and @p obj2, if they are not in collision
 * @return The minimal distance between @p obj1 and @p obj2
 */
EXTERN_C COLLISIONCODEGEN_API int collisioncodegen_intersect(CollisionGeometryPtr obj1,
                              double* pos1, 
                              double* quat1,
                              CollisionGeometryPtr obj2,
                              double* pos2, 
                              double* quat2,
                              double computeDistance,
                              double* p1Vec,
                              double* p2Vec,
                              double* distance);

/**
 * @brief Delete the factory generated CollisionGeometryPtr
 * @param[in] objPtr Pointer to a pointer of CollisionGeometry object
 */
EXTERN_C COLLISIONCODEGEN_API void collisioncodegen_destructGeometry(const CollisionGeometryPtr* objPtr);

#endif

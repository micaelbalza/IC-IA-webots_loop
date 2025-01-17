// Copyright 2018-2019 The MathWorks, Inc.

/**
 * @file
 * @brief Provide additional typedefs needed for collisioncodegen
 */

#ifndef COLLISIONCODEGEN_UTIL_HPP
#define COLLISIONCODEGEN_UTIL_HPP

#if defined(BUILDING_LIBMWCOLLISIONCODEGEN) // should be defined by the mw build infrastructure
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"

#define COLLISIONCODEGEN_API DLL_EXPORT_SYM

#else

#ifndef COLLISIONCODEGEN_API
#define COLLISIONCODEGEN_API
#endif

#endif /* else */

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif
#endif

#endif /* COLLISIONCODEGEN_UTIL_HPP_ */

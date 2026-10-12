#ifndef NUMERIC_TYPES_H
#define NUMERIC_TYPES_H

#include <float.h>

typedef float float32_t;
typedef double float64_t;

_Static_assert( sizeof( float32_t ) == 4U, "float32_t must be 32 bits" );
_Static_assert( sizeof( float64_t ) == 8U, "float64_t must be 64 bits" );
_Static_assert( ( FLT_MANT_DIG == 24 ) && ( DBL_MANT_DIG == 53 ), "IEEE 754 single and double precision required" );

#endif

#ifndef INCLUDED_MATH_H
#define INCLUDED_MATH_H

#include "utils/vec.h"

// remove those 3
void normalize(float (&v)[3]);
void cross(float v1[3], float v2[3], float result[3]);
float dot(float (&v1)[3], float (&v2)[3]);

int glu_InvertMatrixf(const float m[16], float invOut[16]);

void glu_MakeIdentityf( float m[16] );
void glu_LookAt2(float eyex, double eyey, float eyez, float centerx,
          float centery, float centerz, float upx, float upy,
          float upz);

bool invert(const mat4& m, mat4 &inverse, float tolerance);
mat3 inverse(const mat3& m, float tolerance = 0);


vec3 matrix2euler(const mat4& m);

#endif // INCLUDED_MATH_H

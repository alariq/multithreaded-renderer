#include "utils/vec.h"
#include "utils/matrix.h"
#include <cmath>
#include <cassert>

void normalize(float (&v)[3])
{
    float l = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (l == 0.0) return;
    v[0] = v[0]/l;
    v[1] = v[1]/l;
    v[2] = v[2]/l;
}

void cross(float v1[3], float v2[3], float result[3])
{
    result[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result[2] = v1[0]*v2[1] - v1[1]*v2[0];
} 


float dot(float (&v1)[3], float (&v2)[3])
{
    return v1[0]*v2[0] +  v1[1]*v2[1] + v1[2]*v2[2];
}

int glu_InvertMatrixf(const float m[16], float invOut[16])
{
    float inv[16], det;
    int i;

    inv[0] =   m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15]
    + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4] =  -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15]
    - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8] =   m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15]
    + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14]
    - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1] =  -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15]
    - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5] =   m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15]
    + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9] =  -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15]
    - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] =  m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14]
    + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2] =   m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15]
    + m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6] =  -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15]
    - m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] =  m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15]
    + m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14]
    - m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3] =  -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11]
    - m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7] =   m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11]
    + m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11]
    - m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] =  m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10]
    + m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];

    det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (det == 0)
        return false;

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
} 

void glu_MakeIdentityf(float m[16])
{
    m[0+4*0] = 1; m[0+4*1] = 0; m[0+4*2] = 0; m[0+4*3] = 0;
    m[1+4*0] = 0; m[1+4*1] = 1; m[1+4*2] = 0; m[1+4*3] = 0;
    m[2+4*0] = 0; m[2+4*1] = 0; m[2+4*2] = 1; m[2+4*3] = 0;
    m[3+4*0] = 0; m[3+4*1] = 0; m[3+4*2] = 0; m[3+4*3] = 1;
} 

void glu_LookAt2(float eyex, float eyey, float eyez, float centerx,
          float centery, float centerz, float upx, float upy,
          float upz)
{
    float forward[3], side[3], up[3];
    float m[4][4];
    glu_MakeIdentityf(&m[0][0]);

    forward[0] = (float)(centerx - eyex);
    forward[1] = (float)(centery - eyey);
    forward[2] = (float)(centerz - eyez);

    up[0] = (float)upx;
    up[1] = (float)upy;
    up[2] = (float)upz;

    normalize(forward);

    /* Side = forward x up */
    cross(forward, up, side);
    normalize(side);

    /* Recompute up as: up = side x forward */
    cross(side, forward, up);

    m[0][0] = side[0];
    m[1][0] = side[1];
    m[2][0] = side[2];

    m[0][1] = up[0];
    m[1][1] = up[1];
    m[2][1] = up[2];

    m[0][2] = -forward[0];
    m[1][2] = -forward[1];
    m[2][2] = -forward[2];

    float eye[3] = { -(float)eyex, -(float)eyey, -(float)eyez};
    float forwneg[] = { -forward[0], -forward[1], -forward[2]};
    float tx = eye[0]*side[0] + eye[1]*up[0] + eye[2]*forwneg[0];
    float ty = eye[0]*side[1] + eye[1]*up[1] + eye[2]*forwneg[1];
    float tz = eye[0]*side[2] + eye[1]*up[2] + eye[2]*forwneg[2];

    m[0][3] = tx;
    m[1][3] = ty;
    m[2][3] = tz;

    float minv[16];

    glu_InvertMatrixf(&m[0][0], minv);
    

    //glMultMatrixf(&m[0][0]);
    //glTranslated(-eyex, -eyey, -eyez);
}

// adapted from openvdb

bool isApproxEqual(float a, float b, float tolerance) {
    return std::abs(a - b) < tolerance;
}

bool isExactlyEqual(float a, float b) {
    return a == b;
}

float det2(const mat4& a, int i0, int i1, int j0, int j1) {
        int i0row = i0;
        int i1row = i1;
        return a.elem[i0row][j0]*a.elem[i1row][j1] - a.elem[i0row][j1]*a.elem[i1row][j0];
}

float det3(const mat4 &a, int i0, int i1, int i2, int j0, int j1, int j2) {
	int i0row = i0;
	return a.elem[i0row][j0] * det2(a, i1, i2, j1, j2) +
		   a.elem[i0row][j1] * det2(a, i1, i2, j2, j0) +
		   a.elem[i0row][j2] * det2(a, i1, i2, j0, j1);
}

bool invert(const mat4& m, mat4 &inverse, float tolerance);

bool inverse(const mat4& m, mat4& inv, float tolerance = 0) {
        //
        // inv [ A  | b ]  =  [ E  | f ]    A: 3x3, b: 3x1, c': 1x3 d: 1x1
        //     [ c' | d ]     [ g' | h ]
        //
        // If A is invertible use
        //
        //   E  = A^-1 + p*h*r
        //   p  = A^-1 * b
        //   f  = -p * h
        //   g' = -h * c'
        //   h  = 1 / (d - c'*p)
        //   r' = c'*A^-1
        //
        // Otherwise use gauss-jordan elimination
        //

        float m0011 = m[0][0] * m[1][1];
        float m0012 = m[0][0] * m[1][2];
        float m0110 = m[0][1] * m[1][0];
        float m0210 = m[0][2] * m[1][0];
        float m0120 = m[0][1] * m[2][0];
        float m0220 = m[0][2] * m[2][0];

        float detA = m0011 * m[2][2] - m0012 * m[2][1] - m0110 * m[2][2]
               + m0210 * m[2][1] + m0120 * m[1][2] - m0220 * m[1][1];

        bool hasPerspective =
                (!isExactlyEqual(m[0][3], float(0.0)) ||
                 !isExactlyEqual(m[1][3], float(0.0)) ||
                 !isExactlyEqual(m[2][3], float(0.0)) ||
                 !isExactlyEqual(m[3][3], float(1.0)));

        float det;
        if (hasPerspective) {
            det = m[0][3] * det3(m, 1,2,3, 0,2,1)
                + m[1][3] * det3(m, 2,0,3, 0,2,1)
                + m[2][3] * det3(m, 3,0,1, 0,2,1)
                + m[3][3] * detA;
        } else {
            det = detA * m[3][3];
        }

        bool invertible;

        if (isApproxEqual(det,float(0.0),tolerance)) {
            invertible = false;

        } else if (isApproxEqual(detA,float(0.0),float(1e-8))) {
            // det is too small to rely on inversion by subblocks
            invertible = invert(m, inv, tolerance);

        } else {
            invertible = true;
            detA = 1.0f / detA;

            //
            // Calculate A^-1
            //
            inv[0][0] = detA * ( m[1][1] * m[2][2] - m[1][2] * m[2][1]);
            inv[0][1] = detA * (-m[0][1] * m[2][2] + m[0][2] * m[2][1]);
            inv[0][2] = detA * ( m[0][1] * m[1][2] - m[0][2] * m[1][1]);

            inv[1][0] = detA * (-m[1][0] * m[2][2] + m[1][2] * m[2][0]);
            inv[1][1] = detA * ( m[0][0] * m[2][2] - m0220);
            inv[1][2] = detA * ( m0210   - m0012);

            inv[2][0] = detA * ( m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            inv[2][1] = detA * ( m0120 - m[0][0] * m[2][1]);
            inv[2][2] = detA * ( m0011 - m0110);

            if (hasPerspective) {
                //
                // Calculate r, p, and h
                //
                vec3 r;
                r[0] = m[3][0] * inv[0][0] + m[3][1] * inv[1][0]
                     + m[3][2] * inv[2][0];
                r[1] = m[3][0] * inv[0][1] + m[3][1] * inv[1][1]
                     + m[3][2] * inv[2][1];
                r[2] = m[3][0] * inv[0][2] + m[3][1] * inv[1][2]
                     + m[3][2] * inv[2][2];

                vec3 p;
                p[0] = inv[0][0] * m[0][3] + inv[0][1] * m[1][3]
                     + inv[0][2] * m[2][3];
                p[1] = inv[1][0] * m[0][3] + inv[1][1] * m[1][3]
                     + inv[1][2] * m[2][3];
                p[2] = inv[2][0] * m[0][3] + inv[2][1] * m[1][3]
                     + inv[2][2] * m[2][3];

                float h = m[3][3] - dot(p, vec3(m[3][0],m[3][1],m[3][2]));
                if (isApproxEqual(h,float(0.0),tolerance)) {
                    invertible = false;

                } else {
                    h = 1.0 / h;

                    //
                    // Calculate h, g, and f
                    //
                    inv[3][3] = h;
                    inv[3][0] = -h * r[0];
                    inv[3][1] = -h * r[1];
                    inv[3][2] = -h * r[2];

                    inv[0][3] = -h * p[0];
                    inv[1][3] = -h * p[1];
                    inv[2][3] = -h * p[2];

                    //
                    // Calculate E
                    //
                    p *= h;
                    inv[0][0] += p[0] * r[0];
                    inv[0][1] += p[0] * r[1];
                    inv[0][2] += p[0] * r[2];
                    inv[1][0] += p[1] * r[0];
                    inv[1][1] += p[1] * r[1];
                    inv[1][2] += p[1] * r[2];
                    inv[2][0] += p[2] * r[0];
                    inv[2][1] += p[2] * r[1];
                    inv[2][2] += p[2] * r[2];
                }
            } else {
                // Equations are much simpler in the non-perspective case
                inv[3][0] = - (m[3][0] * inv[0][0] + m[3][1] * inv[1][0]
                                + m[3][2] * inv[2][0]);
                inv[3][1] = - (m[3][0] * inv[0][1] + m[3][1] * inv[1][1]
                                + m[3][2] * inv[2][1]);
                inv[3][2] = - (m[3][0] * inv[0][2] + m[3][1] * inv[1][2]
                                + m[3][2] * inv[2][2]);
                inv[0][3] = 0.0;
                inv[1][3] = 0.0;
                inv[2][3] = 0.0;
                inv[3][3] = 1.0;
            }
        }

        assert(invertible && "Inversion of singular 4x4 matrix");
        return inv;
    }

    float det(const mat3& m) 
    {
        const float* mm = m;
        const float co00 = mm[4]*mm[8] - mm[5]*mm[7];
        const float co10 = mm[5]*mm[6] - mm[3]*mm[8];
        const float co20 = mm[3]*mm[7] - mm[4]*mm[6];
        return mm[0]*co00  + mm[1]*co10 + mm[2]*co20;
    }

    /// Determinant of matrix
    float det(const mat4& m)
    {
        const float *ap = m;
        mat3 submat;
        float *sp = (float*)submat;

        float determinant = 0;
        int sign = 1;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    if ((k != i) && (j != 0)) {
                        *sp++ = *ap;
                    }
                    ap++;
                }
            }

            determinant += (float)sign * m[0][i] * det(submat);
            sign = -sign;
        }

        return determinant;
    }

void swap(float a, float b) {
    float t = a;
    a = b;
    b = t;
}

/// Invert via gauss-jordan elimination. Modified from dreamworks internal mx library
bool invert(const mat4& m, mat4 &inverse, float tolerance) {
    mat4 temp = m;
    inverse = mat4::identity();

    // Forward elimination step
    double det = 1.0;
    for (int i = 0; i < 4; ++i) {
        int row = i;
        double max = fabs(temp[i][i]);

        for (int k = i+1; k < 4; ++k) {
            if (fabs(temp[k][i]) > max) {
                row = k;
                max = fabs(temp[k][i]);
            }
        }

        if (isExactlyEqual(max, 0.0)) return false;

        // must move pivot to row i
        if (row != i) {
            det = -det;
            for (int k = 0; k < 4; ++k) {
                swap(temp[row][k], temp[i][k]);
                swap(inverse[row][k], inverse[i][k]);
            }
        }

        double pivot = temp[i][i];
        det *= pivot;

        // scale row i
        for (int k = 0; k < 4; ++k) {
            temp[i][k] /= pivot;
            inverse[i][k] /= pivot;
        }

        // eliminate in rows below i
        for (int j = i+1; j < 4; ++j) {
            double t = temp[j][i];
            if (!isExactlyEqual(t, 0.0)) {
                // subtract scaled row i from row j
                for (int k = 0; k < 4; ++k) {
                    temp[j][k] -= temp[i][k] * t;
                    inverse[j][k] -= inverse[i][k] * t;
                }
            }
        }
    }

    // Backward elimination step
    for (int i = 3; i > 0; --i) {
        for (int j = 0; j < i; ++j) {
            double t = temp[j][i];

            if (!isExactlyEqual(t, 0.0)) {
                for (int k = 0; k < 4; ++k) {
                    inverse[j][k] -= inverse[i][k]*t;
                }
            }
        }
    }
    return det*det >= tolerance*tolerance;
}

// Return the adjoint of this matrix, i.e., the transpose of its cofactor.
mat3 adjoint(const mat3& m) {
	return mat3(m.e[4] * m.e[8] - m.e[5] * m.e[7], m.e[2] * m.e[7] - m.e[1] * m.e[8],
				m.e[1] * m.e[5] - m.e[2] * m.e[4], m.e[5] * m.e[6] - m.e[3] * m.e[8],
				m.e[0] * m.e[8] - m.e[2] * m.e[6], m.e[2] * m.e[3] - m.e[0] * m.e[5],
				m.e[3] * m.e[7] - m.e[4] * m.e[6], m.e[1] * m.e[6] - m.e[0] * m.e[7],
				m.e[0] * m.e[4] - m.e[1] * m.e[3]);
}

mat3 inverse(const mat3& m, float tolerance /*= 0*/) {
	mat3 inv(adjoint(m));

	const float det = inv.e[0] * m.e[0] + inv.e[1] * m.e[3] + inv.e[2] * m.e[6];

	// If the determinant is 0, m was singular and the result will be invalid.
	if (isApproxEqual(det, 0.0f, tolerance)) {
		assert(0 && "Inversion of singular 3x3 matrix");
	}
	return inv * (1.0f / det);
}

bool eps_eq(float a, float b, float eps) {
    return fabsf(a-b) <= eps;
}

// http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
vec3 matrix2euler(const mat4& m) {
	const float m20 = m[2][0];
	const float π = M_PI;
	float θ, ψ, φ;
    // m20!=-1 && m20 != 1
	if (fabsf(fabsf(m20) - 1) > 1e-9f) {
		float θ1 = -asin(m20);
		//float θ2 = π − θ1;
		float ψ1 = atan2(m[2][1] * cos(θ1), m[2][2] * cos(θ1));
		//float ψ2 = atan2(m[2][1] * cos(θ2), m[2][2] * cos(θ2));
		float φ1 = atan2(m[1][0] * cos(θ1), m[0][0] * cos(θ1));
		//float φ2 = atan2(m[1][0] * cos(θ2), m[1][1] * cos(θ2));
        θ = θ1;
		ψ = ψ1;
		φ = φ1;
	} else {
		φ = 0.0f; // anything; can set to 0
		if (eps_eq(m20, -1.0f, 1e-9f)) {
			θ = π / 2;
			ψ = φ + atan2(m[0][1], m[0][2]);
		} else {
			θ = -π / 2;
			ψ = -φ + atan2(-m[0][1], -m[0][2]);
		}
	}
    return vec3(ψ, θ, φ);
}


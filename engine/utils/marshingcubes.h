#pragma once

#include <immintrin.h> 
#include "utils/vec.h"

extern const int edgeTable[256];
extern const int triTable[256][16];

#if defined(PLATFORM_WINDOWS)
    #define GCC_ALIGN(x)
    #define MS_ALIGN(x) __declspec(align(x))
#else
    #define GCC_ALIGN(x) __attribute__((aligned(x)))
    #define MS_ALIGN(x)
#endif

typedef MS_ALIGN(16) vec4 Vector4A16 GCC_ALIGN(16);
typedef MS_ALIGN(16) vec4 *PVector4A16 GCC_ALIGN(16);

#ifdef USE_SSE
typedef __m128 aVector;
#else
typedef vec4 aVector;
#endif

typedef struct {
	aVector pos;
	aVector norm;
	float value;
} VERTICE;

typedef struct {
	aVector* pos;
	aVector* norm;
	float* value;
} VERTICE_SOA;

typedef struct {
	aVector pos;
	aVector norm;
} DRAW_VERTICE;

typedef struct {
	DRAW_VERTICE p[3];
} TRIANGLE;

typedef struct {
   VERTICE* p[8];
} GRIDCELL;

typedef struct {
   aVector* pos[8];
   aVector* norm[8];
   float* value[8];
} GRIDCELL_SOA;

#define FORCE_INLINE inline

#ifdef USE_SSE
FORCE_INLINE void vec_set(aVector& v, float* farr)
{
	v = _mm_set_ps(farr[3],farr[2],farr[1],farr[0]);
}
FORCE_INLINE aVector vec_add(const aVector& v1, const aVector& v2)
{
	return _mm_add_ps(v1,v2);
}
FORCE_INLINE void vec_add_eq(aVector& v1, const aVector& v2)
{
	v1 = _mm_add_ps(v1,v2);
}

FORCE_INLINE aVector vec_sub(const aVector& v1, const aVector& v2)
{
	return _mm_sub_ps(v1,v2);
}

FORCE_INLINE aVector vec_mul(const aVector& v1, const float f)
{
	aVector t = _mm_set_ps(f,f,f,f);
	return _mm_mul_ps(v1,t);
}

FORCE_INLINE float vec_dot(const aVector& v1, const aVector& v2)
{
	aVector t = _mm_mul_ps(v1,v2);
	return t.m128_f32[0] + t.m128_f32[1] + t.m128_f32[2] + t.m128_f32[3];
}

#else
FORCE_INLINE void vec_set(aVector& v, float* farr)
{
	v.x = farr[0];
	v.y = farr[1];
	v.z = farr[2];
	v.w = farr[3];
}
FORCE_INLINE void vec_set3(aVector& v, float* farr)
{
	v.x = farr[0];
	v.y = farr[1];
	v.z = farr[2];
}
FORCE_INLINE aVector vec_add(const aVector& v1, const aVector& v2)
{
	return v1 + v2;
}
FORCE_INLINE void vec_add_eq(aVector& v1, const aVector& v2)
{
	v1 += v2;
}

FORCE_INLINE aVector vec_sub(const aVector& v1, const aVector& v2)
{
	return v1 - v2;
}

FORCE_INLINE aVector vec_mul(const aVector& v1, const float f)
{
	return v1*f;
}
FORCE_INLINE float vec_dot(const aVector& v1, const aVector& v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z + v1.w*v2.w;
}
FORCE_INLINE float vec_dot3(const aVector& v1, const aVector& v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
#endif

/*
   Given a grid cell and an isolevel, calculate the triangular
   facets required to represent the isosurface through the cell.
   Return the number of triangular facets, the array "triangles"
   will be loaded up with the vertices at most 5 triangular facets.
	0 will be returned if the grid cell is either totally above
   of totally below the isolevel.
*/
int Polygonise(const GRIDCELL* __restrict grid, float isolevel,TRIANGLE * __restrict triangles, int numCubes);
int Polygonise2(const GRIDCELL_SOA& grid, float isolevel,TRIANGLE *triangles);

/*
   Linearly interpolate the position where an isosurface cuts
   an edge between two vertices, each with their own scalar value
*/
VERTICE VertexInterp(float isolevel, const VERTICE* p1, const VERTICE* p2);
VERTICE VertexInterp2(float isolevel, const GRIDCELL_SOA& grid, const int ip1, const int ip2);

template<typename T>
T VertexInterp3(float isolevel, const T& p1, const T& p2)
{
	float mu;
	T p;

	float p1val = p1.value;
	float p2val = p2.value;

	if (fabs(isolevel-p1val) < 0.00001f)
		return p1;
	if (fabs(isolevel-p2val) < 0.00001f)
		return p2;
	if (fabs(p1val-p2val) < 0.00001f)
		return p1;
	mu = (isolevel - p1val) / (p2val - p1val);

	p.pos = vec_sub(p2.pos, p1.pos);
	p.pos = vec_mul(p.pos, mu);
	p.pos = vec_add(p1.pos, p.pos); 

	p.norm = vec_sub(p2.norm, p1.norm);
	p.norm = vec_mul(p.norm, mu);
	p.norm = vec_add(p1.norm, p.norm); 

	return(p);
}

// returns number of generated vertices, 3 consecutve vertices form a triangle
template<typename Grid, typename MeshBuffer>
int PolygoniseGrid(const Grid& grid, float isolevel,
			   MeshBuffer& mb, int numCubes) {
    int cur_vertex = 0;

	for (int a = 0; a < numCubes; a++) {
		const typename Grid::Cell& cell = grid[a];
		typename Grid::Vertex vertlist[12];

		/*
		  Determine the index into the edge table which
		  tells us which vertices are inside of the surface
	   */
		int cubeindex = 0;
		if (cell.p(0).value < isolevel) cubeindex |= 1;
		if (cell.p(1).value < isolevel) cubeindex |= 2;
		if (cell.p(2).value < isolevel) cubeindex |= 4;
		if (cell.p(3).value < isolevel) cubeindex |= 8;

		if (cell.p(4).value < isolevel) cubeindex |= 16;
		if (cell.p(5).value < isolevel) cubeindex |= 32;
		if (cell.p(6).value < isolevel) cubeindex |= 64;
		if (cell.p(7).value < isolevel) cubeindex |= 128;

		/* Cube is entirely in/out of the surface */
		if (edgeTable[cubeindex] == 0)
			continue;

		/* Find the vertices where the surface intersects the cube */
		if (edgeTable[cubeindex] & 1)
			vertlist[0] = VertexInterp3(isolevel, cell.p(0), cell.p(1));
		if (edgeTable[cubeindex] & 2)
			vertlist[1] = VertexInterp3(isolevel, cell.p(1), cell.p(2));
		if (edgeTable[cubeindex] & 4)
			vertlist[2] = VertexInterp3(isolevel, cell.p(2), cell.p(3));
		if (edgeTable[cubeindex] & 8)
			vertlist[3] = VertexInterp3(isolevel, cell.p(3), cell.p(0));
		if (edgeTable[cubeindex] & 16)
			vertlist[4] = VertexInterp3(isolevel, cell.p(4), cell.p(5));
		if (edgeTable[cubeindex] & 32)
			vertlist[5] = VertexInterp3(isolevel, cell.p(5), cell.p(6));
		if (edgeTable[cubeindex] & 64)
			vertlist[6] = VertexInterp3(isolevel, cell.p(6), cell.p(7));
		if (edgeTable[cubeindex] & 128)
			vertlist[7] = VertexInterp3(isolevel, cell.p(7), cell.p(4));
		if (edgeTable[cubeindex] & 256)
			vertlist[8] = VertexInterp3(isolevel, cell.p(0), cell.p(4));
		if (edgeTable[cubeindex] & 512)
			vertlist[9] = VertexInterp3(isolevel, cell.p(1), cell.p(5));
		if (edgeTable[cubeindex] & 1024)
			vertlist[10] = VertexInterp3(isolevel, cell.p(2), cell.p(6));
		if (edgeTable[cubeindex] & 2048)
			vertlist[11] = VertexInterp3(isolevel, cell.p(3), cell.p(7));

		/* Create the triangle */
		for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {

            mb.allocate_vb(3);

    		const typename Grid::Vertex& v0 = vertlist[triTable[cubeindex][i]];
			mb.p(cur_vertex, v0.pos);
			mb.n(cur_vertex++, v0.norm);

			const typename Grid::Vertex& v1 = vertlist[triTable[cubeindex][i + 1]];
			mb.p(cur_vertex, v1.pos);
			mb.n(cur_vertex++, v1.norm);

			const typename Grid::Vertex& v2 = vertlist[triTable[cubeindex][i + 2]];
			mb.p(cur_vertex, v2.pos);
			mb.n(cur_vertex++, v2.norm);
		}
	}
	return cur_vertex;
}


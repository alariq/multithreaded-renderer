//#version 420
//#define SLUGGY 1

in vec2 Texcoord;
in vec4 Color;

flat in vec4 glyphBandScale;
flat in uvec4 bandMaxTexCoords;
out vec4 FragColor;

uniform vec4 Foreground;
uniform sampler2DRect curvesTex;
uniform usampler2DRect bandsTex;

#define curveTex curvesTex
#define bandTex bandsTex

const float epsilon = 0.0001;
const float kQuadraticEpsilon = 0.0001;


#define glyphScale     glyphBandScale.xy
#define bandScale      glyphBandScale.zw
#define bandMax        bandMaxTexCoords.xy
#define bandsTexCoords bandMaxTexCoords.zw

uint CalcRootCode(float y1, float y2, float y3)
{
	// Calculate the root eligibility code for a sample-relative quadratic Bézier curve.
	// Extract the signs of the y coordinates of the three control points.

	uint i1 = floatBitsToUint(y1) >> 31U;
	uint i2 = floatBitsToUint(y2) >> 30U;
	uint i3 = floatBitsToUint(y3) >> 29U;

	uint shift = (i2 & 2U) | (i1 & ~2U);
	shift = (i3 & 4U) | (shift & ~4U);

	// Eligibility is returned in bits 0 and 8.

	return ((0x2E74U >> shift) & 0x0101U);
}

vec2 SolvePoly(vec2 p1, vec2 p2, vec2 p3, float pixelsPerEm, vec2 cov_wgt) {
	
	//uint code = (0x2E74U >> (((p1.y > 0.0) ? 2U : 0U) + ((p2.y > 0.0) ? 4U : 0U) + ((p3.y > 0.0) ? 8U : 0U))) & 3U;
    uint code = CalcRootCode(p1.y, p2.y, p3.y);
	if(code == 0U) {
		return cov_wgt;
	}

	float coverage = 0.0;
	float cov = cov_wgt.x;
	float wgt = cov_wgt.y;

	vec2 a = p1 - p2 * 2.0 + p3;
	vec2 b = p1 - p2;
	float ra = 1.0 / a.y;
	float rb = 0.5 / b.y;

	float d = sqrt(max(b.y * b.y - a.y * p1.y, 0.0));
	float t1 = (b.y - d) * ra;
	float t2 = (b.y + d) * ra;

    // using 1/65536 causing artifacts
	//if (abs(a.y) < 1.0 / 65536.0) t1 = t2 = p1.y * rb;
	if (abs(a.y) < 10*kQuadraticEpsilon) t1 = t2 = p1.y * rb;

	float x1 = (a.x * t1 - b.x * 2.0) * t1 + p1.x;
	float x2 = (a.x * t2 - b.x * 2.0) * t2 + p1.x;
	
	if ((code & 1U) != 0U) {
		cov += clamp(x1 * pixelsPerEm + 0.5, 0.0, 1.0);
		wgt = max(wgt, clamp(1.0 - abs(x1) * 2.0, 0.0, 1.0));
	}
	if (code > 1U) { 
		cov -= clamp(x2 * pixelsPerEm + 0.5, 0.0, 1.0);
		wgt = max(wgt, clamp(1.0 - abs(x2) * 2.0, 0.0, 1.0));
	}

	return vec2(cov, wgt);
}

float CalcCoverage(float xcov, float ycov, float xwgt, float ywgt, int flags)
{
	// Combine coverages from the horizontal and vertical rays using their weights.
	// Absolute values ensure that either winding direction convention works.
	float coverage = max(abs(xcov * xwgt + ycov * ywgt) / max(xwgt + ywgt, 1.0 / 65536.0), min(abs(xcov), abs(ycov)));

	// Using nonzero fill rule here.
	coverage = clamp(coverage, 0, 1);
	// If SLUG_WEIGHT is defined during compilation, then take a square root to boost optical weight.
	#if defined(SLUG_WEIGHT)
		coverage = sqrt(coverage);
	#endif
	return (coverage);
}

#define glyphParam bandMaxTexCoords.zwxy
#define bandParam vec4(0, 0, glyphBandScale.zw)

void main()
{
	float red = 0;
	vec2 pixelsPerEm = vec2(1.0 / fwidth(Texcoord.x), 1.0 / fwidth(Texcoord.y));

	uvec2 glyphLoc = glyphParam.xy & 0x0FFFU;
	//uvec2 bandMax = glyphParam.zw;// >> 12U;

	// Determine what bands the current pixel lies in by applying a scale
	// and offset to the texture coordinates. The scale given by bandParam.z
	// is the same for both directions, but there are different offsets given
	// by bandParam.xy. Band indexes are clamped to [0, bandMax.xy].
	uvec2 bandIndex = uvec2(clamp(ivec2(Texcoord * bandParam.zw + bandParam.xy), ivec2(0U, 0U), ivec2(bandMax)));

    vec2 cov_wgt_x = vec2(0,0);
    {
	const uint off = glyphLoc.y * 4096U + glyphLoc.x + bandIndex.y;
	const uvec2 hbandData = texelFetch(bandTex, ivec2(off& 0xFFFU, off>> 12U)).xy;
	uint numCurves = hbandData.x;
	uint curveStart = hbandData.y;

	for(uint curve = 0U; curve < numCurves; ++curve)
	{
		uint curveOffset = curveStart + curve;
		ivec2 curveLoc = ivec2(texelFetch(bandTex,   ivec2(curveOffset & 0xFFFU, curveOffset >> 12U)).xy);

		vec4 p12 = texelFetch(curvesTex, curveLoc) - vec4(Texcoord, Texcoord);
		vec2 p3 = texelFetch(curvesTex, ivec2(curveLoc.x + 1, curveLoc.y)).xy - Texcoord;
		
		if(curveLoc.x >=4095) red = 1;

		if (max(max(p12.x, p12.z), p3.x) * pixelsPerEm.x < -0.5) break;

		cov_wgt_x = SolvePoly(p12.xy, p12.zw, p3, pixelsPerEm.x, cov_wgt_x);
	}
    }

	vec2 cov_wgt_y = vec2(0,0);
    {
	const uint off = glyphLoc.y * 4096U + glyphLoc.x + bandMax.y + 1U + bandIndex.x;
	const uvec2 vbandData = texelFetch(bandTex, ivec2(off& 0xFFFU, off>> 12U)).xy;
	uint numCurves = vbandData.x;
	uint curveStart = vbandData.y;

	for(uint curve = 0U; curve < numCurves; ++curve)
	{
		uint curveOffset = curveStart + curve;
		ivec2 curveLoc = ivec2(texelFetch(bandsTex, ivec2(curveOffset & 0xFFFU, curveOffset >> 12U)).xy);

		vec4 p12 = texelFetch(curvesTex, curveLoc)- vec4(Texcoord, Texcoord);
		vec2 p3 = texelFetch(curvesTex, ivec2(curveLoc.x + 1, curveLoc.y)).xy - Texcoord;

		if(curveLoc.x >= 4095) red = 1;


		if (max(max(p12.y, p12.w), p3.y) * pixelsPerEm.y < -0.5) break;

		cov_wgt_y = SolvePoly(p12.yx, p12.wz, p3.yx, pixelsPerEm.y, cov_wgt_y);
	}
    }

	float coverageX = min(abs(cov_wgt_x.x), 1.0);
	float coverageY = min(abs(cov_wgt_y.x), 1.0);
    float coverage = 0;
	//coverage = (coverageX + coverageY) * 0.5;
	coverage = CalcCoverage(cov_wgt_x.x, cov_wgt_y.x, cov_wgt_x.y, cov_wgt_y.y, /*glyphData.w*/0);

	//FragColor = vec4(float(bandIndex.x)/bandScale.x, float(bandIndex.y)/bandScale.y, 0, 1);
	FragColor = red>0 ? vec4(1,0,0,1) : Color * Foreground * vec4(1,1,1,coverage);
}

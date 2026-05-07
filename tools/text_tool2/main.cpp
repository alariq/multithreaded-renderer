#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <SDL3_ttf/SDL_ttf.h>

#include "gos_font.h"
#include <stdio.h>

// errno + strerror
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm> // stable sort

#define USE_STBI
#if defined(USE_STBI)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#endif

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include "utils/logging.h"
#include "utils/myarray.h"

#include <GL/glew.h>

template<typename T>
static T Min(T a, T b) { return a < b ? a : b; }

template<typename T>
static T Min(T a, T b, T c) { return Min(a, Min(b, c)); }

template<typename T>
static T Max(T a, T b) { return a > b ? a : b; }

template<typename T>
static T Max(T a, T b, T c) { return Max(a, Max(b, c)); }

template<typename T>
static T Clamp(T x, T a, T b) { return Min(Max(x, a), b); }

struct Curve {
    float x1, y1, x2, y2, x3, y3;
    uint32_t texelIndex; // indexes the curves texture
    bool first; // first curve in a shape
};
BufferT<Curve, int> g_curves;
static BufferT<SlugCodePoint, int> g_codePoints;
static BufferT<uint16_t, int> g_bandsTextureBandOffsets; // GL_RG16 [curve_count band_offset]
static BufferT<uint16_t, int> g_bandsTextureCurveOffsets; // GL_RG16 [curve_offset curve_offset]
static BufferT<float, int> g_curvesTexture; // GL_RGBA32F [x1 y1 x2 y2]
static uint32_t g_bandCount = 8;
struct GlobalFontData {
    int lineSpacing;
    int unitsPerEm;
} g_globalFontData;


void usage(char** argv) {
    printf("%s <font_name.ttf> <out_font_name>\n", argv[0]);
}

static bool ProcessCodePoint(const stbtt_fontinfo* font, int codePoint, bool* b_was_ignored)
{
	const int glyphIdx = stbtt_FindGlyphIndex(font, codePoint);

	SlugCodePoint cp = {0};
    cp.codePoint = codePoint;

    stbtt_GetGlyphHMetrics(font, glyphIdx, &cp.advance, &cp.bearingX);

	// get the glyph's visible data bounding box
	int igx1=0, igy1=0, igx2=0, igy2=0;
	stbtt_GetGlyphBox(font, glyphIdx, &igx1, &igy1, &igx2, &igy2);
	const float gx1 = (float)igx1;
	const float gy1 = (float)igy1;

	cp.width = (uint32_t)(igx2 - igx1);
	cp.height = (uint32_t)(igy2 - igy1);

    cp.minX = igx1;
    cp.maxX = igx2;
    cp.minY = igy1;
    cp.maxY = igy2;

    bool b_supported = true;

	stbtt_vertex* vertices;
	const int vertexCount = stbtt_GetGlyphShape(font, glyphIdx, &vertices);
	if(vertexCount == 0) {
		log_warning("U+%04X has no vertices\n", (unsigned int)codePoint);
        b_supported = false;
	}

	// we don't support cubic Bézier curves
	for(int v = 0; v < vertexCount; ++v) {
		if(vertices[v].type == STBTT_vcubic) {
			log_warning("U+%04X has bicubic curves\n", (unsigned int)codePoint);
            b_supported = false;
		}
	}

    // do not process curves for this codePoint (e.g. non-printed char like spacebar)
    if(!b_supported) {
        g_codePoints.push(cp);
        return false;
    }

	// build temporary curve list
	Curve curve = { 0 };
	curve.first = false;
	g_curves.reset();
	for(int v = 0; v < vertexCount; ++v)
	{
		const stbtt_vertex& vert = vertices[v];
		if(vert.type == STBTT_vcurve)
		{
			curve.x1 = curve.x3;
			curve.y1 = curve.y3;
			curve.x2 = (float)vert.cx - gx1;
			curve.y2 = (float)vert.cy - gy1;
			curve.x3 = (float)vert.x - gx1;
			curve.y3 = (float)vert.y - gy1;
			g_curves.push(curve);
			curve.first = false;
		}
		else if(vert.type == STBTT_vline)
		{
			curve.x1 = curve.x3;
			curve.y1 = curve.y3;
			curve.x3 = (float)vert.x - gx1;
			curve.y3 = (float)vert.y - gy1;
            // as suggested by https://github.com/EricLengyel/Slug/tree/main (Tips & Tricks)
			curve.x2 = curve.x3;// floorf((curve.x1 + curve.x3) / 2.0f);
			curve.y2 = curve.y3;//floorf((curve.y1 + curve.y3) / 2.0f);
			g_curves.push(curve);
			curve.first = false;
		}
		else if(vert.type == STBTT_vmove)
		{
			curve.first = true;
			curve.x3 = (float)vert.x - gx1;
			curve.y3 = (float)vert.y - gy1;
		}
	}

	const float fbandDelta = 0.0f;
	const int bandsTexelIndex = g_bandsTextureBandOffsets.size() / 2;

	// fix up curves where the control point is one of the endpoints
	for(int i=0; i< g_curves.size(); ++i)
	{
        Curve& c = g_curves[i];
		if((c.x2 == c.x1 && c.y2 == c.y1) ||
		   (c.x2 == c.x3 && c.y2 == c.y3))
		{
			c.x2 = (c.x1 + c.x3) / 2.0f;
			c.y2 = (c.y1 + c.y3) / 2.0f;
		}
	}

	// write curves texture

	for(int i=0; i< g_curves.size(); ++i)
	{
        Curve& c = g_curves[i];

		// make sure we start a curve at a texel's boundary
		if(c.first && g_curvesTexture.size() % 4 != 0)
		{
			int toAdd = 4 - (g_curvesTexture.size() % 4);
            while(toAdd--) {
				g_curvesTexture.push(-1.0f);
			}
		}

		// make sure a curve doesn't cross a row boundary
		const bool newRow = (g_curvesTexture.size() / 4) % SLUG_TEXTURE_WIDTH == SLUG_TEXTURE_WIDTH - 1;
		if(newRow)
		{
			size_t toAdd = 8 - (g_curvesTexture.size() % 4);
            while(toAdd--) {
				g_curvesTexture.push(-1.0f);
			}
		}

		// [A1 B1] [C1=A2 B2] [C2=A3 B3] ...
		if(c.first || newRow)
		{
			c.texelIndex = (uint32_t)g_curvesTexture.size() / 4;
			assert(g_curvesTexture.size() % 4 == 0);
			g_curvesTexture.push(c.x1);
			g_curvesTexture.push(c.y1);
		}
		else
		{
			c.texelIndex = (((uint32_t)g_curvesTexture.size() / 2) - 1) / 2;
		}
		
		assert(g_curvesTexture.size() % 2 == 0);
		g_curvesTexture.push(c.x2);
		g_curvesTexture.push(c.y2);
		g_curvesTexture.push(c.x3);
		g_curvesTexture.push(c.y3);
	}

	const uint32_t dimX = 1 + (uint32_t)(igx2 - igx1);
	const uint32_t dimY = 1 + (uint32_t)(igy2 - igy1);
	uint32_t bandCount = g_bandCount;
    // what if dims are more than 2 times less?
	if(dimX < bandCount || dimY < bandCount)
	{
		bandCount = Min(dimX, dimY) / 2;
	}

	// horizontal bands
	
	const uint32_t bandDimY = (dimY + bandCount - 1) / bandCount;
	const float fbandDimY = (float)bandDimY;
	float bandMinY = -fbandDelta;
	float bandMaxY = fbandDimY + fbandDelta;

    std::stable_sort(g_curves.data(), g_curves.data() + g_curves.size(),
            [](const Curve& a, const Curve& b) { 
                return Max(a.x1, a.x2, a.x3) > Max(b.x1, b.x2, b.x3);
            });

    float bandEps = 0;
	for(uint32_t b = 0; b < bandCount; ++b)
	{
		uint16_t bandTexelOffset = (uint16_t)(g_bandsTextureCurveOffsets.size() / 2); // 2x 16 bits
		uint16_t curveCount = 0;

        for(int i=0; i< g_curves.size(); ++i)
        {
            const Curve& c = g_curves[i];

			// reject perfectly horizontal curves
			if(c.y1 == c.y2 && c.y2 == c.y3) {
				continue;
			}

			// reject curves that don't cross the band
			const float curveMinY = Min(c.y1, c.y2, c.y3);
			const float curveMaxY = Max(c.y1, c.y2, c.y3);
			if(curveMinY > bandMaxY + bandEps || curveMaxY < bandMinY - bandEps) {
				continue;
			}

			// push the curve offsets
			const uint32_t texelIndex = c.texelIndex;
			const uint16_t curveOffsetX = (uint16_t)(texelIndex % (uint32_t)SLUG_TEXTURE_WIDTH);
			const uint16_t curveOffsetY = (uint16_t)(texelIndex / (uint32_t)SLUG_TEXTURE_WIDTH);
			g_bandsTextureCurveOffsets.push(curveOffsetX);
			g_bandsTextureCurveOffsets.push(curveOffsetY);

			++curveCount;
		}

		// @TODO: don't push more data if this band is the same as the previous one

		// push the horizontal band
		g_bandsTextureBandOffsets.push(curveCount);
		g_bandsTextureBandOffsets.push(bandTexelOffset);

		bandMinY += fbandDimY;
		bandMaxY += fbandDimY;

		if(bandTexelOffset >= 0xFFFF ||
		   g_bandsTextureCurveOffsets.size() / 2 >= 0xFFFF)
		{
			log_error("Too much data generated to be indexed! Try a lower band count.\n");
            exit(1);
		}
	}

	//
	// vertical bands
	//
	
	const uint32_t bandDimX = (dimX + bandCount - 1) / bandCount;
	const float fbandDimX = (float)bandDimX;
	float bandMinX = -fbandDelta;
	float bandMaxX = fbandDimX + fbandDelta;

    std::stable_sort(g_curves.data(), g_curves.data() + g_curves.size(),
            [](const Curve& a, const Curve& b) { 
                return Max(a.y1, a.y2, a.y3) > Max(b.y1, b.y2, b.y3);
            });

	for(uint32_t b = 0; b < bandCount; ++b)
	{
		uint16_t bandTexelOffset = (uint16_t)(g_bandsTextureCurveOffsets.size() / 2); // 2x 16 bits
		uint16_t curveCount = 0;

        for(int i=0; i< g_curves.size(); ++i)
        {
            const Curve& c = g_curves[i];

			// reject perfectly vertical curves
			if(c.x1 == c.x2 && c.x2 == c.x3) {
				continue;
			}

			// reject curves that don't cross the band
			const float curveMinX = Min(c.x1, c.x2, c.x3);
			const float curveMaxX = Max(c.x1, c.x2, c.x3);
			if(curveMinX > bandMaxX + bandEps || curveMaxX < bandMinX - bandEps) {
				continue;
			}

			// push the curve offsets
			const uint32_t texelIndex = c.texelIndex;
			const uint16_t curveOffsetX = (uint16_t)(texelIndex % (uint32_t)SLUG_TEXTURE_WIDTH);
			const uint16_t curveOffsetY = (uint16_t)(texelIndex / (uint32_t)SLUG_TEXTURE_WIDTH);
			g_bandsTextureCurveOffsets.push(curveOffsetX);
			g_bandsTextureCurveOffsets.push(curveOffsetY);

			++curveCount;
		}

		// @TODO: don't push more data if this band is the same as the previous one

		// push the vertical band
		g_bandsTextureBandOffsets.push(curveCount);
		g_bandsTextureBandOffsets.push(bandTexelOffset);

		bandMinX += fbandDimX;
		bandMaxX += fbandDimX;

		if(bandTexelOffset >= 0xFFFF || g_bandsTextureCurveOffsets.size() / 2 >= 0xFFFF)
		{
			log_error("Too much data generated to be indexed! Try a lower band count.\n");
            exit(1);
		}
	}

	// push the code point

	cp.codePoint = codePoint;
	cp.bandCount = bandCount;
	cp.bandDimX = bandDimX;
	cp.bandDimY = bandDimY;
	cp.bandsTexCoordX = (uint16_t)(bandsTexelIndex % (uint32_t)SLUG_TEXTURE_WIDTH);
	cp.bandsTexCoordY = (uint16_t)(bandsTexelIndex / (uint32_t)SLUG_TEXTURE_WIDTH);

	g_codePoints.push(cp);

    static bool b_print_debug = true;

    if (b_print_debug) {
        printf("codePoint: %d\n", codePoint);
        printf("curveCount: %d\n", (int)g_curves.size());
        printf("width = %d height: %d\n", cp.width, cp.height);
        printf("bandCount = %d\n", cp.bandCount);
        printf("dimX %d dimY %d\n", cp.bandDimX, cp.bandDimY);
        printf("bandTC: %d %d\n", cp.bandsTexCoordX, cp.bandsTexCoordY);

        printf("TextureBandOffsets:\n");
        for(int i=0; i< g_bandsTextureBandOffsets.size(); ++i) {
            uint16_t v = g_bandsTextureBandOffsets[i];
            printf("%d ", v);
            if (i % 32 == 31)
                puts("");
        }
        puts("");
        printf("TextureCurveOffsets:\n");
        for(int i=0; i< g_bandsTextureCurveOffsets.size(); ++i) {
            uint16_t v = g_bandsTextureCurveOffsets[i];
            printf("%d ", v);
            if (i % 32 == 31)
                puts("");
        }
        puts("");

        printf("CurvesTexture:\n");
        for (int i = 0; i < (int)g_curvesTexture.size() / 4; i ++) {
            float v0 = g_curvesTexture[4 * i + 0];
            float v1 = g_curvesTexture[4 * i + 1];
            float v2 = g_curvesTexture[4 * i + 3];
            float v3 = g_curvesTexture[4 * i + 4];
            printf("[%.4f %.4f %.4f %.4f] ", v0, v1, v2, v3);
            if (i % 4 == 3)
                puts("");
        }
        puts("");

        b_print_debug = false;
    }

	if(bandsTexelIndex / (uint32_t)SLUG_TEXTURE_WIDTH >= 0xFFFF)
	{
		log_error("Too much curve data generated! :-(\n");
        exit(1);
	}

	// check the data's validity
    for(int i=0; i< g_curves.size(); ++i)
    {
        const Curve& c = g_curves[i];
		const bool sameRow = c.texelIndex / SLUG_TEXTURE_WIDTH == (c.texelIndex + 1) / SLUG_TEXTURE_WIDTH;
		if(!sameRow) {
			log_warning("U+%04X encoding failed! Texel indices %u and %u are not in the same row\n",
						 (unsigned int)codePoint, (unsigned int)c.texelIndex, (unsigned int)c.texelIndex + 1);
		}
	}


	return true;
}

bool gos_save_slug_font(const char* outFile)
{
    FILE* h = fopen(outFile, "wb");
    if(!h) {
        int last_err = errno;
        printf("fopen: %s\n", strerror(last_err));
        return false;
    }

    fwrite("SLUGFONT", sizeof("SLUGFONT"), 1, h);

	fwrite(&g_globalFontData.lineSpacing, sizeof(g_globalFontData.lineSpacing), 1, h);
	fwrite(&g_globalFontData.unitsPerEm, sizeof(g_globalFontData.unitsPerEm), 1, h);

	const uint16_t codePointCount = (uint16_t)g_codePoints.size();
	fwrite(&codePointCount, sizeof(codePointCount), 1, h);
	fwrite(&g_codePoints[0], g_codePoints.size() * sizeof(SlugCodePoint),1, h);

	const uint16_t curvesTexWidth = SLUG_TEXTURE_WIDTH;
	const uint32_t curvesTexTexels = (uint32_t)g_curvesTexture.size() / 4;
	const uint32_t curvesTexBytes = (uint32_t)g_curvesTexture.size() * (uint32_t)sizeof(g_curvesTexture[0]);
	const uint16_t curvesTexHeight = (uint16_t)((curvesTexTexels + curvesTexWidth - 1) / curvesTexWidth);
	fwrite(&curvesTexWidth, sizeof(curvesTexWidth), 1, h);
	fwrite(&curvesTexHeight, sizeof(curvesTexHeight), 1, h);
	fwrite(&curvesTexBytes, sizeof(curvesTexBytes), 1, h);
	fwrite(&g_curvesTexture[0], curvesTexBytes, 1, h);

	const uint16_t bandsTexWidth = SLUG_TEXTURE_WIDTH;
	
	const uint32_t bandsTexTexels = (uint32_t)(g_bandsTextureBandOffsets.size() + g_bandsTextureCurveOffsets.size()) / 2;
	const uint32_t bandsTexBytes = bandsTexTexels * (uint32_t)sizeof(uint16_t) * 2;
	const uint16_t bandsTexHeight = (uint16_t)((bandsTexTexels + bandsTexWidth - 1) / bandsTexWidth);
	fwrite(&bandsTexWidth, sizeof(bandsTexWidth), 1, h);
	fwrite(&bandsTexHeight, sizeof(bandsTexHeight), 1, h);
	fwrite(&bandsTexBytes, sizeof(bandsTexBytes), 1, h);
	fwrite(g_bandsTextureBandOffsets.data(), g_bandsTextureBandOffsets.size() * sizeof(uint16_t), 1, h);
	fwrite(g_bandsTextureCurveOffsets.data(), g_bandsTextureCurveOffsets.size() * sizeof(uint16_t), 1, h);

    fclose(h);

    return true;
}

int main(int argc, char** argv)
{
    if(argc < 3) {
        usage(argv);
        return 1;
    }

    const char* const fontFile = argv[1];
    const char* outFile = argv[2];

    FILE* ttf_fh = fopen(fontFile,"rb");
    if(!ttf_fh) {
        printf("Failed to open font file: %s\n", fontFile);
        return 1;
    }

    fseek(ttf_fh, 0, SEEK_END);
    const size_t ttf_size = ftell(ttf_fh);
    fseek(ttf_fh, 0, SEEK_SET);

    unsigned char* ttf_buffer = (unsigned char*)malloc(ttf_size);
    if(!ttf_buffer) {
        printf("Failed to allocate memory for ttf file: %zu\n", ttf_size);
        fclose(ttf_fh);
        return 1;
    }

    size_t read_v = fread(ttf_buffer, ttf_size, 1, ttf_fh);
    if(read_v != 1) {
        printf("Failed to read ttf file: %s\n", fontFile);
        free(ttf_buffer);
        fclose(ttf_fh);
        return 1;
    }

    fclose(ttf_fh);


    stbtt_fontinfo font;
    // get offset for the first font (in case there are many)
    const int font0_offset = stbtt_GetFontOffsetForIndex(ttf_buffer,0);
    if(!stbtt_InitFont(&font, ttf_buffer, font0_offset)) {
        printf("Failed to init ttf font: %s\n", fontFile);
        free(ttf_buffer);
        return 1;
    }

    const float unitsPerEm = 1.0f / stbtt_ScaleForMappingEmToPixels(&font, 1.0f);
    printf("unitsPerEm: %f\n", unitsPerEm);
    g_globalFontData.unitsPerEm = (int)unitsPerEm;

    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(&font, &ascent, &descent, &lineGap);
    printf("Font ascent: %d descent: %d lineGap: %d\n", ascent, descent, lineGap);
    g_globalFontData.lineSpacing = ascent + descent + lineGap;

    int num_failed = 0;
    int num_ignored = 0;
    int startCodepoint = 32;
    int endCodepoint = 126;
    for(int i=startCodepoint; i<=endCodepoint; ++i) {
        bool b_was_ignored = false;
        if(!ProcessCodePoint(&font, i, &b_was_ignored)) {
            num_failed++;
            num_ignored += b_was_ignored?1:0;
        }
    }

    log_info("num_total:%d num_failed: %d num_ignored: %d\n", endCodepoint - startCodepoint, num_failed, num_ignored);
    
    if(0 == g_codePoints.size()) {
		log_error("No valid code point found in: %s\n", fontFile);
		return 1;
	}

	// fix up the bands' texel offsets first
	const uint32_t bandsTexTexels = (uint32_t)(g_bandsTextureBandOffsets.size() + g_bandsTextureCurveOffsets.size()) / 2;
	const uint16_t bandHeaderTexels = (uint16_t)(g_bandsTextureBandOffsets.size() / 2);
	for(int i = 1; i < g_bandsTextureBandOffsets.size(); i += 2)
	{
		g_bandsTextureBandOffsets[i] += bandHeaderTexels;
		if(g_bandsTextureBandOffsets[i] >= bandsTexTexels)
		{
			log_error("Too much data generated to be indexed! Try a lower band count.\n");
            exit(1);
		}
	}

    gos_save_slug_font(outFile);
    SlugFontData sfd;
    gos_load_slug_font(outFile, sfd);

    return 0;
}



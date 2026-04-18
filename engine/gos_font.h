#ifndef GOS_FONT_H
#define GOS_FONT_H

typedef struct {
    int32_t minx;
    int32_t maxx;
    int32_t miny;
    int32_t maxy;
    int32_t advance;
    uint32_t valid;
    uint32_t u;
    uint32_t v;
} gosGlyphMetrics;

struct gosGlyphInfo {
    gosGlyphInfo():num_glyphs_(0), glyphs_(0) {}
    uint32_t num_glyphs_;
    uint32_t start_glyph_;
    gosGlyphMetrics* glyphs_;
    uint32_t max_advance_;
    uint32_t font_ascent_;
    uint32_t font_line_skip_;
};

bool gos_load_glyphs(const char* glyphFile, gosGlyphInfo& gi);

#pragma pack(push, 1)
struct SlugCodePoint {
	uint32_t codePoint;
	uint32_t width;
	uint32_t height;
	uint32_t bandCount;
	uint32_t bandDimX;
	uint32_t bandDimY;
	uint16_t bandsTexCoordX;
	uint16_t bandsTexCoordY;
    int32_t advance;
    int32_t bearingX;
    int32_t minX;
    int32_t maxX;
    int32_t minY;
    int32_t maxY;
};
#pragma pack(pop)

#define SLUG_TEXTURE_WIDTH  4096
#define SLUG_TEXTURE_MASK  0xFFF
#define SLUG_TEXTURE_SHIFT    12

struct SlugFontData {
    int lineSpacing;
    int unitsPerEm;
    uint16_t codePointsCount;
    SlugCodePoint* codePoints;

    uint16_t curvesTexWidth;
    uint16_t curvesTexHeight;
    uint32_t curvesTexBytes;
    float* curvesTexture; // GL_RGBA32F [x1 y1 x2 y2]
                          //
    uint16_t bandsTexWidth;
    uint16_t bandsTexHeight;
    uint32_t bandsTexBytes;

    uint16_t* bandsTexture;
};

bool gos_load_slug_font(const char* file, SlugFontData& sfd);
void gos_destroy_slug_font(SlugFontData& sfd);

#endif // GOS_FONT_H

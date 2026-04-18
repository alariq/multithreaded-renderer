#include<inttypes.h>
#include <stdio.h>
// errno + strerror
#include <errno.h>
#include <string.h>
#include <assert.h>

#include "gameos.hpp"
#include "gos_font.h"
#include "utils/logging.h"

bool gos_load_glyphs(const char* glyphFile, gosGlyphInfo& gi)
{
    FILE* glyph_info = fopen(glyphFile, "rb");
    if(!glyph_info) {
        int last_err = errno;
        SPEW(("ERROR", "fopen: %s : %s\n", strerror(last_err), glyphFile));
        return false;
    }

    fread(&gi.num_glyphs_, sizeof(gi.num_glyphs_), 1, glyph_info);
    fread(&gi.start_glyph_, sizeof(gi.start_glyph_), 1, glyph_info);

    fread(&gi.max_advance_, sizeof(gi.max_advance_), 1, glyph_info);
    fread(&gi.font_ascent_, sizeof(gi.font_ascent_), 1, glyph_info);
    fread(&gi.font_line_skip_, sizeof(gi.font_line_skip_), 1, glyph_info);

    size_t num_structs_read = 0;

    gi.glyphs_ = new gosGlyphMetrics[gi.num_glyphs_];

    while(num_structs_read!= gi.num_glyphs_) {
        num_structs_read+= fread(&gi.glyphs_[num_structs_read], 
                sizeof(gosGlyphMetrics), 
                gi.num_glyphs_ - num_structs_read,
                glyph_info);
    }

    fclose(glyph_info);

    return true;
}

bool gos_load_slug_font(const char* file, SlugFontData& fd)
{
    FILE* h = fopen(file, "rb");
    if(!h) {
        int last_err = errno;
        printf("%s fopen: %s\n", file, strerror(last_err));
        return false;
    }

    char buf[sizeof("SLUGFONT")];
    int rv = fread(buf, sizeof("SLUGFONT"), 1, h);
    if(rv!=1 || strcmp(buf, "SLUGFONT")) {
        printf("Malformed file: %s\n", file);
        fclose(h);
        return false;
    }

	rv = fread(&fd.lineSpacing, sizeof(fd.lineSpacing), 1, h);
    assert(rv==1);
	rv = fread(&fd.unitsPerEm, sizeof(fd.unitsPerEm), 1, h);
    assert(rv==1);

	rv = fread(&fd.codePointsCount, sizeof(fd.codePointsCount), 1, h);
    assert(rv==1);
    const uint32_t num_codepoint_bytes = sizeof(SlugCodePoint)*fd.codePointsCount;
    fd.codePoints = (SlugCodePoint*)malloc(num_codepoint_bytes);
	rv = fread(fd.codePoints, num_codepoint_bytes, 1, h);
    assert(rv==1);

	rv = fread(&fd.curvesTexWidth, sizeof(fd.curvesTexWidth), 1, h);
    assert(rv==1);
	rv = fread(&fd.curvesTexHeight, sizeof(fd.curvesTexHeight), 1, h);
    assert(rv==1);
	rv = fread(&fd.curvesTexBytes, sizeof(fd.curvesTexBytes), 1, h);
    assert(rv==1);
    fd.curvesTexture = (float*)malloc(fd.curvesTexBytes);
    if(!fd.curvesTexture) {
        log_error("Failed to allocate memory for curves texture: %s\n", file);
        fclose(h);
        return false;
    }

	rv = fread(fd.curvesTexture, fd.curvesTexBytes, 1, h);
    assert(rv==1);

	rv = fread(&fd.bandsTexWidth, sizeof(fd.bandsTexWidth), 1, h);
    assert(rv==1);
	rv = fread(&fd.bandsTexHeight, sizeof(fd.bandsTexHeight), 1, h);
    assert(rv==1);
	rv = fread(&fd.bandsTexBytes, sizeof(fd.bandsTexBytes), 1, h);
    assert(rv==1);

    fd.bandsTexture = (uint16_t*)malloc(fd.bandsTexBytes);
    if(!fd.curvesTexture) {
        log_error("Failed to allocate memory for curves texture: %s\n", file);
        fclose(h);
        return false;
    }

	rv = fread(fd.bandsTexture, fd.bandsTexBytes, 1, h);
    assert(rv==1);
#if 0
    int count = (int)fd.curvesTexBytes/(4*sizeof(float));
    for(int i=0;i<count;++i) {
        vec4* pdata = (vec4*)fd.curvesTexture;
        printf("[%.4f %.4f %.4f %.4f] ", pdata[i].x, pdata[i].y, pdata[i].z, pdata[i].w);
        if(i % 8 == 7) puts("");
    }
#endif

#if 0
    struct pix { uint16_t x, y; };
    int count = (int)fd.bandsTexBytes/(2*sizeof(uint16_t));
    for(int i=0;i<count;++i) {
        pix* pdata = (pix*)fd.bandsTexture;
        printf("%d %d ", pdata[i].x, pdata[i].y);
        if(i % 16 == 15) puts("");
    }
#endif

    fclose(h);
    return true;
}

void gos_destroy_slug_font(SlugFontData& sfd) {
    free(sfd.curvesTexture);
    sfd.curvesTexture = nullptr;
    free(sfd.bandsTexture);
    sfd.bandsTexture = nullptr;
}


#ifndef GL_RENDER_CONSTANTS
#define GL_RENDER_CONSTANTS

#include "render_constants.h"
#include <cassert>

extern const GLint textureFormats[TF_COUNT];
extern const GLint textureInternalFormats[TF_COUNT];
extern const int textureFormatNumChannels[TF_COUNT];
extern const GLint textureFormatChannelType[TF_COUNT];

static inline uint32_t getInternalTextureFormat(TexFormat format)
{
    assert(format > 0 && format < TF_COUNT);
    return textureInternalFormats[format];
}

#endif // GL_RENDER_CONSTANTS

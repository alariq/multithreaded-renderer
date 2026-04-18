//#version 300 es

layout(location = 0) in vec4 pos;
layout(location = 1) in vec4 color;
layout(location = 2) in vec2 texcoord;
layout(location = 3) in vec4 vaScaleBias;
layout(location = 4) in vec4 vaGlyphBandScale;
layout(location = 5) in uvec4 vaBandMaxTexCoords;

uniform mat4 mvp;

// Constant data for current glyph.
flat out vec4 glyphBandScale; // glyphParam
// Scale and offset for band indexes.
flat out uvec4 bandMaxTexCoords; // bandParam

out vec4 Color;
out vec2 Texcoord;

void main(void)
{
    gl_Position = vec4(pos.xy * vaScaleBias.xy + vaScaleBias.zw, 0, 1);
    Color = color;
    Texcoord = vec2(texcoord.x, 1.0 - texcoord.y) * vaGlyphBandScale.xy;

	glyphBandScale = vaGlyphBandScale;
    bandMaxTexCoords = vaBandMaxTexCoords;
}


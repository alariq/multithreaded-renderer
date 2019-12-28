//#version 450
#define PREC highp

uniform sampler2D tex;

in PREC vec2 o_pos;
in PREC vec2 o_uv;

void main(void)
{
	gl_FragDepth = texture(tex, o_uv).r;
}

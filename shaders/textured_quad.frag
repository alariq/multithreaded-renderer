#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex;

in PREC vec2 o_pos;
in PREC vec2 o_uv;

void main(void)
{
	PREC vec3 c = texture2D(tex, o_uv).rgb;
	FragColor = vec4(c, 1.0);
}

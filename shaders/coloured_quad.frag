#define PREC highp

layout (location=0) out PREC vec4 FragColor;

in PREC vec2 o_pos;
in PREC vec4 o_colour;

void main(void)
{
	FragColor = vec4(o_pos.xy*o_colour.xy, o_colour.z, 1.0);
}

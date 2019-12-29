#define PREC highp

layout (location=0) out PREC vec4 FragColor;

in PREC vec3 o_normal;

uniform vec4 colour_;

void main(void)
{
	FragColor = vec4(colour_);// + vec4(0.5*o_normal + 0.5, 1.0));
}

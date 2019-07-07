#define PREC highp

layout (location=0) out PREC vec4 FragColor;

in PREC vec2 tc;
in PREC vec3 norm;

void main(void)
{
	FragColor = vec4(1.0);
}

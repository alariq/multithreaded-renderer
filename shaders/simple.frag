#define PREC highp

layout (location=0) out PREC vec4 FragColor;

in PREC vec2 tc;
in PREC vec3 norm;
uniform sampler2D tex;

void main(void)
{
	PREC vec3 c = norm*0.5 + 0.5;// texture2D(tex, tc).rgb;
	FragColor = vec4(c + vec3(0.1,0.1,0.1), 1.0);
}

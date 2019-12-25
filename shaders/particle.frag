#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex;
uniform vec4 has_texture;

in PREC vec2 o_uv;
in PREC float o_lifetime;

void main(void)
{
	PREC vec3 c = has_texture.x!=0.0 ? texture2D(tex, o_uv).rgb : vec3(1.0);
	//FragColor = vec4( 0.0000000001*c*(1.0 - o_lifetime) + uv.x, uv.y, 0.0, 1.0);
	FragColor = vec4(o_uv.x, o_uv.y, 0.0, (1.0 - o_lifetime) );
}

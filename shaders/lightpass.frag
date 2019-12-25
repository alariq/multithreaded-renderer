
#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex1;
uniform sampler2D tex2;
uniform sampler2D tex3;

uniform vec4 lightdir_;

in PREC vec2 o_pos;
in PREC vec2 o_uv;

void main(void)
{

	PREC vec3 albedo = texture2D(tex1, o_uv).rgb;
	PREC vec3 normal = texture2D(tex2, o_uv).rgb;
	PREC float depth = texture2D(tex3, o_uv).r;

    normal = 2.0*normal - vec3(1.0);

    float ndotl = max(0.2, dot(normal, -lightdir_.xyz));
    float intensity = 1.2;
    vec3 diffuse = albedo * ndotl * intensity;
    vec3 ambient = vec3(0.1, 0.05, 0.05);

	FragColor = vec4(diffuse + ambient, 1.0);
}

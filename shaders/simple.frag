#define PREC highp

layout (location=0) out PREC vec4 FragColor;

in PREC vec2 tc;
in PREC vec3 norm;
in PREC vec4 shadow_pos;

uniform sampler2D tex1;
uniform sampler2D tex2;

uniform vec4 lightdir_;
uniform vec4 has_texture;
uniform mat4 vpshadow_;

#define PCF

void main(void)
{

    PREC vec3 shadow_proj_pos = shadow_pos.xyz / shadow_pos.w;

    // xy -> -1.1 to 0..1 to transfrom from NDC to uv
    // z -> -1..1 to 0..1 to transform from NDC to viewport transform
    shadow_proj_pos.xyz = shadow_proj_pos.xyz*0.5 + vec3(0.5);

	PREC vec3 c = has_texture.x ? texture2D(tex1, tc).rgb : norm*0.5 + 0.5;

    const float bias = max(0.0005 * (1.0 - dot(norm, lightdir_.xyz)), 0.00005);
#ifdef PCF
    PREC float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(tex2, 0);
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            float depth = texture(tex2, shadow_proj_pos.xy + vec2(x, y) * texelSize).x; 
            shadow += shadow_proj_pos.z - bias > depth ? 0.25 : 1.0;
         }    
    }
    shadow /= 9.0;
#else
	PREC float depth = texture2D(tex2, vec2(shadow_proj_pos.xy)).x;
    float shadow = depth + bias > shadow_proj_pos.z ? 1.0 : 0.25;
#endif

    if(has_texture.y > 0.0) // has shadowmap
    {
        c *= shadow;
    }

	FragColor = vec4(c, 1.0);
}

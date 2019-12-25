#define PREC highp

#define PCF 
#define PCF_4TAP

#ifdef PCF
    #define TexSampler_t sampler2DShadow
#else
    #define TexSampler_t sampler2D
#endif


layout (location=0) out PREC vec4 FragColor;

in PREC vec2 tc;
in PREC vec3 norm;
in PREC vec3 opos;
in PREC vec4 shadow_pos0;
in PREC vec4 shadow_pos1;
in vec4 gl_FragCoord;

uniform sampler2D tex1;
uniform TexSampler_t tex2;
uniform TexSampler_t tex3;

uniform vec4 lightdir_;
uniform vec4 has_texture;
uniform mat4 wvpshadow0_;
uniform mat4 wvpshadow1_;
uniform vec4 z_far_; // far z in projection (+normalized to 0..1) space for each cascade (x,y,z,w)


#define EPSILON -0.001


float getShadow(TexSampler_t shadow_sampler, const vec3 tc, float bias)
{
#ifdef PCF
    PREC float shadow = 0.0;
    vec2 texSize = textureSize(shadow_sampler, 0);
    vec2 texelSize = 1.0 / texSize;

#if defined(PCF_4TAP)
    vec2 offset = 0.5*texelSize;
    vec2 snapped_pos = floor(tc.xy * texSize + vec2(0.5)) * texelSize;
    //vec3 sp = vec3(snapped_pos, tc.z);
    //vec2 w = abs(snapped_pos + offset - tc.xy) * texSize;   
    vec3 sp = tc;
    vec2 w = offset * texSize;
    //float ret = texture(shadow_sampler, sp + vec3(offset, -bias)).x * (1-w.x) * (1-w.y);
    //ret += texture(shadow_sampler, sp + vec3(offset.x, -offset.y, -bias)).x * (1-w.x) * (w.y);
    //ret += texture(shadow_sampler, sp + vec3(-offset.x, offset.y, -bias)).x * (w.x) * (1-w.y);
    //ret += texture(shadow_sampler, sp + vec3(-offset.x, -offset.y, -bias)).x * (w.x) * (w.y);
 
    // 9 tap pcf
    float Factor = 0.0;
    for (int y = -1 ; y <= 1 ; y++) {
        for (int x = -1 ; x <= 1 ; x++) {
            vec2 Offsets = vec2(x * texelSize.x, y * texelSize.y);
            vec3 UVC = vec3(tc.xy + Offsets, tc.z - bias);
            Factor += texture(shadow_sampler, UVC);
        }
    }
    float ret = 0.5 + Factor/18.0;
#else
    float ret = texture(shadow_sampler, tc + vec3(0, 0, -bias)).x;
#endif // PCF_4TAP
    shadow = ret;

#else // !PCF
	PREC float depth = texture2D(shadow_sampler, tc.xy).x;
    float shadow = depth + bias > tc.z ? 1.0 : 0.25;
#endif

    return shadow;
}

void main(void)
{
    float cascade_idx = 0;
#if 0 // vertex shader transforms
    PREC vec3 shadow_proj_pos = shadow_pos0.xyz / shadow_pos0.w;
    // if this pixel falls in cascade1 calculate corresponding textur map coords
    if(gl_FragCoord.z > z_far_.x)
    {
        shadow_proj_pos = shadow_pos1.xyz / shadow_pos1.w;
        cascade_idx = 1.0f;
    }
#else // pixel shader transforms
    PREC vec3 shadow_proj_pos = (wvpshadow0_*vec4(opos, 1)).xyz;
    if(gl_FragCoord.z > z_far_.x)
    {
        shadow_proj_pos = (wvpshadow1_*vec4(opos, 1)).xyz;
        cascade_idx = 1.0f;
    }
#endif

    // xy -> -1.1 to 0..1 to transfrom from NDC to uv
    // z -> -1..1 to 0..1 to transform from NDC to viewport transform
    shadow_proj_pos.xyz = shadow_proj_pos.xyz*0.5 + vec3(0.5);

	PREC vec3 albedo = has_texture.x!=0.0 ? texture2D(tex1, vec2(tc.x, 1-tc.y)).rgb : norm*0.5 + 0.5;

    float bias = 0.00001 + 0.0003*(1.0 - dot(norm, lightdir_.xyz));// max(0.000018* (1.0 - (dot(norm, lightdir_.xyz))), 0.00004);
    bias *= 1.0*cascade_idx + 2.5f;
    //bias = -1.0*bias;
    
    float shadow = 1.0;
    if(gl_FragCoord.z > z_far_.x)
        shadow = getShadow(tex3, shadow_proj_pos.xyz, bias);
    else
        shadow = getShadow(tex2, shadow_proj_pos.xyz, bias);

    PREC vec3 color = albedo;
    if(has_texture.y > 0.0) // has shadowmap
    {
        PREC float shadow_ambient = 0.8;
        color = color*shadow*shadow_ambient + color*(1-shadow_ambient);
    }

	FragColor = vec4(color, 1.0);
    // visualize cascades
	//FragColor = vec4(color * (gl_FragCoord.z > z_far_.x ? vec3(1,0.5,0.5) : vec3(0.5,1,0.5)), 1.0);
}


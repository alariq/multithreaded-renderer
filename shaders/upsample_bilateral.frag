//#version 450
#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform mat4 proj_;

uniform sampler2D tex1;// low_res_colour_;
uniform sampler2D tex2;// low_res_depth_;
uniform sampler2D tex3;// full_res_depth_;

#define low_res_colour_ tex1
#define low_res_depth_  tex2
#define full_res_depth_ tex3

uniform vec4 low_res_texture_size_;

in PREC vec2 o_pos;
in PREC vec2 o_uv;

in vec2 o_uv00;
in vec2 o_uv10;
in vec2 o_uv01;
in vec2 o_uv11;

float fetch_linear_z(float z_buf_depth, mat4 proj)
{
    // z_ndc same thing as z post projection, just a different name
    float z_ndc = 2.0*z_buf_depth - 1; // may be influenced by glDepthRange

    // post projection Z -> view Z
    // post projection Z is Zproj / Wproj , where Wproj is Zview
    float view_space_depth = proj[3][2] / (z_ndc - proj[2][2]);
    return view_space_depth;
}
    
float fetchLowResDepth(vec2 uv)
{
    float z = texture(low_res_depth_, vec2(uv.x,uv.y)).r;
    return fetch_linear_z(z, proj_);
}

float fetchFullResDepth(vec2 uv)
{
    float z = texture(full_res_depth_, vec2(uv.x, uv.y)).r;
    return fetch_linear_z(z, proj_);
}

void main(void)
{
    // simple bilinear
    //FragColor = texture(low_res_colour_, o_uv);
    //return;

	// the lowp specifiers below are used to reduce register pressure
	lowp vec4 c00 = texture(low_res_colour_, o_uv00);
	lowp vec4 c10 = texture(low_res_colour_, o_uv10);
	lowp vec4 c01 = texture(low_res_colour_, o_uv01);
	lowp vec4 c11 = texture(low_res_colour_, o_uv11);

	float z00 = fetchLowResDepth(o_uv00);
	float z10 = fetchLowResDepth(o_uv10);
	float z01 = fetchLowResDepth(o_uv01);
	float z11 = fetchLowResDepth(o_uv11);

    //ivec2 texSize = textureSize(low_res_depth_, 0);
    //vec2 low_res_tex_size = vec2(texSize.x, texSize.y);

	vec2 f = fract(o_uv00 * low_res_texture_size_.xy);
	vec2 g = vec2(1.0) - f;
	float w00 = g.x * g.y;
	float w10 = f.x * g.y;
	float w01 = g.x * f.y;
	float w11 = f.x * f.y;

    const float depthMult_ = 32.0;
    const float threshold_  = 0.1;

	float zfull = fetchFullResDepth(o_uv);
	vec4 depthDelta = abs(vec4(z00,z10,z01,z11) - vec4(zfull));
	vec4 weights = vec4(w00,w10,w01,w11) / (depthDelta * depthMult_ + vec4(1.0));

	float weightSum = dot(weights, vec4(1.0));
	weights /= weightSum;
	
	lowp vec4 outputColor =	c00 * weights.x +
							c10 * weights.y +
							c01 * weights.z +
							c11 * weights.w;

#define ENABLE_ISLAND_FIX 0

#if ENABLE_ISLAND_FIX
	if (weightSum < threshold_)
	{
	    lowp vec4 nearestDepthColor = c00;
		float dist = abs(z00 - zfull);
	    float minDist = dist;
		dist = abs(z10 - zfull); if (dist < minDist) { minDist = dist; nearestDepthColor = c10; }
		dist = abs(z01 - zfull); if (dist < minDist) { minDist = dist; nearestDepthColor = c01; }
		dist = abs(z11 - zfull); if (dist < minDist) { minDist = dist; nearestDepthColor = c11; }
		outputColor = nearestDepthColor;
	}
#endif

	FragColor = outputColor;
    //FragColor = vec4(z00.xxx/10,1);
}

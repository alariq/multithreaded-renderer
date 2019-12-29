//#version 450

layout(location = 0) in vec2 pos;
layout(location = 1) in vec2 uv;

uniform vec4 low_res_texel_size_;

out vec2 o_pos;
out vec2 o_uv;

out vec2 o_uv00;
out vec2 o_uv10;
out vec2 o_uv01;
out vec2 o_uv11;



void main(void)
{
	gl_Position     = vec4(pos.xy, -1, 1);
    o_pos = pos;
    o_uv = pos*0.5 + 0.5;
 
	o_uv00 = o_uv - 0.5 * low_res_texel_size_.xy;
	o_uv10 = o_uv00 + vec2(low_res_texel_size_.x, 0.0);
	o_uv01 = o_uv00 + vec2(0.0, low_res_texel_size_.y);
	o_uv11 = o_uv00 + low_res_texel_size_.xy;

}


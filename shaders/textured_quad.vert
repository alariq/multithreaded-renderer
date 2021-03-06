
layout(location = 0) in vec2 pos;
layout(location = 1) in vec2 uv;

uniform vec4 scale_offset;

out vec2 o_pos;
out vec2 o_uv;

void main(void)
{
	gl_Position     =  vec4(scale_offset.zw + scale_offset.xy*pos.xy, 0.0, 1);
    o_pos = pos;
    o_uv = pos*0.5 + 0.5;
}


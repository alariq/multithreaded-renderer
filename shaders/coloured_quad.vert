
layout(location = 0) in vec2 pos;


uniform vec4 colour;
uniform vec4 scale_offset;

out vec2 o_pos;
out vec4 o_colour;

void main(void)
{
	gl_Position     =  vec4(scale_offset.zw + scale_offset.xy*pos.xy, 0.9999, 1);
    o_pos = pos;
    o_colour = colour;
}


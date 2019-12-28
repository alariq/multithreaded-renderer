
layout(location = 0) in vec2 pos;
layout(location = 1) in vec2 uv;


out vec2 o_pos;
out vec2 o_uv;

void main(void)
{
	gl_Position     = vec4(pos.xy, 0.9999, 1);
    o_pos = pos;
    o_uv = pos*0.5 + 0.5;
}


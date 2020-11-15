#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex;

in PREC vec2 o_pos;
in PREC vec2 o_uv;

void main(void)
{
	PREC vec3 c = texture2D(tex, o_uv).rgb;
    PREC float part_radius = 0.1;

    //PREC float v = clamp(0.5* (c.x / part_radius)+0.5, 0.0f, 1.0f);
    //PREC vec3 color = mix(vec3(1,0,0), vec3(0,1,0), v);
    
    PREC float v = 1 - clamp(abs((c.x / part_radius)), 0.0, 1.0);
    PREC vec3 color = v.xxx;
	
    FragColor = vec4(color, 1.0);
}

#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex;

in PREC vec2 o_pos;
in PREC vec2 o_uv;


#define SDF
//#define EIGEN


void main(void)
{
	PREC vec4 c = texture2D(tex, o_uv).rgba;
    PREC float part_radius = 0.1;

    //PREC float v = clamp(0.5* (c.x / part_radius)+0.5, 0.0f, 1.0f);
    //PREC vec3 color = mix(vec3(1,0,0), vec3(0,1,0), v);
    
    //PREC float v = 1 - clamp(abs((c.x / part_radius)), 0.0, 1.0);
    
    
    // volume, dist
    PREC float v = c.x;//abs(c.x);
    
    //PREC vec3 color = v == 9999.0 ? vec3(1,0,0) : (v<0.0 ? vec3(0,v,0) : vec3(0,0,v) );
#if defined(SDF)
    v = c.x==9999.0f ? 1.0 : c.x;
    v = abs(v)<0.036 ? v : 1;
    v = pow(1.0-abs(v), 32.0);
    PREC vec3 color  = v.xxx;
#elif defined(EIGEN)
    PREC vec3 color  = v == 9999.0 ? vec3(0) : abs(50*v.xxx);
#endif

	// normal
    //PREC vec3 color = vec3(c.zy, 0);

    FragColor = vec4(color, 1.0);
}

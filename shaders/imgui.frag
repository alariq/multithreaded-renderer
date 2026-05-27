//#version 130

uniform sampler2D tex1;
in vec2 Frag_UV;
in vec4 Frag_Color;
out vec4 Out_Color;

vec3 linear_to_srgb(vec3 cl) {

    cl = clamp(cl, 0, 1);

    vec3 cs;
    cs.r =  cl.r < 0.0031308 ? 12.92 * cl.r : 1.055 * pow(cl.r, 0.41666) - 0.055;
    cs.g =  cl.g < 0.0031308 ? 12.92 * cl.g : 1.055 * pow(cl.g, 0.41666) - 0.055;
    cs.b =  cl.b < 0.0031308 ? 12.92 * cl.b : 1.055 * pow(cl.b, 0.41666) - 0.055;
    return cs;
}


void main()
{
    vec4 c = Frag_Color * texture(tex1, Frag_UV.st);
    Out_Color = vec4(linear_to_srgb(c.rgb), 1);
}

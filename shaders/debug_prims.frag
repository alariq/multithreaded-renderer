//#version 450

// material to draw debug lines, and other stuff
//

#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex1;

uniform vec4 colour_;
uniform vec4 flags_;

in PREC vec4 o_colour;
in PREC vec2 o_uv;

void main(void)
{
    PREC vec4 tex_color = texture(tex1, o_uv);
    if(flags_.x == 0.0) {
        tex_color = vec4(1.0);
    }

    // have not decided which one to use
    FragColor = colour_*tex_color + 0.0001*o_colour;// + vec4(0.5,0.5,0.5,0.5);
}


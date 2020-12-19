//#version 450

// material to draw debug lines, and other stuff
//

#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform vec4 colour_;
in PREC vec4 o_colour;

void main(void)
{
    // have not decided whch one to use
    FragColor = colour_ + 0.0001*o_colour;
}


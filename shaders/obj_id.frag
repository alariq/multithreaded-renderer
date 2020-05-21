//#version 450

#define PREC highp

layout (location=0) out PREC int ObjId;

uniform vec4 obj_id_;

void main(void)
{
    ObjId = int(obj_id_.x);
}


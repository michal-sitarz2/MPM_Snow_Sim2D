#version 430
layout(local_size_x = 64) in;

layout(std430, binding = 7)  coherent buffer GMass { float gMass[]; };
layout(std430, binding = 8)  coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 9) coherent buffer GVelY { float gVelY[]; };
layout(std430, binding = 10) coherent buffer GVelCol { vec2 gVelCol[]; };
layout(std430, binding = 11) coherent buffer GVelFric { vec2 gVelFric[]; };

uniform int gWidth;
uniform int gHeight;

void main()
{
    uint id = gl_GlobalInvocationID.x;
    uint total = uint((gWidth + 1) * (gHeight + 1));
    if (id >= total) return;

    gMass[id] = 0.0;
    gVelX[id] = 0.0;
    gVelY[id] = 0.0;
    gVelCol[id] = vec2(0.0);
    gVelFric[id] = vec2(0.0);
}
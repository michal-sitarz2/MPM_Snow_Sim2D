#version 430
layout(local_size_x = 64) in;

layout(std430, binding = 7)  coherent buffer GMass { float gMass[]; };
layout(std430, binding = 8)  coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 9) coherent buffer GVelY { float gVelY[]; };
layout(std430, binding = 10) coherent buffer GVelZ { float gVelZ[]; };
layout(std430, binding = 11) coherent buffer GVelCol { vec4 gVelCol[]; };
layout(std430, binding = 12) coherent buffer GVelFric { vec4 gVelFric[]; };

uniform int gWidth;
uniform int gHeight;
uniform int gDepth;

void main()
{
    uint id = gl_GlobalInvocationID.x;
    uint total = uint((gWidth + 1) * (gHeight + 1) * (gDepth + 1));
    if (id >= total) return;

    gMass[id] = 0.0;
    gVelX[id] = 0.0;
    gVelY[id] = 0.0;
    gVelZ[id] = 0.0;
    gVelCol[id] = vec4(0.0);
    gVelFric[id] = vec4(0.0);
}
#version 460 compatibility
#extension GL_NV_shader_atomic_float : enable
// #include common.glsl
layout(local_size_x = 64) in;

/* Particle variables */
layout(std430, binding = 0) readonly buffer Pos { vec2  pos[]; };
layout(std430, binding = 1) readonly buffer Vel { vec2  vel[]; };
layout(std430, binding = 2) readonly buffer Mass { float mass[]; };
layout(std430, binding = 6) readonly buffer Ap { mat2  ap[]; };
layout(std430, binding = 7) readonly buffer Bp { mat2  bp[]; };

/* Grid variables */
layout(std430, binding = 8)  coherent buffer GMass { float gMass[]; };
layout(std430, binding = 9)  coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 10) coherent buffer GVelY { float gVelY[]; };
layout(std430, binding = 11) coherent buffer GForceX { float gForceX[]; };
layout(std430, binding = 12) coherent buffer GForceY { float gForceY[]; };

uniform int numParticles;
uniform int gWidth;
uniform int gHeight;

void main()
{
    uint pid = gl_GlobalInvocationID.x;
    if (pid >= uint(numParticles)) return;

    vec2 pPos = pos[pid];
    vec2 pVel = vel[pid];
    float pMass = mass[pid];
    mat2 pBp = bp[pid];
    mat2 pAp = ap[pid];

    int gx = int(pPos.x);
    int gy = int(pPos.y);

    /* Particles to Grid */
    for (int i = gx - 1; i <= gx + 2; i++)  
    { // Neihbour loop: 4 cells in each direction (cubic B-spline)
        for (int j = gy - 1; j <= gy + 2; j++)
        {
            if (i < 0 || i > gWidth || j < 0 || j > gHeight) continue;

            /* Get grid index */
            int cid = j * (gWidth + 1) + i;
            vec2 gi = vec2(float(i), float(j));
            
            /* Compute the Grid Weights */
            vec2 dist = pPos - gi;
            float w  = computeWeight(dist);
            vec2 dw = computeGradWeight(dist);

            /* APIC velocity */
            float m = w * pMass;
            vec2 apic_v = pVel + 3.0 * pBp * (-dist);
            vec2 wv = m * apic_v;

            /* Compute Force */
            vec2 f = pAp * dw;

            atomicAdd(gMass[cid], m);
            atomicAdd(gVelX[cid], wv.x);
            atomicAdd(gVelY[cid], wv.y);
            atomicAdd(gForceX[cid], f.x);
            atomicAdd(gForceY[cid], f.y);
        }
    }
}
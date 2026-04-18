#version 460 compatibility
#extension GL_NV_shader_atomic_float : enable
// #include common.glsl
layout(local_size_x = 64) in;

/* Particle variables */
layout(std430, binding = 0) readonly buffer Pos { vec2 pos[]; };
layout(std430, binding = 1) readonly buffer Vel { vec2 vel[]; };
layout(std430, binding = 2) readonly buffer Mass { float mass[]; };
layout(std430, binding = 3) readonly buffer Vol { float vol[]; };
layout(std430, binding = 4) readonly buffer Fe { mat2 fe[]; };
layout(std430, binding = 5) readonly buffer Fp { mat2 fp[]; };
layout(std430, binding = 6) readonly buffer Bp { mat2 bp[]; };

/* Grid variables */
layout(std430, binding = 7)  coherent buffer GMass { float gMass[]; };
layout(std430, binding = 8)  coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 9) coherent buffer GVelY { float gVelY[]; };

uniform int numParticles;
uniform int gWidth;
uniform int gHeight;

uniform float XI;
uniform float MU;
uniform float LAMBDA;
uniform float DELTA_TIME;

void main()
{
    uint pid = gl_GlobalInvocationID.x;
    if (pid >= uint(numParticles)) return;

    vec2 pPos = pos[pid];
    vec2 pVel = vel[pid];
    float pMass = mass[pid];
    float pVol = vol[pid];
    mat2 pBp = bp[pid];
    mat2 pFe = fe[pid];
    mat2 pFp = fp[pid];

    if (pVol == 0.0) return;

    /* Compute Stress */
    float J_e = determinant(pFe);
    float J_p = determinant(pFp);

    mat2 U, V; vec2 Sigma;
    svd(pFe, U, Sigma, V);
    mat2 R = polarR(U, V);

    float harden = exp(XI * (1.0 - J_p));
    float mu     = MU * harden;
    float lambda = LAMBDA * harden;

    /* Cauchy Stress */
    mat2 PFt = 2.0 * mu * (pFe - R) * transpose(pFe)
             + lambda * (J_e - 1.0) * J_e * mat2(1.0);
    mat2 stress = PFt;

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
            
            vec2 dist = pPos - gi;

            /* Compute the Grid Weights */
            float w  = computeWeight(dist);

            /* APIC momentum */
            float m = w * pMass;
            vec2 apic_v = pVel + 3.0 * pBp * (-dist);

            /* MLS-MPM: force folded into momentum */
            vec2 force_contrib = pVol * stress * dist * 3.0 * DELTA_TIME;
            vec2 wv = m * apic_v + w * force_contrib;

            atomicAdd(gMass[cid], m);
            atomicAdd(gVelX[cid], wv.x);
            atomicAdd(gVelY[cid], wv.y);
        }
    }
}
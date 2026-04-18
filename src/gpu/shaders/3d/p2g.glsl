#version 460 compatibility
#extension GL_NV_shader_atomic_float : enable
// #include common.glsl
layout(local_size_x = 64) in;

/* Particle variables */
layout(std430, binding = 0) readonly buffer Pos { vec4 pos[]; };
layout(std430, binding = 1) readonly buffer Vel { vec4 vel[]; };
layout(std430, binding = 2) readonly buffer Mass { float mass[]; };
layout(std430, binding = 3) readonly buffer Vol { float vol[]; };
layout(std430, binding = 4) readonly buffer Fe { mat4 fe[]; };
layout(std430, binding = 5) readonly buffer Fp { mat4 fp[]; };
layout(std430, binding = 6) readonly buffer Bp { mat4 bp[]; };

/* Grid variables */
layout(std430, binding = 7)  coherent buffer GMass { float gMass[]; };
layout(std430, binding = 8)  coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 9) coherent buffer GVelY { float gVelY[]; };
layout(std430, binding = 10) coherent buffer GVelZ { float gVelZ[]; };

uniform int numParticles;
uniform int gWidth;
uniform int gHeight;
uniform int gDepth;

uniform float XI;
uniform float MU;
uniform float LAMBDA;
uniform float DELTA_TIME;

void main()
{
    uint pid = gl_GlobalInvocationID.x;
    if (pid >= uint(numParticles)) return;

    vec3 pPos = pos[pid].xyz;
    vec3 pVel = vel[pid].xyz;
    float pMass = mass[pid];
    float pVol = vol[pid];
    mat3 pBp = mat3(bp[pid]);
    mat3 pFe = mat3(fe[pid]);
    mat3 pFp = mat3(fp[pid]);

    if (pVol == 0.0) return;

    /* Compute Stress */
    float J_e = determinant(pFe);
    float J_p = determinant(pFp);

    mat3 U, V; vec3 Sigma;
    svd(pFe, U, Sigma, V);
    mat3 R = polarR(U, V);

    float harden = exp(XI * (1.0 - J_p));
    float mu     = MU * harden;
    float lambda = LAMBDA * harden;

    /* Cauchy Stress */
    mat3 PFt = 2.0 * mu * (pFe - R) * transpose(pFe)
             + lambda * (J_e - 1.0) * J_e * mat3(1.0);
    mat3 stress = PFt;

    int gx = int(pPos.x), gy = int(pPos.y), gz = int(pPos.z);

    /* Particles to Grid */
    for (int i = gx - 1; i <= gx + 2; i++)  
    { // Neihbour loop: 4 cells in each direction (cubic B-spline)
        for (int j = gy - 1; j <= gy + 2; j++)
        {
            for (int k = gz - 1; k <= gz + 2; k++)
            {
                if (i < 0 || i > gWidth || j < 0 || j > gHeight || k < 0 || k > gDepth) continue;

                /* Get grid index */
                int cid = k * (gWidth + 1) * (gHeight + 1) + j * (gWidth + 1) + i; 
                vec3 gi = vec3(float(i), float(j), float(k));
                vec3 dist = pPos - gi;

                /* Compute the Grid Weights */
                float w  = computeWeight(dist);

                /* APIC momentum */
                float m = w * pMass;
                vec3 apic_v = pVel + 3.0 * pBp * (-dist);

                /* MLS-MPM: force folded into momentum */
                vec3 force_contrib = pVol * stress * dist * 3.0 * DELTA_TIME;
                vec3 wv = m * apic_v + w * force_contrib;

                atomicAdd(gMass[cid], m);
                atomicAdd(gVelX[cid], wv.x);
                atomicAdd(gVelY[cid], wv.y);
                atomicAdd(gVelZ[cid], wv.z);
            }
        }
    }
}
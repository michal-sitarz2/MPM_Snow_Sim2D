#version 430
// #include common.glsl
layout(local_size_x = 64) in;

layout(std430, binding = 0) coherent buffer Pos { vec2 pos[]; };
layout(std430, binding = 1) coherent buffer Vel { vec2 vel[]; };
layout(std430, binding = 4) coherent buffer Fe { mat2 fe[]; };
layout(std430, binding = 5) coherent buffer Fp { mat2 fp[]; };
layout(std430, binding = 6) coherent buffer Bp { mat2 bp[]; };

layout(std430, binding = 10) readonly buffer GVelCol { vec2 gVelCol[]; };
layout(std430, binding = 11) readonly buffer GVelFric { vec2 gVelFric[]; };

uniform int numParticles;
uniform int gWidth;
uniform int gHeight;
uniform float DELTA_TIME;
uniform float THETA_C;
uniform float THETA_S;

void main()
{
    
    uint pid = gl_GlobalInvocationID.x;
    if (pid >= uint(numParticles)) return;

    /* Grid to Particles */
    vec2 pPos = pos[pid];
    int gx = int(pPos.x);
    int gy = int(pPos.y);
    vec2 old_pos = pPos;

    vec2 new_vel = vec2(0.0);
    mat2 new_Bp = mat2(0.0);
    vec2 new_pos = vec2(0.0);

    for (int i = gx - 1; i <= gx + 2; i++)
    {
        for (int j = gy - 1; j <= gy + 2; j++)
        {
            // Check if the interpolated cell in bounds
            if (i < 0 || i > gWidth || j < 0 || j > gHeight) continue;

            // Get the corresponding cell
            int cid = j * (gWidth + 1) + i;
            vec2 gi = vec2(float(i), float(j));
            
            // Distance between particle and cell
            vec2 dist = old_pos - gi;

            // Weights
            float weight = computeWeight(dist);

            // Update Velocity
            vec2 vFric = gVelFric[cid], vCol  = gVelCol[cid];
            new_vel += vFric * weight;

            // Update APIC
            new_Bp += weight * mat2(
                vFric.x * (-dist.x), vFric.y * (-dist.x),
                vFric.x * (-dist.y), vFric.y * (-dist.y)
            );

            // Update position
            new_pos += (DELTA_TIME * vCol + gi) * weight;
        }
    }

    vel[pid] = new_vel;
    bp[pid] = new_Bp;
    pos[pid] = new_pos;

    /* Update Deformation Gradient using APIC Bp */
    mat2 C = 3.0 * new_Bp;
    mat2 FE_hat = (mat2(1.0) + DELTA_TIME * C) * fe[pid];
    mat2 FP_hat = fp[pid];
    
    // SVD of Elastic Force
    mat2 U; vec2 Sigma; mat2 V;
    svd(FE_hat, U, Sigma, V);
    vec2 SigmaHat = Sigma;

    // Clamping Sigma
    float theta_0 = 1.0 - THETA_C, theta_1 = 1.0 + THETA_S;
    Sigma = clamp(Sigma, vec2(theta_0), vec2(theta_1));
    Sigma = max(Sigma, vec2(1e-6));

    // Updating F_E
    fe[pid] = U * mat2(Sigma.x, 0.0, 0.0, Sigma.y) * transpose(V);

    // Updating F_P
    vec2 SigmaRatio = SigmaHat / Sigma;
    fp[pid] = V * mat2(SigmaRatio.x, 0.0, 0.0, SigmaRatio.y) * transpose(V) * FP_hat;
}
#version 430
// #include common.glsl
layout(local_size_x = 64) in;

layout(std430, binding = 0) coherent buffer Pos { vec4 pos[]; };
layout(std430, binding = 1) coherent buffer Vel { vec4 vel[]; };
layout(std430, binding = 4) coherent buffer Fe { mat4 fe[]; };
layout(std430, binding = 5) coherent buffer Fp { mat4 fp[]; };
layout(std430, binding = 6) coherent buffer Bp { mat4 bp[]; };

layout(std430, binding = 11) readonly buffer GVelCol { vec4 gVelCol[]; };
layout(std430, binding = 12) readonly buffer GVelFric { vec4 gVelFric[]; };

uniform int numParticles;
uniform int gWidth;
uniform int gHeight;
uniform int gDepth;
uniform float DELTA_TIME;
uniform float THETA_C;
uniform float THETA_S;

void main()
{
    
    uint pid = gl_GlobalInvocationID.x;
    if (pid >= uint(numParticles)) return;

    /* Grid to Particles */
    vec3 pPos = pos[pid].xyz;
    int gx = int(pPos.x), gy = int(pPos.y), gz = int(pPos.z);
    vec3 old_pos = pPos;

    vec3 new_vel = vec3(0.0);
    mat3 new_Bp = mat3(0.0);
    vec3 new_pos = vec3(0.0);

    for (int i = gx - 1; i <= gx + 2; i++)
    {
        for (int j = gy - 1; j <= gy + 2; j++)
        {
            for (int k = gz - 1; k <= gz + 2; k++)
            {
                // Check if the interpolated cell in bounds
                if (i < 0 || i > gWidth || j < 0 || j > gHeight || k < 0 || k > gDepth) continue;

                // Get the corresponding cell
                int cid = k * (gWidth + 1) * (gHeight + 1) + j * (gWidth + 1) + i; 
                vec3 gi = vec3(float(i), float(j), float(k));
                
                // Distance between particle and cell
                vec3 dist = old_pos - gi;

                // Weights
                float weight = computeWeight(dist);

                // Update Velocity
                vec3 vFric = gVelFric[cid].xyz, vCol = gVelCol[cid].xyz;
                new_vel += vFric * weight;

                // Update APIC
                new_Bp += weight * outerProduct(vFric, -dist);

                // Update position
                new_pos += (DELTA_TIME * vCol + gi) * weight;
            }
        }
    }

    vel[pid] = vec4(new_vel, 0.0f);
    pos[pid] = vec4(new_pos, 0.0f);
    bp[pid] = mat4(new_Bp);

    /* Update Deformation Gradient using APIC Bp */
    mat3 C = 3.0 * new_Bp;
    mat3 pFe = mat3(fe[pid]);
    mat3 FE_hat = (mat3(1.0) + DELTA_TIME * C) * pFe;
    mat3 FP_hat = mat3(fp[pid]);
    
    // SVD of Elastic Force
    mat3 U; vec3 Sigma; mat3 V;
    svd(FE_hat, U, Sigma, V);
    vec3 SigmaHat = Sigma;

    // Clamping Sigma
    float theta_0 = 1.0 - THETA_C, theta_1 = 1.0 + THETA_S;
    Sigma = clamp(Sigma, vec3(theta_0), vec3(theta_1));
    Sigma = max(Sigma, vec3(1e-6));

    // Updating F_E
    mat3 fe_result = U * mat3(
        Sigma.x, 0.0, 0.0,  
        0.0, Sigma.y, 0.0,
        0.0, 0.0, Sigma.z) * transpose(V);

    fe[pid] = mat4(fe_result);

    // Updating F_P
    vec3 SigmaRatio = SigmaHat / Sigma;
    mat3 fp_result = V * mat3(
        SigmaRatio.x, 0.0, 0.0, 
        0.0, SigmaRatio.y, 0.0,
        0.0, 0.0, SigmaRatio.z) * transpose(V) * FP_hat;

    fp[pid] = mat4(fp_result);
}
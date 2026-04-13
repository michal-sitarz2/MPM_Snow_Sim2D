#version 430
// #include common.glsl
layout(local_size_x = 64) in;

layout(std430, binding = 3) readonly  buffer Vol { float vol[]; };
layout(std430, binding = 4) coherent  buffer Fe { mat2 fe[]; };
layout(std430, binding = 5) readonly  buffer Fp { mat2 fp[]; };
layout(std430, binding = 6) writeonly buffer Ap { mat2 ap[]; };

uniform int numParticles;

uniform float XI;
uniform float MU;
uniform float LAMBDA;

void main()
{
    uint id = gl_GlobalInvocationID.x;
    if (id >= uint(numParticles)) return;
    if (vol[id] == 0.0) 
    { 
        ap[id] = mat2(0.0); 
        return; 
    }

    mat2 Fe = fe[id];
    mat2 Fp = fp[id];

    float J_e = determinant(Fe);
    float J_p = determinant(Fp);

    /* SVD Polar decomposition */
    mat2 U, V; vec2 Sigma;
    svd(Fe, U, Sigma, V);
    mat2 R = polarR(U, V);

    /* Exponential hardening */
    float harden = exp(XI * (1.0 - J_p));
    float lambda = LAMBDA * harden;
    float mu = MU * harden;

    /* Elastic force differential */
    mat2 dF = 2.0 * mu * (Fe - R) * transpose(Fe) + lambda * (J_e - 1.0) * J_e * mat2(1.0);
    ap[id] = dF * vol[id];
}
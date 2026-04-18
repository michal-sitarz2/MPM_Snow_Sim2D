/* 
============================================
          3D MPM Common Functions
============================================ 
*/

/* Symmetric Schur Decomposition */
void schur_decomp(float a_pp, float a_pq, float a_qq, out float c, out float s)
{
    if (abs(a_pq) < 1e-10)
    {
        c = 1.0;
        s = 0.0;
        return;
    }
    float tau = (a_qq - a_pp) / (2.0 * a_pq);
    float stt = sqrt(1.0 + tau * tau);
    float tan_val = 1.0 / (abs(tau) + stt);
    if (tau < 0.0) tan_val = -tan_val;
    c = 1.0 / sqrt(1.0 + tan_val * tan_val);
    s = tan_val * c;
}

/* Jacobi Rotation to Columns */
void jacobi_rot(inout mat3 A, int p, int q, float c, float s)
{
    vec3 col_p = A[p];
    vec3 col_q = A[q];
    A[p] = c * col_p - s * col_q;
    A[q] = s * col_p + c * col_q;
}

/* Jacobi Rotation to Rows of Symmetric Matrix */
void jacobi_rot_sym(inout mat3 S, int p, int q, float c, float s)
{
    // Rotate S = G^T * S * G where G is Givens rotation in (p,q) plane
    float Spp = S[p][p], Sqq = S[q][q], Spq = S[p][q];

    S[p][p] = c*c*Spp - 2.0*s*c*Spq + s*s*Sqq;
    S[q][q] = s*s*Spp + 2.0*s*c*Spq + c*c*Sqq;
    S[p][q] = 0.0;
    S[q][p] = 0.0;

    // Off-diagonal elements involving p and q
    for (int i = 0; i < 3; i++)
    {
        if (i == p || i == q) continue;
        float Sip = S[i][p], Siq = S[i][q];
        S[i][p] = c * Sip - s * Siq;
        S[p][i] = S[i][p];
        S[i][q] = s * Sip + c * Siq;
        S[q][i] = S[i][q];
    }
}

/* 3x3 SVD with Jacobi Iteration (McAdams et al. 2011) */
void svd(mat3 M, out mat3 U, out vec3 Sigma, out mat3 V)
{
    // Step 1: Compute S = M^T * M
    mat3 S = transpose(M) * M;
    V = mat3(1.0);

    // Step 2: Jacobi iteration for eigendecomposition
    for (int sweep = 0; sweep < 6; sweep++) // 6 for safety
    {
        float c, s;

        /* Rotation for (0,1) */
        schur_decomp(S[0][0], S[0][1], S[1][1], c, s);
        jacobi_rot_sym(S, 0, 1, c, s);
        jacobi_rot(V, 0, 1, c, s);

        /* Rotation for (0,2) */
        schur_decomp(S[0][0], S[0][2], S[2][2], c, s);
        jacobi_rot_sym(S, 0, 2, c, s);
        jacobi_rot(V, 0, 2, c, s);

        /* Rotation for (1,2) */
        schur_decomp(S[1][1], S[1][2], S[2][2], c, s);
        jacobi_rot_sym(S, 1, 2, c, s);
        jacobi_rot(V, 1, 2, c, s);
    }

    // Step 3: Singular values
    Sigma = vec3(
        sqrt(max(S[0][0], 1e-12)),
        sqrt(max(S[1][1], 1e-12)),
        sqrt(max(S[2][2], 1e-12))
    );

    // Step 4: U = M * V * diag(1/Sigma)
    mat3 SigmaInv = mat3(
        1.0/Sigma.x, 0, 0,
        0, 1.0/Sigma.y, 0,
        0, 0, 1.0/Sigma.z
    );
    U = M * V * SigmaInv;

    // Step 5: Fix reflection (det(U) > 0)
    if (determinant(U) < 0.0)
    {
        U[2] = -U[2];
        Sigma.z = -Sigma.z;
    }
}

/* Polar Decomposition Rotation Matrix */
mat3 polarR(mat3 U, mat3 V) { return U * transpose(V); }

/* Cubic B-Spline Grid Weights */
float N(float x)
{
    float x_abs = abs(x);
    if (x_abs < 1.0)
    {
        return (0.5 * pow(x_abs, 3.0)) - pow(x_abs, 2.0) + (2.0 / 3.0);
    }
    else if (x_abs < 2.0)
    {
        return pow(2.0 - x_abs, 3.0) / 6.0;
    }
    return 0.0;
}

float N_grad(float x)
{
    float x_abs = abs(x);
    if (x_abs < 1.0)
    {
        return (1.5 * x_abs * x) - (2.0 * x);
    }
    else if (x_abs < 2.0)
    {
        return (-0.5 * x_abs * x) + (2.0 * x) - ((2.0 * x) / x_abs);
    }
    return 0.0;
}

float computeWeight(vec3 dist)
{
    return N(dist.x) * N(dist.y) * N(dist.z);
}

vec3 computeGradWeight(vec3 dist)
{
    return vec3(
        N_grad(dist.x) * N(dist.y)      * N(dist.z),
        N(dist.x)      * N_grad(dist.y) * N(dist.z),
        N(dist.x)      * N(dist.y)      * N_grad(dist.z)
    );
}
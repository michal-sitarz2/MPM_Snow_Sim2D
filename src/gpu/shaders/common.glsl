/* Analytical SVD for 2x2 matrices */
void svd(mat2 M, out mat2 U, out vec2 Sigma, out mat2 V)
{
    // Step 1: M.T * M
    mat2 MtM = transpose(M) * M;

    // Step 2: Eigendecomposition (column-major matrices)
    float a = MtM[0][0];
    float b = MtM[1][0];
    float d = MtM[1][1];

    // Guard against degenerate atan when b=0 and a=d
    float theta = (abs(b) < 1e-10 && abs(a - d) < 1e-10)
                  ? 0.0
                  : 0.5 * atan(2.0 * b, a - d);

    float ct = cos(theta);
    float st = sin(theta);

    // V: eigenvector of matrix M.T * M
    V = mat2(ct, st, -st, ct);

    // Step 3: Singular values
    mat2 diag = transpose(V) * MtM * V;
    Sigma = sqrt(max(vec2(diag[0][0], diag[1][1]), vec2(1e-12)));

    // Step 4: U = M * V * diag(1 / Sigma)
    mat2 SigmaInv = mat2(1.0 / Sigma.x, 0.0, 0.0, 1.0 / Sigma.y);
    U = M * V * SigmaInv;

    // Step 5: Fix sign (singular values should be positive)
    if (determinant(U) < 0.0)
    {
        U[1]    = -U[1];
        Sigma.y = -Sigma.y;
    }
}

/* Polar Decomposition Rotation Matrix */
mat2 polarR(mat2 U, mat2 V) { return U * transpose(V); }

/* Grid Weights */
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

float computeWeight(vec2 dist)
{
    return N(dist.x) * N(dist.y);
}

vec2 computeGradWeight(vec2 dist)
{
    return vec2(
        N_grad(dist.x) * N(dist.y),
        N(dist.x) * N_grad(dist.y)
    );
}
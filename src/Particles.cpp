#include "Particles.h"

#include <stdexcept>
#include <Eigen/Dense>
#include "helpers.h"

Particle::Particle(glm::vec2 inPos, glm::vec2 inVel, float inVol)
    : pos(inPos)
    , vel(inVel)
    , volInit(inVol)
{ 
    A_p = glm::mat2(0.f);
    B_p = glm::mat2(0.f);
    F_Elastic = glm::mat2(1.f);
    F_Plastic = glm::mat2(1.f);

    // TODO:
    mass = (volInit * RHO) / 100.f;
}

void Particle::PreComputeForces()
{
    if (volInit == 0.f) return;

    float J_e = GetElasticDet();
    float J_p = GetPlasticDet();

    /* Polar Decomposition of Elastic Force via SVD */
    Eigen::Matrix2f F_eig = GLMToEigen(F_Elastic);
    
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(F_eig, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2f R_eig = svd.matrixU() * svd.matrixV().transpose();

    glm::mat2 F = F_Elastic;
    glm::mat2 R = EigenToGLM(R_eig);

    // Exponential Hardening (TODO?)
    float harden = std::exp(XI * (1.0f - J_p));
    float lambda = LAMBDA * harden;
    float mu = MU * harden;

    // TODO: FinvT?

    glm::mat2 dF_Elastic =
        2.f * mu * (F - R) * glm::transpose(F) + lambda * (J_e - 1.f) * J_e * glm::mat2(1.f);

    // APIC
    A_p = dF_Elastic * volInit;

    if (std::isnan(harden) || std::isnan(A_p[0][0]) || std::isnan(A_p[0][1]) || std::isnan(A_p[1][0]) || std::isnan(A_p[1][1]))
    {
        std::cout << "Elastic: " << GetElasticForce()[0][0] << ", " << GetElasticForce()[0][1] << ", " << GetElasticForce()[1][0] << ", " << GetElasticForce()[1][1] << ", " << std::endl;
        std::cout << "J_e: " << J_e << std::endl;

        std::cout << "Plastic: " << GetPlasticForce()[0][0] << ", " << GetPlasticForce()[0][1] << ", " << GetPlasticForce()[1][0] << ", " << GetPlasticForce()[1][1] << ", " << std::endl;
        std::cout << "J_p: " << J_p << std::endl;

        std::cout << "Harden: " << harden << std::endl;
        
        while (true) {}
    }
}

void Particle::UpdateDeformationGrad(glm::mat2 gradVel)
{
    // Temporary definitions
    glm::mat2 FE_hat = (glm::mat2(1.0f) + (DELTA_TIME * gradVel)) * GetElasticForce();
    glm::mat2 FP_hat = GetPlasticForce();

    // SVD of Elastic Force
    Eigen::Matrix2f ElasticEig = GLMToEigen(FE_hat);

    Eigen::JacobiSVD<Eigen::Matrix2f> ElasticSVD(ElasticEig, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2f U = ElasticSVD.matrixU();
    Eigen::Matrix2f V = ElasticSVD.matrixV();
    Eigen::Vector2f SigmaHat = ElasticSVD.singularValues();

    // Fixing Orientations
    /*if (U.determinant() < 0) {
        U.col(1) *= -1;
        SigmaHat(1) *= -1;
    }

    if (V.determinant() < 0) {
        V.col(1) *= -1;
    }*/

    // Clamping Sigma
    float theta_0 = (1.0 - THETA_C), theta_1 = (1.0 + THETA_S);
    auto clamp = [theta_0, theta_1](float x) -> float {
        return std::max(theta_0, std::min(x, theta_1));
    };

    Eigen::Vector2f Sigma = SigmaHat.unaryExpr(clamp);
    Sigma = Sigma.cwiseMax(1e-6f);

    // Updating F_E
    Eigen::Matrix2f F_E = U * Sigma.asDiagonal() * V.transpose();
    SetElastic(EigenToGLM(F_E));
    
    // Updating F_P
    Eigen::Matrix2f SigmaRatio = (SigmaHat.cwiseQuotient(Sigma)).asDiagonal();
    Eigen::Matrix2f F_P = V * SigmaRatio * V.transpose() * GLMToEigen(FP_hat);
    SetPlastic(EigenToGLM(F_P));
}

Particles::Particles(Grid& inGrid) : grid(inGrid)
{
    std::cout << "[DEBUG] Setting up the particle shapes" << std::endl;
    for (SHAPE* shape : SHAPES)
        Init(*shape);
}

void Particles::Solver()
{
    /* Step 1: Rasterize Particle Data to Grid - Mass and Velocity */
    grid.RasterizeToGrid(particles);

    /* Steps 3-6: Compute Grid Forces, Velocities, Collisions */
    grid.ComputeGridForces();

    /* Step 8: Interpolate Grid Velocities to Particles */
    InterpolateVelocities();

    /* Steps 7 & 10: Update Deformation Gradient and Position */
    Update();

    /* Reset the Grid */
    grid.Reset();
}

void Particles::InterpolateVelocities()
{
    for (Particle& particle : particles)
    {
        auto pPos = particle.GetPosition();
        int gPosX = static_cast<int>(pPos.x);
        int gPosY = static_cast<int>(pPos.y);

        particle.ResetBp();
        particle.ResetVelocity();

        for (int i = gPosX - 1; i <= gPosX + 2; i++)
        {
            for (int j = gPosY - 1; j <= gPosY + 2; j++)
            {
                // Check if the interpolated cell in bounds
                if (!grid.InBounds(i, j)) continue;

                // Get the corresponding cell
                Cell& cell = grid.GetCellAt(i, j);

                // Distance between particle and cell
                glm::vec2 g_i = glm::vec2(i, j);
                glm::vec2 dist = pPos - g_i;

                // TODO: Debugging

                auto vel1 = cell.GetVelocity();
                auto vel2 = cell.GetVelocityCollision();
                auto vel3 = cell.GetVelocityFriction();

                // Weight
                float weight = Grid::ComputeWeight(dist);

                // Update Velocity
                glm::vec2 velocity = cell.GetVelocityFriction();
                particle.AddVelocity(velocity * weight);

                // Update APIC
                glm::mat2 Bp = weight * VectorOuterProduct(velocity, -dist);
                particle.AddVelocityField(Bp);
            }
        }
    }
}

void Particles::Update()
{
    for (Particle& particle : particles)
    {
        auto pPos = particle.GetPosition();
        int gPosX = static_cast<int>(pPos.x);
        int gPosY = static_cast<int>(pPos.y);

        glm::mat2 gradVel(0.0f);

        glm::vec2 old_pos = particle.GetPosition();
        particle.ResetPosition();

        for (int i = gPosX - 1; i <= gPosX + 2; i++)
        {
            for (int j = gPosY - 1; j <= gPosY + 2; j++)
            {
                // Check if the interpolated cell in bounds
                if (!grid.InBounds(i, j)) continue;

                // Get the corresponding cell
                Cell& cell = grid.GetCellAt(i, j);

                // Distance between particle and cell
                glm::vec2 g_i = glm::vec2(i, j);
                glm::vec2 dist = old_pos - g_i;
                
                // Weight gradient
                float weight = Grid::ComputeWeight(dist);
                glm::vec2 dWeight = Grid::ComputeGradWeight(dist);

                // Update Position
                glm::vec2 velocity = cell.GetVelocityCollision();
                glm::vec2 new_pos = DELTA_TIME * velocity + g_i;
                particle.AddPosition(new_pos * weight);

                // Update Deformation
                gradVel += VectorOuterProduct(velocity, dWeight);
            }
        }

        particle.UpdateDeformationGrad(gradVel);
    }
}

void Particles::Init(SHAPE& shape)
{
    auto* snowball = dynamic_cast<SNOWBALL_SHAPE*> (&shape);
    if (!snowball)  
        throw std::runtime_error("[ERROR] Failed to cast to SNOWBALL");

    float minDist = snowball->radius / std::sqrt((float)NUM_PARTICLES);
    snowball->GenerateSamples(minDist);

    for (auto& pos : snowball->diskSamples)
    {
        // TODO:
        float vol = 2 * PI * std::pow(snowball->radius, 2) / (float) snowball->diskSamples.size();
        Particle particle(pos, snowball->vel, vol);
        particles.push_back(particle);
    }
}
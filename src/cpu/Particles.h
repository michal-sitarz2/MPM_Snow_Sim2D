#pragma once

#include "Grid.h"
#include "Parameters.h"

class Particle
{
public:
    Particle(glm::vec2 inPos, glm::vec2 inVel, float inVol);

    void PreComputeForces();

    /* Update */
    void AddPosition(glm::vec2 inPos)     { pos += inPos; }
    void AddVelocity(glm::vec2 inVel)     { vel += inVel; }
    void AddVelocityField(glm::mat2 inBp) { B_p += inBp; }

    void SetElastic(glm::mat2 elastic) { F_Elastic = elastic; };
    void SetPlastic(glm::mat2 plastic) { F_Plastic = plastic; };

    void ResetPosition() { pos = glm::vec2(0.f); }
    void ResetVelocity() { vel = glm::vec2(0.f); }
    void ResetBp()       { B_p = glm::mat2(0.f); }

    void UpdateDeformationGrad(glm::mat2 gradVel);

    /* Getters */
    float GetMass() const             { return mass; }
    float GetVol() const              { return volInit; }
    glm::vec2 GetPosition() const     { return pos; }
    glm::vec2 GetVelocity() const     { return vel; }
    glm::mat2 GetElasticForce() const { return F_Elastic; }
    glm::mat2 GetPlasticForce() const { return F_Plastic; }
    glm::mat2 GetBp() const           { return B_p; }
    glm::mat2 GetAp() const           { return A_p; }
    float GetElasticDet() const       { return glm::determinant(F_Elastic); }
    float GetPlasticDet() const       { return glm::determinant(F_Plastic); }
    float GetDefGradDet() const       { return glm::determinant(F_Elastic * F_Plastic); }
    
private:
    glm::vec2 pos;
    glm::vec2 vel;

    float mass;
    float volInit;

    glm::mat2 A_p;
    glm::mat2 B_p;
    glm::mat2 F_Elastic;
    glm::mat2 F_Plastic;
};

class Particles
{
public:
    Particles(Grid& inGrid);

    void Solver();
    void InterpolateVelocities();

    std::vector<Particle>& GetParticles() { return particles; }

private:
    Grid& grid;
    std::vector<Particle> particles;

    void Init(SHAPE& shape);
    void Update();
};
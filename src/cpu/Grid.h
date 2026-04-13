#pragma once

#include <vector>
#include <iostream>
#include <glm/glm.hpp>

class Particle;

struct BorderLine
{
    BorderLine(glm::vec2 inP1, glm::vec2 inP2, glm::vec2 inNormal);

    glm::vec2 p1;
    glm::vec2 p2;
    glm::vec2 normal;
};

class Cell
{
public:
    Cell(glm::vec2 inLoc);

    void Reset();
    
    /* Cell Operations */
    void DetectCollisions(std::vector<BorderLine>& border);

    /* Getters */
    float GetMass()                  const { return mass; }
    glm::vec2 GetLocation()          const { return loc; }
    glm::vec2 GetVelocity()          const { return vel; }
    glm::vec2 GetForce()             const { return force; }
    glm::vec2 GetVelocityCollision() const { return vel_collision; }
    glm::vec2 GetVelocityFriction()  const { return vel_friction; }

    /* Update Cell */
    void SetForce(glm::vec2 inForce)  { force = inForce; }
    
    void AddVelocity(glm::vec2 inVel) { vel += inVel; }
    void AddMass(float inMass)        { mass += inMass; }
    void AddForce(glm::vec2 inForce)  { force += inForce; }

    void NormalizeVelocity() { vel /= mass; } // Normalizes the velocity by the mass

private:
    float mass;      // Mass
    glm::vec2 loc;   // Location
    glm::vec2 vel;   // Velocity
    glm::vec2 force; // Force

    glm::vec2 vel_collision; // Collision Velocity
    glm::vec2 vel_friction;  // Friction Velocity

};

class Grid
{
public:
    Grid(int inWidth, int inHeight, int inBoundMargin);

    /* Grid Functions */
    void RasterizeToGrid(std::vector<Particle>& particles);
    void ComputeGridForces();
    void Reset();

    /* Getters */
    std::vector<Cell>& GetCells()        { return cells; }
    std::vector<BorderLine>& GetBorder() { return borderLines; }

    /* Helpers */
    bool InBounds(int x, int y);
    int CellIndex(int x, int y);
    Cell& GetCellAt(int i, int j) { return cells[CellIndex(i, j)]; }

    /* Compute Weights */
    static float ComputeWeight(glm::vec2 dist) { return N(dist.x) * N(dist.y); }
    static glm::vec2 ComputeGradWeight(glm::vec2 dist) 
    {
        return glm::vec2{
            N_grad(dist.x) * N(dist.y),
            N(dist.x) * N_grad(dist.y)
        };
    }

    /* Cubic B-spline */
    static float N(const float x)
    {
        float x_abs = std::fabs(x);

        float N = 0.f;
        if (x_abs < 1)
        {
            N = (0.5f * std::pow(x_abs, 3)) - (std::pow(x_abs, 2)) + (2.f / 3.f);
        }
        else if (x_abs >= 1 && x_abs < 2)
        {
            N = std::pow(2 - x_abs, 3) / 6.f;
        }
        return N;
    }

    static float N_grad(const float x)
    {
        float x_abs = std::fabs(x);

        float dN = 0.f;
        if (x_abs < 1)
        {
            dN = (1.5f * x_abs * x) - (2.0f * x);
        }
        else if (x_abs >= 1 && x_abs < 2)
        {
            dN = (-0.5f * x_abs * x) + (2.f * x) - ((2.f * x) / x_abs);
        }
        return dN;
    }

private:
    int width;
    int height;
    int margin;

    std::vector<Cell> cells;
    std::vector<BorderLine> borderLines;
};

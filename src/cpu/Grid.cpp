#include "Grid.h"
#include "Particles.h"

BorderLine::BorderLine(glm::vec2 inP1, glm::vec2 inP2, glm::vec2 inNormal)
    : p1(inP1)
    , p2(inP2)
    , normal(inNormal) { }

Cell::Cell(glm::vec2 inLoc) : loc(inLoc)
{
    Reset();   
}

// Inspired by: https://github.com/Elias-Gu/MPM2D, for separators
void Cell::DetectCollisions(std::vector<BorderLine>& border)
{
    /* Border Collisions */
    vel_collision = vel;

    std::vector<BorderLine> collisions;
    for (BorderLine& line : border)
    {
        // Signed Distance to Boundary
        glm::vec2 diff = loc - line.p1;
        float dist = glm::dot(line.normal, diff);

        // Test Distance after Moving (predicts future position)
        glm::vec2 test_diff = (loc + DELTA_TIME * vel_collision) - line.p1;
        float test_dist = glm::dot(line.normal, test_diff);

        // Net change in penetration during the timestep
        float delta_dist = test_dist - std::min(dist, 0.f);

        if (delta_dist < 0.f)
        {
            // Velocity correction in the direction of the normal
            vel_collision -= (delta_dist * line.normal) / DELTA_TIME;

            // Save collisions 
            collisions.push_back(line);
        }
    }

    /* Friction for Tangential Velocity */
    vel_friction = vel_collision;
    for (BorderLine& line : collisions)
    {
        // Tangential Velocity
        glm::vec2 vt = vel_collision - line.normal * glm::dot(line.normal, vel_friction);

        float vt_norm = glm::length(vt);
        if (vt_norm <= 1e-7) continue;
        glm::vec2 time = vt / vt_norm;

        // Coulomb's friction
        glm::vec2 diff_col = vel_collision - vel;
        vel_friction -= std::min(vt_norm, FRICTION * glm::length(diff_col)) * time;
    }
}

void Cell::Reset()
{
    mass = 0.f;
    vel = glm::vec2(0.f);
    force = glm::vec2(0.f);

    vel_collision = glm::vec2(0.f);
    vel_friction = glm::vec2(0.f);
}

Grid::Grid(int inWidth, int inHeight, int inBoundMargin) 
    : width(inWidth)
    , height(inHeight)
    , margin(inBoundMargin)
{
    std::cout << "Setting up the grid" << std::endl;
    
    /* Defining the grid of cells */
    for (int y = 0; y <= height; y++)
    {
        for (int x = 0; x <= width; x++)
        {
            Cell cell(glm::vec2((float)x, (float)y));
            cells.push_back(cell);
        }
    }

    BorderLine borderRight(
        glm::vec2((float)width - margin, (float)margin),
        glm::vec2((float)width - margin, (float)height - margin),
        glm::vec2(-1.f, 0.f)
    );
    borderLines.push_back(borderRight); // Right Border

    /* Defining the border */
    BorderLine borderBottom(
        glm::vec2((float)margin, (float)margin),
        glm::vec2((float)width - margin, (float)margin),
        glm::vec2(0.f, 1.f)
    );
    borderLines.push_back(borderBottom); // Bottom Border

    BorderLine borderLeft(
        glm::vec2((float)margin, (float)height - margin),
        glm::vec2((float)margin, (float)margin),
        glm::vec2(1.f, 0.f)
    );
    borderLines.push_back(borderLeft); // Left Border

    BorderLine borderTop(
        glm::vec2((float)width - margin, (float)height - margin),
        glm::vec2((float)margin, (float)height - margin),
        glm::vec2(0.f, -1.f)
    );
    borderLines.push_back(borderTop); // Top Border
}

void Grid::RasterizeToGrid(std::vector<Particle>& particles)
{
    for (Particle& particle : particles)
    {
        auto pPos = particle.GetPosition();
        int gPosX = static_cast<int>(pPos.x);
        int gPosY = static_cast<int>(pPos.y);

        particle.PreComputeForces();

        auto pMass = particle.GetMass();
        auto pVel = particle.GetVelocity();
        auto B_p = particle.GetBp();
        
        for (int i = gPosX - 1; i <= gPosX + 2; i++)
        {
            for (int j = gPosY - 1; j <= gPosY + 2; j++)
            {
                // Check if the interpolated cell in bounds
                if (!InBounds(i, j)) continue;

                // Cell location
                Cell& cell = cells[CellIndex(i, j)];

                // Distance
                glm::vec2 g_i = glm::vec2(i, j);
                glm::vec2 dist = pPos - g_i;

                // Weight and Weight Gradient
                float weight = ComputeWeight(dist);
                glm::vec2 dWeight = ComputeGradWeight(dist);

                // Compute Mass, Velocity, Force for the cell
                float mass = weight * pMass;
                glm::vec2 apic_v = pVel + 3.f * B_p * (-dist);
                glm::vec2 velocity = mass * apic_v;
                glm::vec2 force = dWeight * particle.GetAp();

                // Update Grid Cells
                cell.AddMass(mass);
                cell.AddVelocity(velocity);
                cell.AddForce(force);
            }
        }
    }
}

void Grid::ComputeGridForces()
{
    for (auto& cell : cells)
    {
        if (cell.GetMass() < 1e-6f) continue;

        // Normalize the velocities
        cell.NormalizeVelocity();

        // Update forces
        glm::vec2 force = glm::vec2(0, GRAVITY) + (-cell.GetForce() / cell.GetMass());
        cell.SetForce(force * DELTA_TIME);

        // Add Force to Velocities
        cell.AddVelocity(cell.GetForce());

        // Collision and Friction
        cell.DetectCollisions(borderLines);
    }
}

void Grid::Reset()
{
    for (auto& cell : cells)
        cell.Reset();
}

bool Grid::InBounds(int x, int y)
{
    return x >= 0 && x <= width && y >= 0 && y <= height;
}

int Grid::CellIndex(int x, int y)
{
    return y * (width + 1) + x;
}


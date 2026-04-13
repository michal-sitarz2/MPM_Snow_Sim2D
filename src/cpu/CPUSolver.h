#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "Parameters.h"
#include "Grid.h"
#include "Particles.h"

class CPUSolver {
public:
    CPUSolver()
        : grid(WIDTH, HEIGHT, BORDER_MARGIN)
        , particles(grid)
    {}

    /* Solver Update Step */
    void Step() { particles.Solver(); }

    /* Getters */
    const std::vector<glm::vec2>& GetParticlePositions()
    {
        cachedPositions.clear();
        for (auto& p : particles.GetParticles())
            cachedPositions.push_back(p.GetPosition());
        return cachedPositions;
    }

private:
    Grid grid;
    Particles particles;
    std::vector<glm::vec2> cachedPositions;
};
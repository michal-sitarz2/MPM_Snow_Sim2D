#pragma once

#include <vector>
#include <random>
#include <cmath>
#include <glm/glm.hpp>

static const float PI = 3.1415926f;

enum SHAPE_TYPE
{
	SNOWBALL
};


#ifdef SIM_3D
inline std::vector<glm::vec3> PoissonDiskSphere(
    glm::vec3 center,
    float     sphereRadius,
    float     minDist,
    int       maxAttempts = 30)
{
    std::mt19937 gen(std::random_device{}());
    auto randF = [&](float lo, float hi) {
        return std::uniform_real_distribution<float>(lo, hi)(gen);
    };

    const float cellSize = minDist / std::sqrt(3.0f);
    const int   gridW = (int)std::ceil(2.0f * sphereRadius / cellSize) + 2;
    const int   gridH = gridW;
    const int   gridD = gridW;

    std::vector<int>       bgGrid(gridW * gridH * gridD, -1);
    std::vector<glm::vec3> samples;
    std::vector<glm::vec3> active;

    glm::vec3 origin = center - glm::vec3(sphereRadius);

    auto toCell = [&](glm::vec3 p) -> std::tuple<int, int, int> {
        return {
            (int)((p.x - origin.x) / cellSize),
            (int)((p.y - origin.y) / cellSize),
            (int)((p.z - origin.z) / cellSize)
        };
    };

    auto addSample = [&](glm::vec3 p) {
        auto [gx, gy, gz] = toCell(p);
        bgGrid[gz * gridW * gridH + gy * gridW + gx] = (int)samples.size();
        samples.push_back(p);
        active.push_back(p);
    };

    // Seed with a random point inside the sphere
    {
        float theta = randF(0.0f, 2.0f * PI);
        float phi = std::acos(randF(-1.0f, 1.0f));
        float r = std::cbrt(randF(0.0f, sphereRadius * sphereRadius * sphereRadius));
        addSample(center + glm::vec3(
            r * std::sin(phi) * std::cos(theta),
            r * std::sin(phi) * std::sin(theta),
            r * std::cos(phi)
        ));
    }

    while (!active.empty())
    {
        std::uniform_int_distribution<int> dist(0, (int)active.size() - 1);
        int ai = dist(gen);
        glm::vec3 point = active[ai];
        bool found = false;

        for (int attempt = 0; attempt < maxAttempts; attempt++)
        {
            // Random direction on sphere
            float theta = randF(0.0f, 2.0f * PI);
            float phi = std::acos(randF(-1.0f, 1.0f));
            float r1 = randF(0.0f, 1.0f);
            float d = minDist * (r1 + 1.0f);
            glm::vec3 candidate = point + glm::vec3(
                d * std::sin(phi) * std::cos(theta),
                d * std::sin(phi) * std::sin(theta),
                d * std::cos(phi)
            );

            if (glm::length(candidate - center) > sphereRadius) continue;

            auto [cgx, cgy, cgz] = toCell(candidate);
            if (cgx < 0 || cgy < 0 || cgz < 0 ||
                cgx >= gridW || cgy >= gridH || cgz >= gridD) continue;

            bool tooClose = false;
            for (int dx = -2; dx <= 2 && !tooClose; dx++)
                for (int dy = -2; dy <= 2 && !tooClose; dy++)
                    for (int dz = -2; dz <= 2 && !tooClose; dz++)
                    {
                        int nx = cgx + dx, ny = cgy + dy, nz = cgz + dz;
                        if (nx < 0 || ny < 0 || nz < 0 ||
                            nx >= gridW || ny >= gridH || nz >= gridD) continue;
                        int ni = bgGrid[nz * gridW * gridH + ny * gridW + nx];
                        if (ni >= 0 && glm::length(samples[ni] - candidate) < minDist)
                            tooClose = true;
                    }

            if (!tooClose)
            {
                addSample(candidate);
                found = true;
                break;
            }
        }

        if (!found)
            active.erase(active.begin() + ai);
    }

    return samples;
}

struct SHAPE_3D
{
    glm::vec3 loc;
    glm::vec3 vel;

    SHAPE_3D(glm::vec3 loc, glm::vec3 vel) : loc(loc), vel(vel) {}
    virtual ~SHAPE_3D() = default;
    virtual void GenerateSamples(float minDist) = 0;
};

struct SNOWBALL_SHAPE_3D : public SHAPE_3D
{
    float radius;
    std::vector<glm::vec3> sphereSamples;

    SNOWBALL_SHAPE_3D(glm::vec3 loc, glm::vec3 vel, float radius)
        : SHAPE_3D(loc, vel), radius(radius) {}

    void GenerateSamples(float minDist) override
    {
        sphereSamples = PoissonDiskSphere(loc, radius, minDist);
    }
};
#else
inline std::vector<glm::vec2> PoissonDiskCircle( // Bridson's algorithm (2007) -> near-maximal Poisson disk distribution
    glm::vec2 center,
    float     circleRadius,
    float     minDist,
    int       maxAttempts = 30)
{
    std::mt19937 gen(std::random_device{}());
    auto randF = [&](float lo, float hi) {
        return std::uniform_real_distribution<float>(lo, hi)(gen);
        };

    const float cellSize = minDist / std::sqrt(2.0f);
    const int   gridW = (int)std::ceil(2.0f * circleRadius / cellSize) + 2;
    const int   gridH = gridW;

    std::vector<int>       bgGrid(gridW * gridH, -1);
    std::vector<glm::vec2> samples;
    std::vector<glm::vec2> active;

    // World origin of the background grid (top-left corner)
    glm::vec2 origin = center - glm::vec2(circleRadius);

    auto toCell = [&](glm::vec2 p) -> std::pair<int, int> {
        return {
            (int)((p.x - origin.x) / cellSize),
            (int)((p.y - origin.y) / cellSize)
        };
    };

    auto addSample = [&](glm::vec2 p) {
        std::pair<int, int> cell = toCell(p);
        int gx = cell.first;
        int gy = cell.second;
        bgGrid[gy * gridW + gx] = (int)samples.size();
        samples.push_back(p);
        active.push_back(p);
    };

    // Seed with a random point inside the circle
    {
        float a = randF(0.0f, 2.0f * PI);
        float r = std::sqrt(randF(0.0f, circleRadius * circleRadius));
        addSample(center + glm::vec2(r * std::cos(a), r * std::sin(a)));
    }

    while (!active.empty())
    {
        std::uniform_int_distribution<int> dist(0, active.size() - 1);
        int ai = dist(gen);
        glm::vec2 point = active[ai];
        bool      found = false;

        for (int attempt = 0; attempt < maxAttempts; attempt++)
        {
            float a = randF(0.0f, 2.0f * PI);
            float r1 = randF(0.0f, 1.0f);
            float dist = minDist * (r1 + 1.0f);
            glm::vec2 candidate = point + glm::vec2(dist * std::cos(a), dist * std::sin(a));

            // Must be inside the circle
            if (glm::length(candidate - center) > circleRadius) continue;

            std::pair<int, int> cell = toCell(candidate);
            int cgx = cell.first;
            int cgy = cell.second;
            if (cgx < 0 || cgy < 0 || cgx >= gridW || cgy >= gridH) continue;

            bool tooClose = false;
            for (int dx = -2; dx <= 2 && !tooClose; dx++)
                for (int dy = -2; dy <= 2 && !tooClose; dy++)
                {
                    int nx = cgx + dx, ny = cgy + dy;
                    if (nx < 0 || ny < 0 || nx >= gridW || ny >= gridH) continue;
                    int ni = bgGrid[ny * gridW + nx];
                    if (ni >= 0 && glm::length(samples[ni] - candidate) < minDist)
                        tooClose = true;
                }

            if (!tooClose)
            {
                addSample(candidate);
                found = true;
                break;
            }
        }

        if (!found)
            active.erase(active.begin() + ai);
    }

    return samples;
}

struct SHAPE 
{
	glm::vec2 loc;                      // Location of the shape on the screen
    glm::vec2 vel;                      // Starting velocity of the snow shape
    std::vector<glm::vec2> coordinates; // Coordinates of vertices composing the shape
    
    // Default constructor for the shape
    SHAPE(glm::vec2& loc, glm::vec2& vel) : loc(loc), vel(vel) {};
    virtual ~SHAPE() = default;
	
    /* Virtual functions that all the children classes have to include */ 
    virtual void Generate() = 0;
    // virtual float ComputeArea() = 0;
    // virtual float ComputeVolume() = 0;
    virtual void GetRandom(float& x, float& y) = 0;

    static double getRandom(double min, double max)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }
};

struct SNOWBALL_SHAPE : public SHAPE
{
	float radius; // Radius of the circle
    int vertex_count = 180; // Number of vertices defining the circle
	SHAPE_TYPE type = SHAPE_TYPE::SNOWBALL; // Shape type

    std::vector<glm::vec2> diskSamples;

    SNOWBALL_SHAPE(glm::vec2& loc, glm::vec2& vel, float radius) 
        : SHAPE(loc, vel), radius(radius)
    { 
        Generate(); 
    };

    void GenerateSamples(float minDist)
    {
        // enforces minimum distance r between all samples
        diskSamples = PoissonDiskCircle(loc, radius, minDist);
    }

    // TODO: 
    // float ComputeArea() override { return 2.0 * PI * std::pow(radius, 2); }
    // float ComputeVolume() override { return (4.0 / 3.0) * PI * std::pow(radius, 3); }
    
    void GetRandom(float& x, float& y) override
    {
        static std::mt19937 gen(std::random_device{}());
        if (diskSamples.empty()) {  // if PoissonDiskCircle not init
            float a = (float)getRandom(0.0, 2.0 * PI);
            float r = std::sqrt((float)getRandom(0.0, radius * radius));
            x = loc.x + r * std::cos(a);
            y = loc.y + r * std::sin(a);
            return;
        }

        int idx = std::uniform_int_distribution<int>(0, (int)diskSamples.size() - 1)(gen);
        x = diskSamples[idx].x;
        y = diskSamples[idx].y;
    }

    void Generate() override
	{
        float alpha = 360.0 / vertex_count;
        int triangleCount = vertex_count - 2;

        std::vector<glm::vec2> triangles;
        
        for (int segmentIdx = 0; segmentIdx < vertex_count; segmentIdx++)
        {
            float segmentAlpha = (alpha * segmentIdx) * (PI / 180.0);
            float
                x = radius * std::cos(segmentAlpha),
                y = radius * std::sin(segmentAlpha);

            coordinates.push_back(loc + glm::vec2(x,y));
        }
	}
};
#endif
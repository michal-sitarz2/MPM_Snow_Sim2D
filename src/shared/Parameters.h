#pragma once

#include <vector>
#include "Shapes.h"

#define USE_GPU true

#ifdef SIM_3D
    using Vec = glm::vec3;
    using Mat = glm::mat3;
	using VecGPU = glm::vec4; // padding for std430
	using MatGPU = glm::mat4; // padding for std430
    static constexpr int DIM = 3;
#else
    using Vec = glm::vec2;
    using Mat = glm::mat2;
	using VecGPU = glm::vec2;
    using MatGPU = glm::mat2;
    static constexpr int DIM = 2;
#endif


/*
Experimental Parameter values inspired by:
	-> https://github.com/Azmisov/snow
	-> https://github.com/Elias-Gu/MPM2D
*/

/* Grid */
static const int
	WIDTH = 200,		// grid width
	HEIGHT = 100,		// grid height
	BORDER_MARGIN = 2;  // border margin on the edges
	
#ifdef SIM_3D
static const int DEPTH = 100;  // grid depth 
#endif

/* Shape Generation */
static const int NUM_PARTICLES = 3000;

/* Snowballs Crushing */
#ifdef SIM_3D
static std::vector<SHAPE_3D*> SHAPES = {
    new SNOWBALL_SHAPE_3D(glm::vec3(50, 45, 50), glm::vec3(40, 0, 0), 15),
    new SNOWBALL_SHAPE_3D(glm::vec3(150, 55, 50), glm::vec3(-40, 0, 0), 15),
};
#else
static std::vector<SHAPE*> SHAPES = {
    new SNOWBALL_SHAPE(glm::vec2(50, 45), glm::vec2(40, 0), 15),
    new SNOWBALL_SHAPE(glm::vec2(150, 55), glm::vec2(-40, 0), 15),
};
#endif

/* Simulation */
static const int MAX_FRAMES = 200;

static const float
	GRAVITY = -9.81f,  // Gravity Force
	FRICTION = 0.3f;   // Coulomb's friction coefficient

// static const bool SAVE_FRAMES = true;
static const int
	WINDOW_WIDTH = 1200,
	WINDOW_HEIGHT = 600;

static const float
	DELTA_TIME = 1e-3,
	SAVE_FPS = 30;

/* Snow Parameters */
static const float 
	PARTICLE_SIZE = 1.f,  // Particle Size
	THETA_C = 2.0e-2,     // Critical Compression
	THETA_S = 6.0e-3,     // Critical Stretch
	XI = 10.f,            // Hardening Coefficient
	NU = 2.0e-1,		  // Poisson Ratio
	E = 1.4e5,            // Youngs Modulus
	RHO = 400.f;	      // Typical densities of snow 100 - 400 kg/m^3


static const float
	MU = E / (2.f * (1.f + NU)),						    // LAME coefficient - MU
	LAMBDA = (E * NU) / ((1.f + NU) * (1.f - (2.f * NU)));  // LAME coefficient - LAMBDA
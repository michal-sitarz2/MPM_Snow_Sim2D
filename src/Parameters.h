#pragma once

#include <vector>
#include "Shapes.h"

/*
Experimental Parameter values inspired by:
	-> https://github.com/Azmisov/snow
	-> https://github.com/Elias-Gu/MPM2D
*/

/* Grid */
static const int
	WIDTH = 200,		// window width
	HEIGHT = 100,		// window height
	BORDER_MARGIN = 2;  // border margin on the edges

/* Shape Generation */
static const int NUM_PARTICLES = 3000;

static std::vector<SHAPE*> SHAPES = {
	/* Snowball Drop: */
	// new SNOWBALL_SHAPE(glm::vec2(100,50), glm::vec2(0, 0), 15),

	/* Snowball Crush: */
	new SNOWBALL_SHAPE(glm::vec2(50,45), glm::vec2(40, 0), 15),
	new SNOWBALL_SHAPE(glm::vec2(150,55), glm::vec2(-40, 0), 15),
};

/* Simulation */

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
	PARTICLE_SIZE = 1.f,  // TODO:?
	THETA_C = 2.0e-2,     // Critical Compression
	THETA_S = 6.0e-3,     // Critical Stretch
	XI = 10,              // Hardening Coefficient
	NU = 2.0e-1,		  // Poisson Ratio
	E = 1.4e5,            // Youngs Modulus
	RHO = 400.f;	      // Typical densities of snow 100 - 400 kg/m^3


static const float
	MU = E / (2.f * (1.f + NU)),						    // LAME coefficient - MU
	LAMBDA = (E * NU) / ((1.f + NU) * (1.f - (2.f * NU)));  // LAME coefficient - LAMBDA
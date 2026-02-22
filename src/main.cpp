/*
 Implemenation of A Material Point Method for Snow Simulation (Stomakhin et al, 2013)
*/

// Includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cstring>
#include <stdexcept>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "Grid.h"
#include "Particles.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h" 

// Variables
GLFWwindow* window;
Grid *grid;
Particles* particleCloud;

int STEPS = 0;
int SAVE_FRAME = 0;
float CUM_DT = 0;

// Namespaces
using namespace glm;


void Initialize()
{
    // Initiailize the Grid
    grid = new Grid(WIDTH, HEIGHT, BORDER_MARGIN);

    // Initialize the Particles based on the pre-defined shapes (Parameters.h)
    particleCloud = new Particles(*grid);
}

void Simulate()
{
    float dt = DELTA_TIME;
    while (CUM_DT < (1.0 / SAVE_FPS))
    {
        particleCloud->Solver();
        CUM_DT += dt;

        STEPS++;
    }

    CUM_DT = 0;
}

void Visualize()
{
    /* Border */
    auto& borderLines = grid->GetBorder();
    if (borderLines.size() != 4) 
        throw std::runtime_error("[ERROR] Grid visualization failed - incorrect boundaries");

    glLineWidth(2.0f);
    glColor3f(0.5f, 0.5f, 0.5f);  // grey

    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 4; i++)
        glVertex2f(borderLines[i].p1.x, borderLines[i].p1.y);

    glEnd();
    glLineWidth(1.0f);

    /* Particles */
    auto& particles = particleCloud->GetParticles();
    if (particles.size() < 1)
    {
        std::cout << "[WARNING] There are not particles to visualize" << std::endl;
        return;
    }

    for (auto& particle : particles)
    {
        glPointSize(PARTICLE_SIZE);
        glColor3f(1.f, 1.f, 1.f);  // white

        glEnable(GL_POINT_SMOOTH);
        glBegin(GL_POINTS);

        vec2 pos = particle.GetPosition();
        glVertex2f(pos.x, pos.y); // TODO: randomize
        glEnd();
    }
}

void saveFrame()
{
    char filepath[200] = RENDER_DIR;
    strcat(filepath, "/frame_");

    std::string frameStr = std::to_string(SAVE_FRAME);
    strcat(filepath, frameStr.c_str());
    strcat(filepath, ".png");

    // Read the data
    glReadBuffer(GL_FRONT);
    unsigned char* data = new unsigned char[WINDOW_WIDTH * WINDOW_HEIGHT * 3];
    glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, data);

    // Flip the image vertically
    for (int y = 0; y < WINDOW_HEIGHT / 2; ++y)
    {
        for (int x = 0; x < WINDOW_WIDTH * 3; ++x)
        {
            std::swap(data[y * WINDOW_WIDTH * 3 + x], data[(WINDOW_HEIGHT - y - 1) * WINDOW_WIDTH * 3 + x]);
        }
    }

    // Save to file
    stbi_write_png(filepath, WINDOW_WIDTH, WINDOW_HEIGHT, 3, data, WINDOW_WIDTH * 3);
    delete[] data;

    SAVE_FRAME++;
}

int main(void)
{
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "MPM Simulation (Snow)", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    
    // Set up the initial OpenGL state for a 2D rendering context
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(0, WIDTH, 0, HEIGHT, -1, 1);
    glViewport(0, 0, (GLsizei)WINDOW_WIDTH, (GLsizei)WINDOW_HEIGHT);

    std::cout << "[DEBUG] Initializing Simulation" << std::endl;
    Initialize();

    int frameCount = 1;
    while (!glfwWindowShouldClose(window)) 
    {
        std::cout << "[DEBUG] Frame: " << frameCount << std::endl;
        Simulate();

        glClear(GL_COLOR_BUFFER_BIT);

        Visualize();

        glfwSwapBuffers(window);
        glfwPollEvents();
    
        saveFrame();

        frameCount++;

        if (SAVE_FRAME == 200) break; // TODO:
    }

    glfwTerminate();
    return 0;
}


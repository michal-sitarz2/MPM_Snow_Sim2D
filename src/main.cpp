/* Implemenation of A Material Point Method for Snow Simulation (Stomakhin et al, 2013) */

// Includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <chrono>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Parameters.h"
#include "Log.h"

// CPU or GPU Simulation Wrapper
#if USE_GPU
    #include "GPUSolver.h"
    using Solver = GPUSolver;
#else
    #include "CPUSolver.h"
    using Solver = CPUSolver;
#endif

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h" 

// Force NVIDIA GPU on laptop with hybrid graphics
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 1;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}

// Variables
GLFWwindow* window;

int STEPS = 0;
int SAVE_FRAME = 0;
float CUM_DT = 0;

// Namespaces
using namespace glm;


#ifdef SIM_3D
float cameraAngle = 0.0f;     // horizontal orbit angle in degrees
float cameraElevation = 30.0f; // vertical angle
float cameraDistance = 300.0f;  // distance from center

void SetupCamera()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT;
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 1.0f, 1000.0f);
    glLoadMatrixf(glm::value_ptr(proj));

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera orbits around the center of the grid
    glm::vec3 center = glm::vec3(WIDTH * 0.5f, HEIGHT * 0.5f, DEPTH * 0.5f);
    float rad = glm::radians(cameraAngle);
    float elev = glm::radians(cameraElevation);
    glm::vec3 eye = center + cameraDistance * glm::vec3(
        std::cos(elev) * std::sin(rad),
        std::sin(elev),
        std::cos(elev) * std::cos(rad)
    );

    glm::mat4 view = glm::lookAt(eye, center, glm::vec3(0, 1, 0));
    glLoadMatrixf(glm::value_ptr(view));
}

void Visualize(Solver& solver)
{
    SetupCamera();
    
    /* Wireframe Box */
    float x0 = (float)BORDER_MARGIN, y0 = (float)BORDER_MARGIN, z0 = (float)BORDER_MARGIN;
    float x1 = (float)(WIDTH - BORDER_MARGIN);
    float y1 = (float)(HEIGHT - BORDER_MARGIN);
    float z1 = (float)(DEPTH - BORDER_MARGIN);

    glLineWidth(2.0f);
    glColor3f(0.5f, 0.5f, 0.5f);
    glBegin(GL_LINES);
        // Bottom face
        glVertex3f(x0,y0,z0); glVertex3f(x1,y0,z0);
        glVertex3f(x1,y0,z0); glVertex3f(x1,y0,z1);
        glVertex3f(x1,y0,z1); glVertex3f(x0,y0,z1);
        glVertex3f(x0,y0,z1); glVertex3f(x0,y0,z0);
        // Top face
        glVertex3f(x0,y1,z0); glVertex3f(x1,y1,z0);
        glVertex3f(x1,y1,z0); glVertex3f(x1,y1,z1);
        glVertex3f(x1,y1,z1); glVertex3f(x0,y1,z1);
        glVertex3f(x0,y1,z1); glVertex3f(x0,y1,z0);
        // Pillars
        glVertex3f(x0,y0,z0); glVertex3f(x0,y1,z0);
        glVertex3f(x1,y0,z0); glVertex3f(x1,y1,z0);
        glVertex3f(x1,y0,z1); glVertex3f(x1,y1,z1);
        glVertex3f(x0,y0,z1); glVertex3f(x0,y1,z1);
    glEnd();
    glLineWidth(1.0f);

    /* Particles as Points */
    const auto& positions = solver.GetParticlePositions();
    if (positions.empty())
    {
        Log::Warn("There are no particles to visualize");
        return;
    }

    glPointSize(PARTICLE_SIZE);
    glColor3f(1.f, 1.f, 1.f);
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    for (const auto& pos : positions)
        glVertex3f(pos.x, pos.y, pos.z);
    glEnd();
}

#else
/* Visualize in 2D: Border and Simulated Particles */
void Visualize(Solver& solver)
{
    /* Border */
    glLineWidth(2.0f);
    glColor3f(0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(BORDER_MARGIN, BORDER_MARGIN);
        glVertex2f(WIDTH - BORDER_MARGIN, BORDER_MARGIN);
        glVertex2f(WIDTH - BORDER_MARGIN, HEIGHT - BORDER_MARGIN);
        glVertex2f(BORDER_MARGIN, HEIGHT - BORDER_MARGIN);
    glEnd();
    glLineWidth(1.0f);

    /* Particles */
    const auto& positions = solver.GetParticlePositions();
    if (positions.empty())
    {
        Log::Warn("There are no particles to visualize");
        return;
    }

    glPointSize(PARTICLE_SIZE);
    glColor3f(1.f, 1.f, 1.f); // white
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    for (const auto& pos : positions)
        glVertex2f(pos.x, pos.y);
    glEnd();
}
#endif

/* Run Simulation Steps */
void Simulate(Solver& solver)
{
    float dt = DELTA_TIME;
    while (CUM_DT < (1.0 / SAVE_FPS))
    {
        solver.Step();
        CUM_DT += dt;
        STEPS++;
    }
    CUM_DT = 0;
}

/* Save the Frame in RENDER_DIR */
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
    if (!USE_GPU && DIM == 3)
    {
        Log::Error("Cannot run 3D MPM on CPU");
        return -1;
    }

    // Initialize the Logger - all Log::[Info]/[Error]/[Warn] goes to console and mpm.log
    Log::Init(std::string(RENDER_DIR) + "/mpm.log");

    // Initialize GLFW - windowing and input library
    if (!glfwInit()) 
    {
        Log::Error("Failed to initialize GLFW");
        return -1;
    }

    // Requesting OpenGL 4.6 Compatibility profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    // Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "MPM Simulation (Snow)", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    // Make the OpenGL context current on this thread
    glfwMakeContextCurrent(window);

    // Initialize GLEW - loading OpenGL function pointers at runtime
    glewExperimental = GL_TRUE;
    glewInit();

    // GPU and driver information
    Log::Info("OpenGL version: " + std::string((const char*)glGetString(GL_VERSION)));
    Log::Info("GLSL version: "   + std::string((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION)));
    Log::Info("Vendor: "         + std::string((const char*)glGetString(GL_VENDOR)));
    Log::Info("Renderer: "       + std::string((const char*)glGetString(GL_RENDERER)));
    Log::Info("================================================================");
    
    // Set up the initial OpenGL state for a 2D rendering context
    #ifdef SIM_3D
        glEnable(GL_DEPTH_TEST);
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
    #else
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, WIDTH, 0, HEIGHT, -1, 1);
        glViewport(0, 0, (GLsizei)WINDOW_WIDTH, (GLsizei)WINDOW_HEIGHT);
    #endif

    std::ostringstream oss;
    oss << "Initializing " << DIM << "D Simulation (" << (USE_GPU ? "GPU" : "CPU") << ")";
    Log::Info(oss.str());

    Solver solver;

    // Main Loop - simulation and render
    int frameCount = 1;
    while (!glfwWindowShouldClose(window)) 
    {
        Log::Info("Frame: " + std::to_string(frameCount));

        // Advance the simulation
        Simulate(solver);

        // Clear the framebuffer and draw the current particle state
        #ifdef SIM_3D
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        #else
            glClear(GL_COLOR_BUFFER_BIT);
        #endif
        Visualize(solver);

        // Present the rendered frame and process any pending window events
        glfwSwapBuffers(window);
        glfwPollEvents();
    
        // Save frame PNGs to /render
        saveFrame();

        // Stop after MAX_FRAMES
        if (frameCount++ >= MAX_FRAMES) break;
    }

    glfwTerminate();
    return 0;
}


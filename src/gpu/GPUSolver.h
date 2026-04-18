#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>

#include "Parameters.h"
#include "Log.h"

class GPUSolver {
public:
    GPUSolver();
    ~GPUSolver();

    void Step();
    const std::vector<glm::vec2>& GetParticlePositions();

private:
    std::vector<glm::vec2> cachedPositions;

    /** Shader Programs **/
    GLuint reset_prog;
    GLuint p2g_prog;
    GLuint gridUpdate_prog;
    GLuint g2p_prog;
    
    /** SSBO Binding Table **/
    // Particle SSBOs
    GLuint ssbo_pos; // 0: particle position
    GLuint ssbo_vel; // 1: particle velocity
    GLuint ssbo_mass; // 2: particle mass
    GLuint ssbo_vol; // 3: particle initial volume

    GLuint ssbo_Fe; // 4: elastic deformation gradient (Fe)
    GLuint ssbo_Fp; // 5: plastic deformation gradient (Fp)
    GLuint ssbo_Bp; // 6: velocity field (Bp)

    // Grid SSBOs
    GLuint ssbo_gMass; // 7: grid cell mass
    GLuint ssbo_gVelX, ssbo_gVelY; // 8, 9: grid velocities
    GLuint ssbo_gVelCol, ssbo_gVelFric; // 10, 11: grid velocity (post collision/friction)

    /** Counts **/
    int numParticles;
    int gWidth, gHeight;

    /** Helpers **/
    void InitBuffers();
    
    void ReadbackParticles(std::vector<glm::vec2>& outPos, std::vector<glm::vec2>& outVel);
    void ReadbackGridMass(std::vector<float>& outMass);

    void DispatchParticles(GLuint prog);
    void DispatchGrid(GLuint prog);
    
    GLuint CompileShader(const char* path, GLenum type);
    GLuint LinkProgram(GLuint shaderID);
    void CheckGLError(const char* name);
};
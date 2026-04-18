#include "GPUSolver.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "CPUSolver.h"

GPUSolver::GPUSolver()
{
    Log::Info("GPU Solver Initialization");

    gWidth = WIDTH;
    gHeight = HEIGHT;

    /* Initialize SSBOs */
    InitBuffers();
    
    /* Try to Compile Shaders */
    try
    {
        // Particles to Grid step
        GLuint p2g_shader = CompileShader(SHADER_DIR "/p2g.glsl", GL_COMPUTE_SHADER);
        p2g_prog = LinkProgram(p2g_shader);

        // Grid Update step
        GLuint gridUpdate_shader = CompileShader(SHADER_DIR "/grid_update.glsl", GL_COMPUTE_SHADER);
        gridUpdate_prog = LinkProgram(gridUpdate_shader);

        // Grid Update step
        GLuint g2p_shader = CompileShader(SHADER_DIR "/g2p.glsl", GL_COMPUTE_SHADER);
        g2p_prog = LinkProgram(g2p_shader);

        // Reset the Grid
        GLuint reset_shader = CompileShader(SHADER_DIR "/reset.glsl", GL_COMPUTE_SHADER);
        reset_prog = LinkProgram(reset_shader);
    }
    catch (const std::exception& e)
    {
        const std::string msg = "Shader Compilation: " + std::string(e.what());
        Log::Error(msg);
        throw msg;
    }
    catch (...)
    {
        const std::string msg = "Shader Compilation";
        Log::Error(msg);
        throw msg;
    }
}

GPUSolver::~GPUSolver() 
{ 
    /* Cleanup */
    glDeleteProgram(reset_prog);
    glDeleteProgram(g2p_prog);
    glDeleteProgram(gridUpdate_prog);
    glDeleteProgram(p2g_prog);
}

/* MPM Solver Steps */
void GPUSolver::Step()
{
    /* Particle to Grid (P2G) scatter step */
    DispatchParticles(p2g_prog);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    /* Update Grid: forces, collision, friction */
    DispatchGrid(gridUpdate_prog);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    /* Grid to Particle (G2P) gather step + deformation update */
    DispatchParticles(g2p_prog);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    /* Reset the Grid */
    DispatchGrid(reset_prog);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);    
}

/* Build SoA vectors from pre-defined shapes */
void GPUSolver::InitBuffers()
{
    /* Allocate and upload SSBO */
    auto allocateSSBO = [](GLuint& id, GLuint binding,
                       const void* data, size_t bytes)
    {
        glGenBuffers(1, &id);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, id);
        glBufferData(GL_SHADER_STORAGE_BUFFER, bytes, data, GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, id);
    };
    
    /* Particles */
    std::vector<float> mass, vol;
    std::vector<glm::vec2> pos, vel;
    std::vector<glm::mat2> Fe, Fp, Ap, Bp;

    for (SHAPE* shape : SHAPES) /* shapes defined in shared/Parameters.h */
    {
        auto* snowball = dynamic_cast<SNOWBALL_SHAPE*>(shape);
        if (!snowball) 
        {
            std::string msg = "Unsupported shape type";
            Log::Error(msg);
            throw std::runtime_error(msg);
        }

        const float radius = snowball->radius;
        float minDist = radius / std::sqrt((float)NUM_PARTICLES);
        snowball->GenerateSamples(minDist);

        for (auto& p : snowball->diskSamples)
        {
            float v = 2.f * PI * radius * radius / (float) snowball->diskSamples.size();
            float m = ((v * RHO) / 100.f); 

            pos.push_back(p);
            vel.push_back(snowball->vel);
            vol.push_back(v);
            mass.push_back(m);
            Fe.push_back(glm::mat2(1.f));
            Fp.push_back(glm::mat2(1.f));
            Ap.push_back(glm::mat2(0.f));
            Bp.push_back(glm::mat2(0.f));
        }
    }

    numParticles = (int)pos.size();
    Log::Info("Particle Count: " + std::to_string(numParticles));

    /* Particle Buffers */
    Log::Info("Particle Buffer allocation");
    allocateSSBO(ssbo_pos,  0, pos.data(),  numParticles * sizeof(glm::vec2));
    allocateSSBO(ssbo_vel,  1, vel.data(),  numParticles * sizeof(glm::vec2));
    allocateSSBO(ssbo_mass, 2, mass.data(), numParticles * sizeof(float));
    allocateSSBO(ssbo_vol,  3, vol.data(),  numParticles * sizeof(float));
    allocateSSBO(ssbo_Fe,   4, Fe.data(),   numParticles * sizeof(glm::mat2));
    allocateSSBO(ssbo_Fp,   5, Fp.data(),   numParticles * sizeof(glm::mat2));
    allocateSSBO(ssbo_Bp,   6, Bp.data(),   numParticles * sizeof(glm::mat2));
    CheckGLError("InitBuffers Particles");

    /* Grid */
    int gSize = (gWidth + 1) * (gHeight + 1);
    Log::Info("Number of grid cells: " + std::to_string(gSize));

    std::vector<float> zeroFloat(gSize, 0.f);
    std::vector<glm::vec2> zeroVec(gSize, glm::vec2(0.f));

    /* Grid Buffers */
    Log::Info("Grid Buffer allocation");
    allocateSSBO(ssbo_gMass,    7,  zeroFloat.data(), gSize * sizeof(float));
    allocateSSBO(ssbo_gVelX,    8,  zeroFloat.data(), gSize * sizeof(float));
    allocateSSBO(ssbo_gVelY,    9, zeroFloat.data(), gSize * sizeof(float));
    allocateSSBO(ssbo_gVelCol,  10, zeroVec.data(), gSize * sizeof(glm::vec2));
    allocateSSBO(ssbo_gVelFric, 11, zeroVec.data(), gSize * sizeof(glm::vec2));
    CheckGLError("InitBuffers Grid");
}

void GPUSolver::ReadbackParticles(std::vector<glm::vec2>& outPos, std::vector<glm::vec2>& outVel)
{
    outPos.resize(numParticles);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_pos);
    glGetBufferSubData(
        GL_SHADER_STORAGE_BUFFER, 
        0, 
        numParticles * sizeof(glm::vec2), 
        outPos.data()
    );

    outVel.resize(numParticles);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_vel);
    glGetBufferSubData(
        GL_SHADER_STORAGE_BUFFER, 
        0, 
        numParticles * sizeof(glm::vec2), 
        outVel.data()
    );

    CheckGLError("ReadbackParticles");
}

void GPUSolver::DispatchParticles(GLuint prog)
{
    glUseProgram(prog);
    
    glUniform1i(glGetUniformLocation(prog, "numParticles"), numParticles);
    glUniform1i(glGetUniformLocation(prog, "gWidth"), gWidth);
    glUniform1i(glGetUniformLocation(prog, "gHeight"), gHeight);
    glUniform1f(glGetUniformLocation(prog, "XI"), XI);
    glUniform1f(glGetUniformLocation(prog, "MU"), MU);
    glUniform1f(glGetUniformLocation(prog, "LAMBDA"), LAMBDA);
    glUniform1f(glGetUniformLocation(prog, "DELTA_TIME"), DELTA_TIME);
    glUniform1f(glGetUniformLocation(prog, "THETA_C"), THETA_C);
    glUniform1f(glGetUniformLocation(prog, "THETA_S"), THETA_S);

    int groups = (numParticles + 63) / 64;
    
    glDispatchCompute(groups, 1, 1);
}

void GPUSolver::DispatchGrid(GLuint prog)
{
    glUseProgram(prog);
    
    glUniform1i(glGetUniformLocation(prog, "gWidth"), gWidth);
    glUniform1i(glGetUniformLocation(prog, "gHeight"), gHeight);
    glUniform1f(glGetUniformLocation(prog, "borderMargin"), (float)BORDER_MARGIN);
    glUniform1f(glGetUniformLocation(prog, "DELTA_TIME"), DELTA_TIME);
    glUniform1f(glGetUniformLocation(prog, "GRAVITY"), GRAVITY);
    glUniform1f(glGetUniformLocation(prog, "FRICTION"), FRICTION);
    
    int cells  = (gWidth + 1) * (gHeight + 1);
    int groups = (cells + 63) / 64;
    
    glDispatchCompute(groups, 1, 1);
}

const std::vector<glm::vec2>& GPUSolver::GetParticlePositions()
{
    glFinish();
    cachedPositions.resize(numParticles);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_pos);
    glGetBufferSubData(
        GL_SHADER_STORAGE_BUFFER, 
        0,               
        numParticles * sizeof(glm::vec2),
        cachedPositions.data()
    );
    
    return cachedPositions;
}

/* Take a text shader file and turn it into a GPU-understandable object and return an ID */
GLuint GPUSolver::CompileShader(const char* path, GLenum type)
{
    /* Read common.glsl */
    std::string common = "";
    std::ifstream commonFile(SHADER_DIR "/common.glsl");
    if (commonFile)
    {
        std::stringstream ss;
        ss << commonFile.rdbuf();
        common = ss.str();
    }
    else
    {
        Log::Warn("common.glsl not found");
    }

    /* Read source from file */
    std::ifstream file(path);
    if (!file)
    {
        std::string msg = std::string("Cannot open GPU shader: ") + path;
        Log::Error(msg);
        throw std::runtime_error(msg);
    }

    std::stringstream ss;
    ss << file.rdbuf();
    std::string src = ss.str();

    /* Inject common.glsl if requested */
    std::string finalSrc;
    if (src.find("// #include common.glsl") != std::string::npos)
    {
        std::string versionAndExtensions;
        std::string rest;
        std::istringstream srcStream(src);
        std::string line;
        while (std::getline(srcStream, line))
        {
            std::string trimmed = line;
            trimmed.erase(0, trimmed.find_first_not_of(" \t"));
            if (trimmed.substr(0, 8) == "#version" || 
                trimmed.substr(0, 10) == "#extension")
                versionAndExtensions += line + "\n";
            else
            {
                std::ostringstream remaining;
                remaining << line << "\n" << srcStream.rdbuf();
                rest = remaining.str();
                break;
            }
        }
        std::ostringstream combined;
        combined << versionAndExtensions << common << "\n" << rest;
        finalSrc = combined.str();
    }
    else
    {
        finalSrc = src;
    }
    const char* c = finalSrc.c_str();

    /* Compile shader (turn into something executable) */
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &c, nullptr);
    glCompileShader(shader);

    /* Verify */
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char log[2048];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        throw std::runtime_error(std::string("GPU Shader compile error in ")
                                 + path + ":\n" + log);
    }

    Log::Info(std::string("GPU Shader Compiled: ") + path);
    return shader;
}

/* Take a computed shader (based on the ID) and make a usable program on the GPU */
GLuint GPUSolver::LinkProgram(GLuint shaderID)
{
    GLuint prog = glCreateProgram();
    glAttachShader(prog, shaderID);
    glLinkProgram(prog); // link into one runnable unit

    /* Verify */
    GLint success;
    glGetProgramiv(prog, GL_LINK_STATUS, &success);
    if (!success)
    {
        char log[2048];
        glGetProgramInfoLog(prog, sizeof(log), nullptr, log);

        std::string msg = std::string("Program link error:\n") + log;
        Log::Error(msg);
        throw std::runtime_error(msg);
    }

    // Shader object no longer needed once linked
    glDeleteShader(shaderID);

    Log::Info("Shader Program Linked");
    return prog;
}

void GPUSolver::CheckGLError(const char* name)
{
    GLenum err;
    bool found = false;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        found = true;
        Log::Error("OpenGL error at " + std::string(name) + ": " + std::to_string(err));
    }
    if (!found)
        Log::Info("No errors at " + std::string(name));
}
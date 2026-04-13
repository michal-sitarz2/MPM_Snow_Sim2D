#version 430
layout(local_size_x = 64) in;

layout(std430, binding = 8) coherent buffer GMass { float gMass[]; };
layout(std430, binding = 9) coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 10) coherent buffer GVelY { float gVelY[]; };
layout(std430, binding = 11) coherent buffer GForceX { float gForceX[]; };
layout(std430, binding = 12) coherent buffer GForceY { float gForceY[]; };
layout(std430, binding = 13) coherent buffer GVelCol { vec2  gVelCol[]; };
layout(std430, binding = 14) coherent buffer GVelFric { vec2  gVelFric[]; };

uniform int gWidth;
uniform int gHeight;
uniform float borderMargin;
uniform float DELTA_TIME;
uniform float GRAVITY;
uniform float FRICTION;

void detectCollisions(vec2 loc, vec2 vel, out vec2 vel_collision, out vec2 vel_friction)
{
    float w = float(gWidth), h = float(gHeight), m = borderMargin;

    /* Borders (see Grid.cpp) */
    vec2 bP[4] = vec2[4](
        vec2(w - m, m),    // right
        vec2(m, m),        // bottom
        vec2(m, h - m),    // left
        vec2(w - m, h - m) // top
    );
    vec2 bN[4] = vec2[4](
        vec2(-1.0, 0.0),  // right
        vec2( 0.0, 1.0),  // bottom
        vec2( 1.0, 0.0),  // left
        vec2( 0.0, -1.0)  // top
    );

    /* Border Collisions */
    vel_collision = vel;

    vec2 colNormals[4]; // 4 walls (TODO: obstacles)
    int totalCollisions = 0;

    for (int b = 0; b < 4; b++)
    {
        // Signed Distance to Boundary
        vec2 diff = loc - bP[b];
        float dist = dot(bN[b], diff);

        // Test Distance after Moving (predicts future position)
        vec2 test_diff = (loc + DELTA_TIME * vel_collision) - bP[b];
        float test_dist = dot(bN[b], test_diff);

        // Net change in penetration during the timestep
        float delta = test_dist - min(dist, 0.0);

        if (delta < 0.0)
        {
            // Velocity correction in the direction of the normal
            vel_collision -= (delta / DELTA_TIME) * bN[b];

            // Save and update total collisions
            colNormals[totalCollisions++] = bN[b];
        }
    }

    /* Friction for Tangential Velocity */
    vel_friction = vel_collision;

    for (int c = 0; c < totalCollisions; c++)
    {
        // Tangential Velocity
        vec2 vt = vel_collision - colNormals[c] * dot(colNormals[c], vel_friction);
    
        float vt_norm = length(vt);
        if (vt_norm <= 1e-7) continue;
        vec2 t = vt / vt_norm;

        // Coulomb's friction
        vec2  diff_col = vel_collision - vel;
        vel_friction -=  min(vt_norm, FRICTION * length(diff_col)) * t;
    }
}

void main()
{
    uint id = gl_GlobalInvocationID.x;
    uint total = uint((gWidth + 1) * (gHeight + 1));
    if (id >= total) return;

    // Skip if Mass Insignificant at the grid cell
    float mass = gMass[id];
    if (mass < 1e-6) return;

    // Normalize the velocities
    vec2 vel = vec2(gVelX[id], gVelY[id]) / mass;

    // Update forces
    vec2 old_force = vec2(gForceX[id], gForceY[id]);
    vec2 new_force = (vec2(0.0, GRAVITY) + (-old_force / mass)) * DELTA_TIME;
    vel += new_force;

    /* Cell location 2D */
    int  x = int(id) % (gWidth + 1);
    int  y = int(id) / (gWidth + 1);
    vec2 loc = vec2(float(x), float(y));

    /* Collisions and Friction */
    vec2 vel_col, vel_fric;
    detectCollisions(loc, vel, vel_col, vel_fric);

    gVelCol[id] = vel_col;
    gVelFric[id] = vel_fric;
}
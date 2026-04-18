#version 430
layout(local_size_x = 64) in;

layout(std430, binding = 7) coherent buffer GMass { float gMass[]; };
layout(std430, binding = 8) coherent buffer GVelX { float gVelX[]; };
layout(std430, binding = 9) coherent buffer GVelY { float gVelY[]; };
layout(std430, binding = 10) coherent buffer GVelZ { float gVelZ[]; };
layout(std430, binding = 11) coherent buffer GVelCol { vec4 gVelCol[]; };
layout(std430, binding = 12) coherent buffer GVelFric { vec4 gVelFric[]; };

uniform int gWidth;
uniform int gHeight;
uniform int gDepth;
uniform float borderMargin;
uniform float DELTA_TIME;
uniform float GRAVITY;
uniform float FRICTION;

void detectCollisions(vec3 loc, vec3 vel, out vec3 vel_collision, out vec3 vel_friction)
{
    float w = float(gWidth), h = float(gHeight), d = float(gDepth), m = borderMargin;

    /* 3D Borders - 6 box faces */
    vec3 bP[6] = vec3[6](
        vec3(w - m, 0, 0),    // right (+X)
        vec3(m, 0, 0),        // left (-X)
        vec3(0, h - m, 0),    // top (+Y)
        vec3(0, m, 0),        // bottom (-Y)
        vec3(0, 0, d - m),    // far (+Z)
        vec3(0, 0, m)         // near (-Z)
    );
    vec3 bN[6] = vec3[6](
        vec3(-1, 0, 0),  // right (+X)
        vec3(1, 0, 0),   // left (-X)
        vec3(0, -1, 0),  // top (+Y)
        vec3(0, 1, 0),   // bottom (-Y)
        vec3(0, 0, -1),  // far (+Z)
        vec3(0, 0, 1)    // near (-Z)
    );

    /* Border Collisions */
    vel_collision = vel;

    vec3 colNormals[6]; // 6 walls (TODO: obstacles)
    int totalCollisions = 0;

    for (int b = 0; b < 6; b++)
    {
        // Signed Distance to Boundary
        vec3 diff = loc - bP[b];
        float dist = dot(bN[b], diff);

        // Test Distance after Moving (predicts future position)
        vec3 test_diff = (loc + DELTA_TIME * vel_collision) - bP[b];
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
        vec3 vt = vel_collision - colNormals[c] * dot(colNormals[c], vel_friction);
    
        float vt_norm = length(vt);
        if (vt_norm <= 1e-7) continue;
        vec3 t = vt / vt_norm;

        // Coulomb's friction
        vec3  diff_col = vel_collision - vel;
        vel_friction -=  min(vt_norm, FRICTION * length(diff_col)) * t;
    }
}

void main()
{
    uint id = gl_GlobalInvocationID.x;
    uint total = uint((gWidth + 1) * (gHeight + 1) * (gDepth + 1));
    if (id >= total) return;

    // Skip if Mass Insignificant at the grid cell
    float mass = gMass[id];
    if (mass < 1e-6) return;

    // Normalize the velocities
    vec3 vel = vec3(gVelX[id], gVelY[id], gVelZ[id]) / mass;

    // Gravity
    vel += vec3(0.0, GRAVITY, 0.0) * DELTA_TIME;

    /* Cell location 3D */
    int wh = (gWidth + 1) * (gHeight + 1);
    int z = int(id) / wh;
    int rem = int(id) % wh;
    int y = rem / (gWidth + 1);
    int x = rem % (gWidth + 1);
    vec3 loc = vec3(float(x), float(y), float(z));

    /* Collisions and Friction */
    vec3 vel_col, vel_fric;
    detectCollisions(loc, vel, vel_col, vel_fric);

    gVelCol[id] = vec4(vel_col, 0.0);
    gVelFric[id] = vec4(vel_fric, 0.0);
}
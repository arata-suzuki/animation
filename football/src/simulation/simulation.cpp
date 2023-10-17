#include "simulation.hpp"

using namespace cgp;




// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force

vec3 compute_structural_force(cloth_structure& cloth, float K, float L0, int ku, int kv, int size, std::vector<int> neighbor) {
    if ((ku < 1 and neighbor[0] < 0) or (ku >= size - 1 and neighbor[0] > 0) or (kv < 1 and neighbor[1] < 0) or
        (kv >= size - 1 and neighbor[1] > 0))
        return vec3(0, 0, 0);

    vec3 pi = cloth.position(ku, kv);
    return K * (norm(cloth.position(ku + neighbor[0], kv + neighbor[1]) - pi) - L0)
           * ((cloth.position(ku + neighbor[0], kv + neighbor[1]) - pi) /
              norm(cloth.position(ku + neighbor[0], kv + neighbor[1]) - pi));
}

vec3 compute_shearing_force(cloth_structure& cloth, float K, float L0, int ku, int kv, int size, std::vector<int> neighbor) {
    if ((ku < 1 and neighbor[0] < 0) or (ku >= size-1 and neighbor[0] > 0) or (kv < 1 and neighbor[1] < 0) or (kv >= size-1 and neighbor[1] > 0))
        return vec3(0,0,0);

    vec3 pi = cloth.position(ku, kv);
    return K * (norm(cloth.position(ku+neighbor[0],kv+neighbor[1]) - pi) - sqrt(2) * L0)
           * ((cloth.position(ku+neighbor[0],kv+neighbor[1]) - pi) / norm(cloth.position(ku+neighbor[0],kv+neighbor[1]) - pi));
}
vec3 compute_bending_force(cloth_structure& cloth, float K, float L0, int ku, int kv, int size, std::vector<int> neighbor) {
    if ((ku < 2 and neighbor[0] < 0) or (ku >= size-2 and neighbor[0] > 0) or (kv < 2 and neighbor[1] < 0) or (kv >= size-2 and neighbor[1] > 0))
        return vec3(0,0,0);
    vec3 pi = cloth.position(ku, kv);
    return K * (norm(cloth.position(ku+neighbor[0],kv+neighbor[1]) - pi) - 2 * L0)
           * ((cloth.position(ku+neighbor[0],kv+neighbor[1]) - pi) / norm(cloth.position(ku+neighbor[0],kv+neighbor[1]) - pi));
}
void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters)
{
    // Direct access to the variables
    //  Note: A grid_2D is a structure you can access using its 2d-local index coordinates as grid_2d(k1,k2)
    //   The index corresponding to grid_2d(k1,k2) is k1 + N1*k2, with N1 the first dimension of the grid.
    //   
    grid_2D<vec3>& force = cloth.force;  // Storage for the forces exerted on each vertex

    grid_2D<vec3> const& position = cloth.position;  // Storage for the positions of the vertices
    grid_2D<vec3> const& velocity = cloth.velocity;  // Storage for the normals of the vertices
    grid_2D<vec3> const& normal = cloth.normal;      // Storage for the velocity of the vertices


    size_t const N_total = cloth.position.size();       // total number of vertices
    size_t const N = cloth.N_samples();                 // number of vertices in one dimension of the grid

    // Retrieve simulation parameter
    //  The default value of the simulation parameters are defined in simulation.hpp
    float const K = parameters.K;              // spring stifness
    float const m = parameters.mass_total / N_total; // mass of a particle
    float const mu = parameters.mu;            // damping/friction coefficient
    float const	L0 = 1.0f / (N - 1.0f);        // rest length between two direct neighboring particle


    // Gravity
    const vec3 g = { 0,0,-9.81f };
    for (int ku = 0; ku < N; ++ku)
        for (int kv = 0; kv < N; ++kv)
            force(ku, kv) = m * g;

    // Drag (= friction)
    for (int ku = 0; ku < N; ++ku)
        for (int kv = 0; kv < N; ++kv)
            force(ku, kv) += -mu * m * velocity(ku, kv);


    // TO DO: Add spring forces ...

    std::vector<std::vector<int>> neighbors = {{1,0}, {0,1}, {-1,0}, {0,-1},
                                               {1,1}, {-1,1}, {1, -1}, {-1,-1},
                                               {2,0}, {0,2}, {-2,0}, {0,-2}};

    //... fill here the force exerted by all the springs attached to the vertex at coordinates (ku,kv).
    for (int ku = 0; ku < N; ++ku) {
        for (int kv = 0; kv < N; ++kv) {
            // ...
            for (int i = 0; i < neighbors.size(); i++) {
                if (i > 7)
                    force(ku,kv) += compute_bending_force(cloth, K, L0, ku, kv, N, neighbors[i]);
                else if (i > 3)
                    force(ku, kv) += compute_shearing_force(cloth, K, L0, ku, kv, N, neighbors[i]);
                else
                    force(ku, kv) += compute_structural_force(cloth, K, L0, ku, kv, N, neighbors[i]);
            }

            //
            // Notes:
            //   - The vertex positions can be accessed as position(ku,kv)
            //   - The neighbors are at position(ku+1,kv), position(ku-1,kv), position(ku,kv+1), etc. when ku+offset is still in the grid dimension.
            //   - You may want to loop over all the neighbors of a vertex to add each contributing force to this vertex
            //   - To void repetitions and limit the need of debuging, it may be a good idea to define a generic function that computes the spring force between two positions given the parameters K and L0
            //   - If the simulation is a bit too slow, you can speed it up in adapting the parameter N_step in scene.cpp that loops over several simulation step between two displays.
        }
    }

}

void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt)
{
    int const N = cloth.N_samples();
    int const N_total = cloth.position.size();
    float const m = parameters.mass_total/ static_cast<float>(N_total);

    for (int ku = 0; ku < N; ++ku) {
        for (int kv = 0; kv < N; ++kv) {
            vec3& v = cloth.velocity(ku, kv);
            vec3& p = cloth.position(ku, kv);
            vec3 const& f = cloth.force(ku, kv);

            // Standard semi-implicit numerical integration
            v = v + dt * f / m;
            p = p + dt * v;
        }
    }

}

void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint, ball_structure& ball)
{
    // Fixed positions of the cloth
    for (auto const& c : constraint.fixed_sample) {
        cloth.position(c.ku, c.kv) = c.position; // set the position to the fixed one
    }

    // To do: apply external constraints
    // For all vertex:
    //   If vertex is below floor level ...
    size_t const N = cloth.N_samples();

    for (int ku = 0; ku < N; ku++) {
        for (int kv = 0; kv < N; kv++) {
            if (norm(cloth.position(ku, kv) - constraint.sphere.center) < ball.r + 0.03f)
            {

                //if (cloth.position(ku, kv).z > constraint.sphere.center.z + 0.1f)
                //    cloth.position(ku,kv).z += constraint.sphere.radius - norm(cloth.position(ku, kv) - constraint.sphere.center) + 0.03f;
                //else {
                cloth.position(ku,kv).y -= (ball.r - norm(cloth.position(ku, kv) - constraint.sphere.center) + 0.03f);
                    //cloth.position(ku,kv).z -= constraint.sphere.radius - norm(cloth.position(ku, kv) - constraint.sphere.center) + 0.03f;
                //}


            }

            else if (cloth.position(ku,kv).z <= constraint.ground_z + 0.01f)
                cloth.position(ku, kv).z = constraint.ground_z + 0.01f;
        }
    }
    //   If vertex is inside collision sphere ...

}

void simulation_power_shoot(ball_structure& ball, float pressing_time, float shooting_angle) {
    //Horizontal speed of the ball, proportionally to the time we hold space
    float v0y = std::min(float(pressing_time), 1.0f) * 8.0f;

     //Vertical speed of the ball, raising exponentially
    float v0z = exp(std::min(float(pressing_time), 1.0f) * 1.5) * 1.8f;

    float v0x = v0y * tan(shooting_angle);
    ball.v = vec3(v0x, -v0y, v0z);
}


void simulation_new_position(ball_structure& ball, std::vector<pole_structure> poles, float dt) {

    vec3 const g = { 0,0,-9.81f };
    vec3 const f = ball.m * g;


    ball.v = (1 - 0.9f * dt) * ball.v + dt * f;
    ball.p = ball.p + dt * ball.v;


    float alpha = 0.9;
    float beta = 0.9;

    vec3 n = vec3(0,0,1);
    vec3 a = vec3(0,0,0);
    float detection = dot(ball.p - a, n);
    if (detection <= ball.r) {
        vec3 v_ortho = dot(ball.v, n) * n;
        vec3 v_para = ball.v - dot(ball.v, n) * n;

        ball.v = alpha * v_para - beta * v_ortho;
        float penet_dist = ball.r - dot((ball.p - a), n);
        ball.p = ball.p + penet_dist * n;
    }

    for (int i = 0; i < poles.size(); i++) {
        vec3 direction_cylinder = poles[i].p1 - poles[i].p0;
        vec3 projection = poles[i].p0 + dot(ball.p - poles[i].p0, direction_cylinder) / dot(direction_cylinder, direction_cylinder) * direction_cylinder;
        float distance = norm(ball.p - projection);
        if (distance <= ball.r) {
            vec3 u = (ball.p - projection) / norm(ball.p - projection);
            vec3 vk = ball.v;
            vec3 vi = {0,0,1};
            ball.v = alpha * vk + beta * (2*poles[i].m/(ball.m + poles[i].m)*(dot(vi - vk, u)) * u);
            float depth = ball.r + poles[i].r - norm(ball.p - projection);
            ball.p = ball.p + (depth/2 * u);
        }
    }

    // Extreme side of the goal
    if (ball.p.y < -0.1 and ball.p.z < 1 and abs(ball.p.x) > 0.28 and abs(ball.p.x) < 0.7) {
        ball.v.y /= 1.5;
        if (ball.p.x < 0)
            ball.v.x += 0.025f;
        else
            ball.v.x -= 0.025f;
    }

    // Normal/middle goal handling
    else if (ball.p.y < -0.14 and ball.p.z < 1 and abs(ball.v.y) > 0.0001f and abs(ball.p.x) < 0.7) {
        ball.v.y /= 1.5;
    }

}


bool simulation_detect_divergence(cloth_structure const& cloth)
{
    bool simulation_diverged = false;
    const size_t N = cloth.position.size();
    for (size_t k = 0; simulation_diverged == false && k < N; ++k)
    {
        const float f = norm(cloth.force.data.at_unsafe(k));
        const vec3& p = cloth.position.data.at_unsafe(k);

        if (std::isnan(f)) // detect NaN in force
        {
            std::cout << "\n **** NaN detected in forces" << std::endl;
            simulation_diverged = true;
        }

        if (f > 600.0f) // detect strong force magnitude
        {
            std::cout << "\n **** Warning : Strong force magnitude detected " << f << " at vertex " << k << " ****" << std::endl;
            simulation_diverged = true;
        }

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) // detect NaN in position
        {
            std::cout << "\n **** NaN detected in positions" << std::endl;
            simulation_diverged = true;
        }
    }

    return simulation_diverged;
}


#pragma once

#include "cgp/cgp.hpp"
#include "../cloth/cloth.hpp"
#include "../constraint/constraint.hpp"


struct simulation_parameters
{
    float dt = 0.005f;        // time step for the numerical integration
    float mass_total = 0.5f; // total mass of the cloth
    float K = 5.0f;         // stiffness parameter
    float mu = 15.0f;        // damping parameter
};

struct ball_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v = {0,0,0}; // Speed

    float r;     // Radius
    float m = 1.0f;     // mass
};

struct pole_structure
{
    float r;
    cgp::vec3 p0;
    cgp::vec3 p1;

    float m = 100.0f;

    pole_structure(float r_, cgp::vec3 p0_, cgp::vec3 p1_) {
        r = r_;
        p0 = p0_;
        p1 = p1_;
    }
};


// Fill the forces in the cloth given the position and velocity
void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters);

// Perform 1 step of a semi-implicit integration with time step dt
void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt);

// Apply the constraints (fixed position, obstacles) on the cloth position and velocity
void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint, ball_structure& ball);

// Give the initial speed of the ball thanks to the space button pressing_time
void simulation_power_shoot(ball_structure& ball, float pressing_time, float shooting_angle);

void simulation_new_position(ball_structure& ball, std::vector<pole_structure> poles, float dt);

// Helper function that tries to detect if the simulation diverged 
bool simulation_detect_divergence(cloth_structure const& cloth);
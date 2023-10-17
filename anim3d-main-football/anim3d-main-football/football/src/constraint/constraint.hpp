#pragma once

#include "cgp/cgp.hpp"
#include "../cloth/cloth.hpp"

// Parameters of the colliding sphere (center, radius)
struct sphere_parameter {
    cgp::vec3 center;
    float radius;
};

// Parameter attached to a fixed vertex (ku,kv) coordinates + 3D position
struct position_contraint {
    int ku;
    int kv;
    cgp::vec3 position;

    position_contraint(int ku_, int kv_, cgp::vec3 pos_) {
        ku = ku_;
        kv = kv_;
        position = pos_;
    }
};


struct constraint_structure
{
    float ground_z = 0.0f;                                   // Height of the flood
    sphere_parameter sphere = { {0.1f, 0.5f, 0.0f}, 0.15f}; // Colliding sphere

    std::vector<position_contraint> fixed_sample; // Storage of all fixed position of the cloth

    // Add a new fixed position
    void add_fixed_position(int ku, int kv, vec3 position);
    // Remove a fixed position
    void remove_fixed_position(int ku, int kv);

};
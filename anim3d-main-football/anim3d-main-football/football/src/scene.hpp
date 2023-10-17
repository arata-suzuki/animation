#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"

#include "cloth/cloth.hpp"
#include "simulation/simulation.hpp"

using cgp::mesh_drawable;


struct gui_parameters {
	bool display_frame = false;
	bool display_wireframe = false;
	int N_sample_edge = 22;  // number of samples of the cloth (the total number of vertices is N_sample_edge^2)
};

// The structure of the custom scene
struct scene_structure : scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	cgp::timer_basic timer;

	// Display of the obstacles and constraints
	cgp::mesh_drawable obstacle_floor;
	cgp::mesh_drawable obstacle_sphere;
    ball_structure ball;
	cgp::mesh_drawable sphere_fixed_position;
    cgp::mesh_drawable cylinder_left;
    cgp::mesh_drawable cylinder_right;
    cgp::mesh_drawable cylinder_top;

    cgp::mesh_drawable preview_sphere;
    cgp::mesh_drawable preview_arrow;
	cgp::mesh_drawable preview_cone;

    cgp::mesh_drawable target_disk;
    cgp::mesh_drawable valid_shot;

	std::vector<vec3> target_positions = {{0,0,0.5}, {0.4,0,0.5}, {-0.4,0,0.5}, {0.4,0,0.2}, {-0.4,0,0.2}, {0,0,0.7}};
	int target_i = 0;

    //Goal structure
    pole_structure cyl_left = pole_structure(0.05f, vec3(0.7f, 0, 0), vec3(0.7f,0,1));
    pole_structure cyl_right = pole_structure(0.05f, {-0.7f, 0, 0}, { -0.7f,0,1 });
    pole_structure cyl_top = pole_structure(0.05f, {-0.73f, 0, 0.97}, { 0.73f,0,0.97});
    std::vector<pole_structure> list_cylinder = {cyl_left, cyl_right, cyl_top};

	// Cloth related structures
	cloth_structure cloth;                     // The values of the position, velocity, forces, etc, stored as a 2D grid
	cloth_structure_drawable cloth_drawable;   // Helper structure to display the cloth as a mesh
	simulation_parameters parameters;          // Stores the parameters of the simulation (stiffness, mass, damping, time step, etc)
	constraint_structure constraint;           // Handle the parameters of the constraints (fixed vertices, floor and sphere)



    // Hierarchy for the arrow showing the direction of the ball
    cgp::hierarchy_mesh_drawable hierarchy;

	// Helper variables
	bool simulation_running = true;   // Boolean indicating if the simulation should be computed
	cgp::opengl_texture_image_structure cloth_texture;             // Storage of the texture ID used for the cloth

    float t_space = 0.0f;
    float t_enter = 0.0f;
    float shooting_angle = 0.0f;    // Initial angle of the ball (x axis)



	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();    // Standard initialization to be called before the animation loop
	void display_frame(); // The frame display to be called within the animation loop
	void display_gui();   // The display of the GUI, also called within the animation loop


	void initialize_cloth(int N_sample); // Recompute the cloth from scratch

	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

};






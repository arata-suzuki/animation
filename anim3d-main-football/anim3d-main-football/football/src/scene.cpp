#include "scene.hpp"


using namespace cgp;



void scene_structure::initialize()
{
    camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
    camera_control.set_rotation_axis_z();
    camera_control.look_at({ 0.0f, 3.5f, 1.5f }, {0,0,0}, {0,0,1});
    global_frame.initialize_data_on_gpu(mesh_primitive_frame());

    obstacle_floor.initialize_data_on_gpu(mesh_primitive_quadrangle({ -3.0f,-3.0f,0 }, { -3.0f,3.0f,0 }, { 3.0f,3.0f,0 }, { 3.0f,-3.0f,0 }));
    obstacle_floor.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/grass.jpg");
    obstacle_floor.model.translation = { 0,0,constraint.ground_z };
    obstacle_floor.material.texture_settings.two_sided = true;

    obstacle_sphere.initialize_data_on_gpu(mesh_primitive_sphere());
    obstacle_sphere.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/ball.jpg");
    vec3 init_pos = {0.0f,1.5f,0.15f};
    ball.p = init_pos;
    obstacle_sphere.model.translation = init_pos;
    obstacle_sphere.model.scaling = constraint.sphere.radius;
    ball.r = constraint.sphere.radius;

    preview_sphere.initialize_data_on_gpu(mesh_primitive_sphere());
    preview_sphere.model.translation = {0, 1.2, 0.15f};
    preview_sphere.model.scaling = 0.06f;
    preview_sphere.material.color = {1,1,0};
    preview_arrow.initialize_data_on_gpu(mesh_primitive_cylinder(0.03f, {0, 0, 0}, {0, -0.4f, 0}));
    preview_arrow.material.color = {1,1,0};
    preview_cone.initialize_data_on_gpu(mesh_primitive_cone(0.06f, 0.1, {0, 0, 0}, {0, -1, 0}));
    preview_cone.material.color = {1,1,0};

    hierarchy.add(preview_sphere, "Sphere base");
    hierarchy.add(preview_arrow, "Arrow", "Sphere base", {0,1.14f,0.15f});
    hierarchy.add(preview_cone, "Cone", "Arrow", {0,-0.4,0});

    // Get the system time.
    unsigned seed = time(0);

    // Seed the random number generator.
    srand(seed);
    int r = std::rand();

    target_i = r % target_positions.size();
    target_disk.initialize_data_on_gpu(mesh_primitive_disc(0.2, target_positions[target_i], {0,1,0}));
    target_disk.material.color = {1,0.6,0};
    target_disk.material.alpha = 0.2;

    valid_shot.initialize_data_on_gpu(mesh_primitive_sphere(0.05, {0.85, 0, 0.9}));



    sphere_fixed_position.initialize_data_on_gpu(mesh_primitive_sphere());
    sphere_fixed_position.model.scaling = 0.02f;
    sphere_fixed_position.material.color = { 0,0,1 };

    //The goal
    cylinder_left.initialize_data_on_gpu(mesh_primitive_cylinder(0.05f, {0.7f, 0, 0}, { 0.7f,0,1 }));
    cylinder_right.initialize_data_on_gpu(mesh_primitive_cylinder(0.05f, {-0.7f, 0, 0}, { -0.7f,0,1 }));
    cylinder_top.initialize_data_on_gpu(mesh_primitive_cylinder(0.05f, {-0.73f, 0, 0.97}, { 0.73f,0,0.97}));


    cloth_texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/filets.jpg");
    initialize_cloth(gui.N_sample_edge);
}

// Compute a new cloth in its initial position (can be called multiple times)
void scene_structure::initialize_cloth(int N_sample)
{
    cloth.initialize(N_sample);
    cloth_drawable.initialize(N_sample);
    cloth_drawable.drawable.texture = cloth_texture;
    cloth_drawable.drawable.material.texture_settings.two_sided = true;

    constraint.fixed_sample.clear();

    //The Goal
    constraint.add_fixed_position(0, 0, { -0.7f,0,1}); //Front right top
    constraint.add_fixed_position(N_sample - 1, 0, { -0.7f,0,0}); //Front right bottom
    constraint.add_fixed_position(N_sample - 1, 5, { -0.7f,-0.4,0}); //Back right bottom
    //constraint.add_fixed_position(0, 5, { -0.7f,-0.2,1}); //Back right top

    //constraint.add_fixed_position(0, N_sample - 6, { 0.7f,-0.2,1}); //Back left top
    constraint.add_fixed_position(N_sample - 1, N_sample - 6, { 0.7f,-0.4,0}); //Back left bottom
    constraint.add_fixed_position(N_sample - 1, N_sample - 1, { 0.7f,0,0}); //Front left bottom
    constraint.add_fixed_position(0, N_sample - 1, { 0.7f,0,1}); //Front left top

    constraint.add_fixed_position(0, (N_sample - 1) / 2, { 0.0f,0,1}); //Front middle

    constraint.add_fixed_position((N_sample - 1) / 4, 0, { -0.7f,0,0.75}); //Front right 1/4
    constraint.add_fixed_position((N_sample - 1) / 2, 0, { -0.7f,0,0.5}); //Front right middle
    constraint.add_fixed_position(N_sample - 1 - (N_sample - 1) / 4, 0, { -0.7f,0,0.25}); //Front right 3/4


    constraint.add_fixed_position((N_sample - 1) / 4, N_sample - 1, { 0.7f,0,0.75}); //Front left 1/4
    constraint.add_fixed_position((N_sample - 1) / 2, N_sample - 1, { 0.7f,0,0.5}); //Front left middle
    constraint.add_fixed_position(N_sample - 1 - (N_sample - 1) / 4, N_sample - 1, { 0.7f,0,0.25}); //Front left 3/4


    constraint.add_fixed_position(5, N_sample - 1 - 5, { 0.6f,-0.2,0.85});
    constraint.add_fixed_position(5, 5, { -0.6f,-0.2,0.85});

}

void scene_structure::display_frame()
{
    // Set the light to the current position of the camera
    environment.light = camera_control.camera_model.position();

    if (gui.display_frame)
        draw(global_frame, environment);

    // Update the current time
    timer.update();


    // Elements of the scene: Obstacles (floor, sphere), and fixed position
    // ***************************************** //

    draw(obstacle_floor, environment);
    draw(cylinder_left, environment);
    draw(cylinder_right, environment);
    draw(cylinder_top, environment);
    draw(target_disk, environment);


    if (t_enter > 0.0f) {
        float max_time = 2.0f;  // max pressing time for angle
        float max_angle = 45.0f;  // max angle in (degrés)
        float pressing_time_dir = glfwGetTime() - t_enter;
        float angle = (pressing_time_dir / max_time) * max_angle;

        // Use a sinusoidal function to create a sweeping motion
        float sweep_factor = sin(2 * M_PI * pressing_time_dir / max_time);

        angle = angle * sweep_factor;
        
        // Make sure angle is within the desired range
        angle = std::max(-max_angle, std::min(angle, max_angle));
        
        float shooting_angle_preview =  angle * (M_PI / 180.0f);
        hierarchy["Arrow"].transform_local.rotation = rotation_transform::from_axis_angle({0,0,1}, shooting_angle_preview);
    }

    // This function must be called before the drawing in order to propagate the deformations through the hierarchy
    hierarchy.update_local_to_global_coordinates();

    // Draw the hierarchy as a single mesh
    draw(hierarchy, environment);
    if (gui.display_wireframe)
        draw_wireframe(hierarchy, environment);

    constraint.sphere.center = obstacle_sphere.model.translation;
    if (abs(ball.v.y) > 0.0001f or abs(ball.v.z) > 0.0001f)
    {
        // Rotation of the ball
        obstacle_sphere.model.rotation = rotation_transform::from_axis_angle({1,0,0}, 2*glfwGetTime());

        float const dt = 0.01f * timer.scale;
        simulation_new_position(ball, list_cylinder, dt);
        obstacle_sphere.model.translation = ball.p;
    }
    else
        obstacle_sphere.model.rotation = rotation_transform::from_axis_angle({1,0,0}, 0);

    if (ball.p.y > -0.02 and ball.p.y < 0.02) {
        vec3 target = target_positions[target_i];
        if (ball.p.x <= target.x + 0.2 and ball.p.x >= target.x - 0.2 and ball.p.z <= target.z + 0.2 and ball.p.z >= target.z - 0.2) {
            valid_shot.material.color = {0,1,0};
        }
        else {
            valid_shot.material.color = {1,0,0};
        }
    }

    draw(valid_shot, environment);
    draw(obstacle_sphere, environment);

    // Draw all fixed point of the goal
    for (auto const& c : constraint.fixed_sample)
    {
        sphere_fixed_position.model.translation = c.position;
        draw(sphere_fixed_position, environment);
    }


    // Simulation of the cloth
    // ***************************************** //
    int const N_step = 1; // Adapt here the number of intermediate simulation steps (ex. 5 intermediate steps per frame)
    for (int k_step = 0; simulation_running == true && k_step < N_step; ++k_step)
    {
        // Update the forces on each particle
        simulation_compute_force(cloth, parameters);

        // One step of numerical integration
        simulation_numerical_integration(cloth, parameters, parameters.dt);

        // Apply the positional (and velocity) constraints
        simulation_apply_constraints(cloth, constraint, ball);

        // Check if the simulation has not diverged - otherwise stop it
        bool const simulation_diverged = simulation_detect_divergence(cloth);
        if (simulation_diverged) {
            std::cout << "\n *** Simulation has diverged ***" << std::endl;
            std::cout << " > The simulation is stoped" << std::endl;
            simulation_running = false;
        }
    }


    // Cloth display
    // ***************************************** //

    // Prepare to display the updated cloth
    cloth.update_normal();        // compute the new normals
    cloth_drawable.update(cloth); // update the positions on the GPU

    // Display the cloth
    glEnable(GL_BLEND); // Color Blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(false); // do not write on depth buffer
    draw(cloth_drawable, environment);
    glDisable(GL_BLEND); // Color BlendingglEnable(GL_BLEND); // Color Blending
    glDepthMask(true);
    if (gui.display_wireframe)
        draw_wireframe(cloth_drawable, environment);


}

void scene_structure::display_gui()
{
    bool reset = false;

    ImGui::Text("Display");
    ImGui::Checkbox("Frame", &gui.display_frame);
    ImGui::Checkbox("Wireframe", &gui.display_wireframe);
    ImGui::Checkbox("Texture Cloth", &cloth_drawable.drawable.material.texture_settings.active);

    ImGui::Spacing(); ImGui::Spacing();

    ImGui::Spacing(); ImGui::Spacing();
    reset |= ImGui::Button("Restart");
    if (reset) {
        obstacle_sphere.model.translation = {0.0f,1.5f,0.15f};
        ball.p = {0.0f,1.5f,0.15f};
        ball.v = {0.0f,0.f,0.f};

        if (valid_shot.material.color.y == 1) {
            target_i = rand() % target_positions.size();
            target_disk.clear();
            target_disk.initialize_data_on_gpu(mesh_primitive_disc(0.2, target_positions[target_i], {0,1,0}));
            target_disk.material.color = {1,0.6,0};
            target_disk.material.alpha = 0.2;
        }


        valid_shot.material.color = {1,1,1};

        initialize_cloth(gui.N_sample_edge);
        simulation_running = true;
    }
}

void scene_structure::mouse_move_event()
{
    if (!inputs.keyboard.shift)
        camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
    camera_control.action_mouse_click(environment.camera_view);
}


void scene_structure::keyboard_event()
{
    camera_control.action_keyboard(environment.camera_view);


    float max_time = 2.0f;  // max pressing time for angle
    float max_angle = 45.0f;  // max angle in (degrés)
    
    //Handling of the direction
    if (inputs.keyboard.enter) {
        if (t_enter == 0.0f)
            t_enter = glfwGetTime();
    }
    else if (t_enter != 0.0f) {
        float pressing_time_dir = glfwGetTime() - t_enter;
        float angle = (pressing_time_dir / max_time) * max_angle;

        // Use a sinusoidal function to create a sweeping motion
        float sweep_factor = sin(2 * M_PI * pressing_time_dir / max_time);

        angle = angle * sweep_factor;
        
        // Make sure angle is within the desired range
        angle = std::max(-max_angle, std::min(angle, max_angle));
        
        shooting_angle = angle * (M_PI / 180.0f); // convert angle in (radians)
        t_enter = 0.0f;
    }
    
    //Handling of the power shoot
    if (inputs.keyboard.space) {
        if (t_space == 0.0f)
            t_space = glfwGetTime();
    }
    else if (t_space != 0.0f) {
        float pressing_time = glfwGetTime() - t_space;
        simulation_power_shoot(ball, pressing_time, shooting_angle);
        t_space = 0.0f;
    }

}
void scene_structure::idle_frame()
{
    camera_control.idle_frame(environment.camera_view);
}


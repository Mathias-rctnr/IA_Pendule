#pragma once

#include <CSFML/Graphics.h>
#include <stdbool.h>

typedef struct
{
    // physics parameters
    float       length;
    float       gravity;
    float       damping_ps;
    float       max_speed_factor;
    float       base_k;
    float       base_d;
    float       max_base_speed;

    // state
    sfVector2f  pivot;
    float       pivot_vel_x;
    sfVector2f  bob_pos;
    bool        first_frame;
    float       theta;
    float       omega;
    int         external_control;
    float       base_vel_cmd;

    // slider (controls pivot.x)
    float       slider_value; // 0..1
    bool        slider_drag;
    bool        bob_drag;
    float       track_width;
    float       track_height;
    float       track_y;
    float       track_left;
    float       thumb_radius;

    // visuals
    sfConvexShape*     base_rect;
    sfRectangleShape*  rod;
    sfCircleShape*     pivot_shape;
    sfCircleShape*     bob_shape;
    sfRectangleShape*  slider_track;
    sfCircleShape*     slider_thumb;
} Pendulum;

bool  pendulum_init(Pendulum* p, sfVector2u window_size);
void  pendulum_handle_event(Pendulum* p, const sfEvent* event);
void  pendulum_update(Pendulum* p, float dt);
void  pendulum_draw(const Pendulum* p, sfRenderWindow* window);
void  pendulum_destroy(Pendulum* p);
void  pendulum_set_external_control(Pendulum* p, int enabled);
void  pendulum_set_base_velocity(Pendulum* p, float v);
void  pendulum_reset(Pendulum* p);
void  pendulum_get_state(const Pendulum* p, float* theta, float* omega, float* pivot_x);
void  pendulum_get_inputs(const Pendulum* p, float* position, float* dirx, float* diry, float* omega);

// Control parameters (e.g., from GA)
typedef struct
{
    float kp;
    float kd;
} PendulumController;

void pendulum_set_controller(Pendulum* p, PendulumController ctrl);

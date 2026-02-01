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

    // state
    sfVector2f  pivot;
    float       pivot_vel_x;
    sfVector2f  bob_pos;
    bool        first_frame;
    float       theta;
    float       omega;

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

// Control parameters (e.g., from GA)
typedef struct
{
    float kp;
    float kd;
} PendulumController;

void pendulum_set_controller(Pendulum* p, PendulumController ctrl);

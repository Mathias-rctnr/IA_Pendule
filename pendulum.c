#include "pendulum.h"

#include <math.h>
#include <stdlib.h>

static sfConvexShape* createRoundedRect(sfVector2f size, float radius, unsigned cornerPts)
{
    const unsigned count = cornerPts * 4;
    sfConvexShape* s = sfConvexShape_create();
    if (!s)
        return NULL;
    sfConvexShape_setPointCount(s, count);

    const sfVector2f centers[4] = {
        {size.x - radius, radius},          // top-right
        {size.x - radius, size.y - radius}, // bottom-right
        {radius, size.y - radius},          // bottom-left
        {radius, radius}                    // top-left
    };
    const float startAngles[4] = {-1.5708f, 0.f, 1.5708f, 3.1416f};

    unsigned idx = 0;
    for (int c = 0; c < 4; ++c)
    {
        for (unsigned p = 0; p < cornerPts; ++p, ++idx)
        {
            const float a  = startAngles[c] + (p / (float)(cornerPts - 1)) * 1.5708f;
            const float ca = cosf(a);
            const float sa = sinf(a);
            sfVector2f pt  = {centers[c].x + radius * ca, centers[c].y + radius * sa};
            sfConvexShape_setPoint(s, idx, pt);
        }
    }
    return s;
}

static sfColor make_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    sfColor c = {r, g, b, a};
    return c;
}

bool pendulum_init(Pendulum* p, sfVector2u window_size)
{
    if (!p)
        return false;

    // Parameters
    p->length           = 200.f;
    p->gravity          = 981.f;
    p->damping_ps       = 0.06f;
    p->max_speed_factor = 12.0f;  // rad/s cap
    p->base_k           = 100.f;   // base spring (1/s^2)
    p->base_d           = 12.f;   // base damping (1/s)
    p->max_base_speed   = 600.f;  // px/s cap for GA control
    p->first_frame      = true;

    // Slider setup
    p->track_width  = 800.f;
    p->track_height = 8.f;
    p->thumb_radius = 12.f;
    p->track_y      = window_size.y - 80.f;
    p->track_left   = (window_size.x - p->track_width) / 2.f;
    p->slider_value = 0.5f;
    p->slider_drag  = false;
    p->bob_drag     = false;

    // Initial pivot/bob
    p->pivot      = (sfVector2f){window_size.x / 2.f, window_size.y / 2.f};
    p->pivot_vel_x = 0.f;
    p->external_control = 0;
    p->base_vel_cmd = 0.f;

    p->theta = -0.7f;
    p->omega = 0.f;
    p->bob_pos = (sfVector2f){
        p->pivot.x + p->length * sinf(p->theta),
        p->pivot.y + p->length * cosf(p->theta)};

    // Shapes
    p->base_rect = createRoundedRect((sfVector2f){900.f, 12.f}, 6.f, 12);
    if (!p->base_rect)
        return false;
    sfConvexShape_setFillColor(p->base_rect, make_color(0x1B, 0x1F, 0x24, 0xFF));
    sfConvexShape_setOutlineThickness(p->base_rect, 1.f);
    sfConvexShape_setOutlineColor(p->base_rect, make_color(0x2D, 0x33, 0x3B, 0xFF));
    sfConvexShape_setPosition(p->base_rect,
                              (sfVector2f){(window_size.x - 900.f) / 2.f,
                                           (window_size.y - 12.f) / 2.f});

    p->rod = sfRectangleShape_create();
    p->pivot_shape = sfCircleShape_create();
    p->bob_shape   = sfCircleShape_create();
    p->slider_track = sfRectangleShape_create();
    p->slider_thumb = sfCircleShape_create();
    if (!p->rod || !p->pivot_shape || !p->bob_shape || !p->slider_track || !p->slider_thumb)
        return false;

    // Rod
    sfRectangleShape_setSize(p->rod, (sfVector2f){p->length, 4.f});
    sfRectangleShape_setOrigin(p->rod, (sfVector2f){0.f, 2.f});
    sfRectangleShape_setFillColor(p->rod, make_color(0xC9, 0xD1, 0xD9, 0xFF));
    sfRectangleShape_setPosition(p->rod, p->pivot);

    // Pivot circle
    const float pivot_r = 18.f;
    sfCircleShape_setRadius(p->pivot_shape, pivot_r);
    sfCircleShape_setOrigin(p->pivot_shape, (sfVector2f){pivot_r, pivot_r});
    sfCircleShape_setFillColor(p->pivot_shape, make_color(0xF4, 0xEE, 0x2A, 0xFF));
    sfCircleShape_setOutlineThickness(p->pivot_shape, 2.f);
    sfCircleShape_setOutlineColor(p->pivot_shape, make_color(0xC9, 0xB9, 0x1A, 0xFF));
    sfCircleShape_setPosition(p->pivot_shape, p->pivot);

    // Bob circle
    const float bob_r = 16.f;
    sfCircleShape_setRadius(p->bob_shape, bob_r);
    sfCircleShape_setOrigin(p->bob_shape, (sfVector2f){bob_r, bob_r});
    sfCircleShape_setFillColor(p->bob_shape, make_color(0xF4, 0xEE, 0x2A, 0xFF));
    sfCircleShape_setOutlineThickness(p->bob_shape, 2.f);
    sfCircleShape_setOutlineColor(p->bob_shape, make_color(0xC9, 0xB9, 0x1A, 0xFF));
    sfCircleShape_setPosition(p->bob_shape, p->bob_pos);

    // Slider visuals
    sfRectangleShape_setSize(p->slider_track, (sfVector2f){p->track_width, p->track_height});
    sfRectangleShape_setPosition(p->slider_track, (sfVector2f){p->track_left, p->track_y});
    sfRectangleShape_setFillColor(p->slider_track, make_color(0x2A, 0x2F, 0x36, 0xFF));

    sfCircleShape_setRadius(p->slider_thumb, p->thumb_radius);
    sfCircleShape_setOrigin(p->slider_thumb, (sfVector2f){p->thumb_radius, p->thumb_radius});
    sfCircleShape_setFillColor(p->slider_thumb, make_color(0xC9, 0xD1, 0xD9, 0xFF));
    sfCircleShape_setPosition(
        p->slider_thumb,
        (sfVector2f){p->track_left + p->slider_value * p->track_width,
                     p->track_y + p->track_height / 2.f});

    return true;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void pendulum_set_controller(Pendulum* p, PendulumController ctrl)
{
    (void)p;
    (void)ctrl; // controller unused in current physics
}

void pendulum_handle_event(Pendulum* p, const sfEvent* event)
{
    if (!p || !event)
        return;
    if (p->external_control)
        return;

    if (event->type == sfEvtMouseButtonPressed && event->mouseButton.button == sfMouseLeft)
    {
        sfVector2i mp = event->mouseButton.position;
        sfVector2f thumbPos = sfCircleShape_getPosition(p->slider_thumb);
        float dx = mp.x - thumbPos.x;
        float dy = mp.y - thumbPos.y;
        if (dx * dx + dy * dy <= p->thumb_radius * p->thumb_radius * 1.2f)
            p->slider_drag = true;

        sfVector2f bobPos = p->bob_pos;
        float bdx = mp.x - bobPos.x;
        float bdy = mp.y - bobPos.y;
        const float pickRadius = 1.5f * sfCircleShape_getRadius(p->bob_shape);
        if (bdx * bdx + bdy * bdy <= pickRadius * pickRadius)
            p->bob_drag = true;
    }
    if (event->type == sfEvtMouseButtonReleased && event->mouseButton.button == sfMouseLeft)
    {
        p->slider_drag = false;
        p->bob_drag    = false;
    }
    if (event->type == sfEvtMouseMoved && p->slider_drag)
    {
        float mx = event->mouseMove.position.x;
        p->slider_value = clampf((mx - p->track_left) / p->track_width, 0.f, 1.f);
    }
    if (event->type == sfEvtMouseMoved && p->bob_drag)
    {
        sfVector2f mp = {(float)event->mouseMove.position.x, (float)event->mouseMove.position.y};
        sfVector2f rel = {mp.x - p->pivot.x, mp.y - p->pivot.y};
        float dist = hypotf(rel.x, rel.y);
        if (dist < 1e-6f)
            dist = 1e-6f;
        float inv = p->length / dist;
        rel.x *= inv;
        rel.y *= inv;
        p->theta     = atan2f(rel.x, rel.y);
        p->omega     = 0.f;
        p->bob_pos   = (sfVector2f){p->pivot.x + rel.x, p->pivot.y + rel.y};
        sfCircleShape_setPosition(p->bob_shape, p->bob_pos);
    }
}

void pendulum_update(Pendulum* p, float dt)
{
    if (!p)
        return;

    if (p->first_frame)
    {
        dt = 0.f;
        p->first_frame = false;
    }
    if (dt > 0.02f)
        dt = 0.02f;

    // If GA is active, integrate its velocity command into the slider target
    if (p->external_control && dt > 0.f)
    {
        float cmd = clampf(p->base_vel_cmd, -p->max_base_speed, p->max_base_speed);
        float delta = (cmd * dt) / p->track_width; // px/s -> slider units
        p->slider_value = clampf(p->slider_value + delta, 0.f, 1.f);
    }

    // Slider-driven pivot (horizontal) with spring-damper (same physics for GA and manual)
    float pivot_target_x = p->track_left + p->slider_value * p->track_width;
    float dx = pivot_target_x - p->pivot.x;
    float pivot_acc_x = p->base_k * dx - p->base_d * p->pivot_vel_x;
    p->pivot_vel_x += pivot_acc_x * dt;
    p->pivot.x += p->pivot_vel_x * dt;

    // Clamp pivot to track
    if (p->pivot.x < p->track_left)
    {
        p->pivot.x = p->track_left;
        p->pivot_vel_x = 0.f;
    }
    if (p->pivot.x > p->track_left + p->track_width)
    {
        p->pivot.x = p->track_left + p->track_width;
        p->pivot_vel_x = 0.f;
    }
    sfCircleShape_setPosition(p->pivot_shape, p->pivot);
    sfRectangleShape_setPosition(p->rod, p->pivot);

    if (!p->bob_drag)
    {
        float theta_dd = -(p->gravity / p->length) * sinf(p->theta)
                         - (pivot_acc_x / p->length) * cosf(p->theta)
                         - p->damping_ps * p->omega;
        p->omega += theta_dd * dt;
        if (p->omega > p->max_speed_factor)
            p->omega = p->max_speed_factor;
        if (p->omega < -p->max_speed_factor)
            p->omega = -p->max_speed_factor;
        p->theta += p->omega * dt;
    }

    p->bob_pos.x = p->pivot.x + p->length * sinf(p->theta);
    p->bob_pos.y = p->pivot.y + p->length * cosf(p->theta);
    sfCircleShape_setPosition(p->bob_shape, p->bob_pos);

    // Update rod geometry
    const sfVector2f d = {p->bob_pos.x - p->pivot.x, p->bob_pos.y - p->pivot.y};
    const float len = hypotf(d.x, d.y);
    const float angleDeg = atan2f(d.y, d.x) * 180.f / (float)M_PI;
    sfRectangleShape_setSize(p->rod, (sfVector2f){len, 4.f});
    sfRectangleShape_setOrigin(p->rod, (sfVector2f){0.f, 2.f});
    sfRectangleShape_setPosition(p->rod, p->pivot);
    sfRectangleShape_setRotation(p->rod, angleDeg);

    // Slider thumb
    sfCircleShape_setPosition(
        p->slider_thumb,
        (sfVector2f){p->track_left + p->slider_value * p->track_width,
                     p->track_y + p->track_height / 2.f});
}

void pendulum_draw(const Pendulum* p, sfRenderWindow* window)
{
    if (!p || !window)
        return;
    sfRenderWindow_drawConvexShape(window, p->base_rect, NULL);
    sfRenderWindow_drawRectangleShape(window, p->rod, NULL);
    sfRenderWindow_drawCircleShape(window, p->pivot_shape, NULL);
    sfRenderWindow_drawCircleShape(window, p->bob_shape, NULL);
    sfRenderWindow_drawRectangleShape(window, p->slider_track, NULL);
    sfRenderWindow_drawCircleShape(window, p->slider_thumb, NULL);
}

void pendulum_destroy(Pendulum* p)
{
    if (!p)
        return;
    sfConvexShape_destroy(p->base_rect);
    sfRectangleShape_destroy(p->rod);
    sfCircleShape_destroy(p->pivot_shape);
    sfCircleShape_destroy(p->bob_shape);
    sfRectangleShape_destroy(p->slider_track);
    sfCircleShape_destroy(p->slider_thumb);
}

void pendulum_set_external_control(Pendulum* p, int enabled)
{
    if (!p)
        return;
    p->external_control = enabled ? 1 : 0;
    p->slider_drag = false;
    p->bob_drag = false;
    if (!p->external_control)
        p->base_vel_cmd = 0.f;
}

void pendulum_set_base_velocity(Pendulum* p, float v)
{
    if (!p)
        return;
    p->base_vel_cmd = v;
}

void pendulum_reset(Pendulum* p)
{
    if (!p)
        return;
    p->slider_value = 0.5f;
    p->pivot.x = p->track_left + p->track_width * p->slider_value;
    p->pivot_vel_x = 0.f;
    p->base_vel_cmd = 0.f;
    p->theta = -0.7f;
    p->omega = 0.f;
    p->bob_pos.x = p->pivot.x + p->length * sinf(p->theta);
    p->bob_pos.y = p->pivot.y + p->length * cosf(p->theta);
    sfCircleShape_setPosition(p->pivot_shape, p->pivot);
    sfCircleShape_setPosition(p->bob_shape, p->bob_pos);
    sfCircleShape_setPosition(
        p->slider_thumb,
        (sfVector2f){p->track_left + p->slider_value * p->track_width,
                     p->track_y + p->track_height / 2.f});
}

void pendulum_get_state(const Pendulum* p, float* theta, float* omega, float* pivot_x)
{
    if (!p)
        return;
    if (theta)
        *theta = p->theta;
    if (omega)
        *omega = p->omega;
    if (pivot_x)
        *pivot_x = p->pivot.x;
}

void pendulum_get_inputs(const Pendulum* p, float* position, float* dirx, float* diry, float* omega)
{
    if (!p)
        return;
    if (position)
    {
        float norm = (p->pivot.x - p->track_left) / p->track_width;
        *position = norm * 2.f - 1.f; // [-1, 1]
    }
    if (dirx)
        *dirx = sinf(p->theta);
    if (diry)
        *diry = cosf(p->theta);
    if (omega)
        *omega = p->omega;
}

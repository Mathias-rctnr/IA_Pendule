#include <CSFML/Graphics.h>
#include <CSFML/Window.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "pendulum.h"
#include "ga.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void draw_network(sfRenderWindow* window, const GAContext* ga, sfVector2u win_size)
{
    if (!window || !ga || !ga->has_champion)
        return;

    const Genome* g = &ga->champion;
    const int in_count = GA_INPUTS;
    const int hid_count = g->hidden;
    const int out_count = 1;

    const float panel_w = 260.f;
    const float panel_h = 200.f + (hid_count > 4 ? (hid_count - 4) * 12.f : 0.f);
    const float panel_x = (float)win_size.x - panel_w - 20.f;
    const float panel_y = 20.f;

    sfRectangleShape* bg = sfRectangleShape_create();
    sfRectangleShape_setSize(bg, (sfVector2f){panel_w, panel_h});
    sfRectangleShape_setPosition(bg, (sfVector2f){panel_x, panel_y});
    sfRectangleShape_setFillColor(bg, (sfColor){0x00, 0x00, 0x00, 0x55});
    sfRenderWindow_drawRectangleShape(window, bg, NULL);
    sfRectangleShape_destroy(bg);

    const float left_x = panel_x + 25.f;
    const float right_x = panel_x + panel_w - 25.f;
    const float mid_x = (left_x + right_x) * 0.5f;

    const float in_top = panel_y + 20.f;
    const float in_bottom = panel_y + panel_h - 20.f;
    const float hid_top = panel_y + 20.f;
    const float hid_bottom = panel_y + panel_h - 20.f;

    sfVector2f in_pos[GA_INPUTS];
    sfVector2f hid_pos[GA_MAX_HIDDEN];
    sfVector2f out_pos = {right_x, (in_top + in_bottom) * 0.5f};

    for (int i = 0; i < in_count; ++i)
    {
        float t = (in_count == 1) ? 0.5f : (float)i / (float)(in_count - 1);
        in_pos[i] = (sfVector2f){left_x, in_top + t * (in_bottom - in_top)};
    }
    for (int i = 0; i < hid_count; ++i)
    {
        float t = (hid_count == 1) ? 0.5f : (float)i / (float)(hid_count - 1);
        hid_pos[i] = (sfVector2f){mid_x, hid_top + t * (hid_bottom - hid_top)};
    }

    // links: input -> hidden
    for (int h = 0; h < hid_count; ++h)
    {
        for (int i = 0; i < in_count; ++i)
        {
            float w = g->w_in[h][i];
            float a = clampf(fabsf(w), 0.f, 1.f);
            uint8_t alpha = (uint8_t)(40 + 180 * a);
            sfColor col = (w >= 0.f) ? (sfColor){0x6A, 0xE3, 0x74, alpha}
                                     : (sfColor){0xFF, 0x66, 0x66, alpha};
            sfVertex line[2] = {
                { .position = in_pos[i], .color = col },
                { .position = hid_pos[h], .color = col }
            };
            sfRenderWindow_drawPrimitives(window, line, 2, sfLines, NULL);
        }
    }

    // links: hidden -> output
    for (int h = 0; h < hid_count; ++h)
    {
        float w = g->w_out[h];
        float a = clampf(fabsf(w), 0.f, 1.f);
        uint8_t alpha = (uint8_t)(40 + 180 * a);
        sfColor col = (w >= 0.f) ? (sfColor){0x6A, 0xE3, 0x74, alpha}
                                 : (sfColor){0xFF, 0x66, 0x66, alpha};
        sfVertex line[2] = {
            { .position = hid_pos[h], .color = col },
            { .position = out_pos, .color = col }
        };
        sfRenderWindow_drawPrimitives(window, line, 2, sfLines, NULL);
    }

    // nodes
    sfCircleShape* node = sfCircleShape_create();
    sfCircleShape_setRadius(node, 5.f);
    sfCircleShape_setOrigin(node, (sfVector2f){5.f, 5.f});
    sfCircleShape_setFillColor(node, (sfColor){0xF4, 0xEE, 0x2A, 0xFF});

    for (int i = 0; i < in_count; ++i)
    {
        sfCircleShape_setPosition(node, in_pos[i]);
        sfRenderWindow_drawCircleShape(window, node, NULL);
    }
    for (int i = 0; i < hid_count; ++i)
    {
        sfCircleShape_setPosition(node, hid_pos[i]);
        sfRenderWindow_drawCircleShape(window, node, NULL);
    }
    sfCircleShape_setPosition(node, out_pos);
    sfRenderWindow_drawCircleShape(window, node, NULL);
    sfCircleShape_destroy(node);
}

int main(void)
{
    const sfVideoMode mode = {1400, 1050, 32};
    sfRenderWindow* window =
        sfRenderWindow_create(mode, "CSFML Pendulum", sfResize | sfClose, sfWindowed, NULL);
    if (!window)
        return EXIT_FAILURE;

    Pendulum pendulum;
    if (!pendulum_init(&pendulum, mode.size))
    {
        sfRenderWindow_destroy(window);
        return EXIT_FAILURE;
    }

    GAContext ga;
    ga_init(&ga, 150);
    ga_set_env(&ga,
               pendulum.track_left,
               pendulum.track_width,
               pendulum.pivot.y,
               pendulum.length,
               pendulum.base_k,
               pendulum.base_d,
               pendulum.gravity,
               pendulum.damping_ps,
               pendulum.max_speed_factor,
               pendulum.max_base_speed,
               -0.98f);

    sfFont* font = sfFont_createFromFile("tuffy.ttf");
    sfText* info_text = sfText_create(font);
    sfText* button_text = sfText_create(font);
    sfRectangleShape* button = sfRectangleShape_create();
    sfRectangleShape* agent_rod = sfRectangleShape_create();
    sfCircleShape* agent_bob = sfCircleShape_create();
    sfRectangleShape* threshold_line = sfRectangleShape_create();
    sfRectangleShape* chart_bg = sfRectangleShape_create();
    sfVertexArray* chart_line = sfVertexArray_create();
    sfVertexArray* grad_ticks = sfVertexArray_create();
    sfText* grad_text = sfText_create(font);
    if (!font || !info_text || !button_text || !button || !agent_rod || !agent_bob || !threshold_line || !chart_bg ||
        !chart_line || !grad_ticks || !grad_text)
    {
        pendulum_destroy(&pendulum);
        sfRenderWindow_destroy(window);
        return EXIT_FAILURE;
    }
    sfText_setCharacterSize(info_text, 16);
    sfText_setFillColor(info_text, sfWhite);
    sfText_setPosition(info_text, (sfVector2f){20.f, 70.f});

    sfText_setCharacterSize(button_text, 16);
    sfText_setFillColor(button_text, sfWhite);
    sfText_setCharacterSize(grad_text, 12);
    sfText_setFillColor(grad_text, (sfColor){0xF4, 0xEE, 0x2A, 0xFF});
    sfRectangleShape_setSize(button, (sfVector2f){140.f, 36.f});
    sfRectangleShape_setPosition(button, (sfVector2f){20.f, 20.f});
    sfRectangleShape_setFillColor(button, (sfColor){0x2A, 0x2A, 0x2A, 0xFF});

    sfRectangleShape_setSize(agent_rod, (sfVector2f){100.f, 2.f});
    sfRectangleShape_setOrigin(agent_rod, (sfVector2f){0.f, 1.f});
    sfCircleShape_setRadius(agent_bob, 8.f);
    sfCircleShape_setOrigin(agent_bob, (sfVector2f){8.f, 8.f});

    sfRectangleShape_setSize(threshold_line, (sfVector2f){(float)mode.size.x, 2.f});
    sfRectangleShape_setFillColor(threshold_line, (sfColor){0xFF, 0x66, 0x66, 0x80});
    float threshold_y = pendulum.pivot.y + pendulum.length * ga.upright_threshold;
    sfRectangleShape_setPosition(threshold_line, (sfVector2f){0.f, threshold_y});

    sfVertexArray_setPrimitiveType(grad_ticks, sfLines);
    const sfFloatRect base_bounds = sfConvexShape_getGlobalBounds(pendulum.base_rect);
    const float grad_y = base_bounds.position.y + base_bounds.size.y + 25.f;
    const float grad_span = base_bounds.size.x;
    const float grad_left = base_bounds.position.x;
    const int grad_min = -50;
    const int grad_max = 50;
    for (int v = grad_min; v <= grad_max; ++v)
    {
        float t = (float)(v - grad_min) / (float)(grad_max - grad_min);
        float x = grad_left + t * grad_span;
        bool major = (v % 10) == 0;
        float h = major ? 10.f : 5.f;
        sfColor col = major ? (sfColor){0xF4, 0xEE, 0x2A, 0xFF} : (sfColor){0xCC, 0xCC, 0xCC, 0xFF};
        sfVertex v0 = { .position = {x, grad_y}, .color = col };
        sfVertex v1 = { .position = {x, grad_y + h}, .color = col };
        sfVertexArray_append(grad_ticks, v0);
        sfVertexArray_append(grad_ticks, v1);
    }

    const float chart_x = 200.f;
    const float chart_y = 20.f;
    const float chart_w = 220.f;
    const float chart_h = 90.f;
    sfRectangleShape_setSize(chart_bg, (sfVector2f){chart_w, chart_h});
    sfRectangleShape_setPosition(chart_bg, (sfVector2f){chart_x, chart_y});
    sfRectangleShape_setFillColor(chart_bg, (sfColor){0x00, 0x00, 0x00, 0x40});
    sfVertexArray_setPrimitiveType(chart_line, sfLineStrip);

    sfClock* clock = sfClock_create();
    if (!clock)
    {
        pendulum_destroy(&pendulum);
        sfRenderWindow_destroy(window);
        return EXIT_FAILURE;
    }

    const sfColor bg = {0x3e, 0x3e, 0x3e, 0xFF}; // background color

    sfEvent event;
    bool running = true;
    float* history = NULL;
    int history_count = 0;
    int history_cap = 0;
    int last_gen = -1;
    bool fast_mode = false;
    const float fixed_step = 1.f / 120.f;
    float display_accum = 0.f;
    while (running && sfRenderWindow_isOpen(window))
    {
        while (sfRenderWindow_pollEvent(window, &event))
        {
            if (event.type == sfEvtClosed)
                running = false;
            if (event.type == sfEvtKeyPressed && event.key.code == sfKeyF)
            {
                fast_mode = !fast_mode;
                if (ga.running && !fast_mode)
                    ga_reset_agents(&ga);
                display_accum = 0.f;
            }
            if (event.type == sfEvtMouseButtonPressed && event.mouseButton.button == sfMouseLeft)
            {
                sfVector2i mp = event.mouseButton.position;
                sfVector2f bp = sfRectangleShape_getPosition(button);
                sfVector2f bs = sfRectangleShape_getSize(button);
                if (mp.x >= bp.x && mp.x <= bp.x + bs.x && mp.y >= bp.y && mp.y <= bp.y + bs.y)
                {
                    if (!ga.running)
                    {
                        ga_start(&ga);
                        pendulum_set_external_control(&pendulum, 1);
                        pendulum_reset(&pendulum);
                        history_count = 0;
                        last_gen = ga.generation;
                        display_accum = 0.f;
                    }
                    else
                    {
                        ga.running = 0;
                        pendulum_set_external_control(&pendulum, 0);
                        pendulum_set_base_velocity(&pendulum, 0.f);
                        pendulum_reset(&pendulum);
                    }
                }
            }
            if (!ga.running)
                pendulum_handle_event(&pendulum, &event);
        }

        float dt = sfTime_asSeconds(sfClock_restart(clock));
        if (ga.running)
        {
            if (fast_mode)
                ga_run_generation(&ga, fixed_step);
            else
            {
                display_accum += dt;
                while (display_accum >= fixed_step)
                {
                    ga_display_step(&ga, fixed_step);
                    display_accum -= fixed_step;
                }
            }
            if (ga.generation != last_gen)
            {
                last_gen = ga.generation;
                if (history_count == history_cap)
                {
                    history_cap = history_cap ? history_cap * 2 : 64;
                    history = realloc(history, (size_t)history_cap * sizeof(float));
                }
                if (history)
                    history[history_count++] = ga.gen_best_fitness;
                if (fast_mode)
                {
                    printf("[FAST] Gen %d best=%.2f overall=%.2f\n",
                           ga.generation,
                           ga.gen_best_fitness,
                           ga.best_fitness);
                    fflush(stdout);
                }
            }
        }
        else
        {
            pendulum_set_base_velocity(&pendulum, 0.f);
            pendulum_update(&pendulum, dt);
        }

        sfRenderWindow_clear(window, bg);
        if (ga.running && !fast_mode)
        {
            // draw base/track
            sfRenderWindow_drawConvexShape(window, pendulum.base_rect, NULL);
            sfRenderWindow_drawRectangleShape(window, pendulum.slider_track, NULL);
            sfRenderWindow_drawRectangleShape(window, threshold_line, NULL);

            const GAAgent* a = ga_get_display_agent(&ga);
            if (a)
            {
                sfColor col = (sfColor){0xF4, 0xEE, 0x2A, 0xFF};
                sfRectangleShape_setFillColor(agent_rod, col);
                sfCircleShape_setFillColor(agent_bob, col);

                float dx = a->bob_x - a->pivot_x;
                float dy = a->bob_y - pendulum.pivot.y;
                float len = sqrtf(dx * dx + dy * dy);
                float angle = atan2f(dy, dx) * 180.f / (float)M_PI;
                sfRectangleShape_setSize(agent_rod, (sfVector2f){len, 2.f});
                sfRectangleShape_setPosition(agent_rod, (sfVector2f){a->pivot_x, pendulum.pivot.y});
                sfRectangleShape_setRotation(agent_rod, angle);
                sfCircleShape_setPosition(agent_bob, (sfVector2f){a->bob_x, a->bob_y});

                sfRenderWindow_drawRectangleShape(window, agent_rod, NULL);
                sfRenderWindow_drawCircleShape(window, agent_bob, NULL);
            }

            draw_network(window, &ga, mode.size);
        }
        else if (!ga.running)
        {
            pendulum_draw(&pendulum, window);
        }
        else
        {
            sfRenderWindow_drawConvexShape(window, pendulum.base_rect, NULL);
        }
        sfRenderWindow_drawVertexArray(window, grad_ticks, NULL);
        // labels for major ticks
        for (int v = grad_min; v <= grad_max; v += 10)
        {
            float t = (float)(v - grad_min) / (float)(grad_max - grad_min);
            float x = grad_left + t * grad_span;
            char label[8];
            snprintf(label, sizeof(label), "%d", v);
            sfText_setString(grad_text, label);
            sfFloatRect b = sfText_getLocalBounds(grad_text);
            sfText_setPosition(grad_text, (sfVector2f){x - b.size.x * 0.5f, grad_y + 12.f});
            sfRenderWindow_drawText(window, grad_text, NULL);
        }
        if (history_count > 1)
        {
            float minv = history[0], maxv = history[0];
            for (int i = 1; i < history_count; ++i)
            {
                if (history[i] < minv)
                    minv = history[i];
                if (history[i] > maxv)
                    maxv = history[i];
            }
            if (fabsf(maxv - minv) < 1e-6f)
                maxv = minv + 1.f;
            sfVertexArray_clear(chart_line);
            for (int i = 0; i < history_count; ++i)
            {
                float t = (float)i / (float)(history_count - 1);
                float x = chart_x + t * chart_w;
                float y = chart_y + chart_h - (history[i] - minv) / (maxv - minv) * chart_h;
                sfVertex v = { .position = {x, y}, .color = (sfColor){0x6A, 0xE3, 0x74, 0xFF} };
                sfVertexArray_append(chart_line, v);
            }
            sfRenderWindow_drawRectangleShape(window, chart_bg, NULL);
            sfRenderWindow_drawVertexArray(window, chart_line, NULL);
        }
        // UI
        sfText_setString(button_text, ga.running ? "Stop GA" : "Start GA");
        sfVector2f bp = sfRectangleShape_getPosition(button);
        sfText_setPosition(button_text, (sfVector2f){bp.x + 16.f, bp.y + 8.f});
        char info[256];
        char time_left[32];
        if (!fast_mode)
            snprintf(time_left, sizeof(time_left), "%.1fs", ga.eval_duration - ga.eval_time);
        else
            snprintf(time_left, sizeof(time_left), "--");
        const char* stage =
            (ga.stage == GA_STAGE_EVAL) ? "EVAL" : (ga.stage == GA_STAGE_SELECT) ? "SELECT" : "MUTATE";
        const GAAgent* display_agent = ga_get_display_agent(&ga);
        float champ = ga.has_champion ? ga.champion_fitness : ga.best_fitness;
        float display_score = (display_agent ? display_agent->fitness : 0.f);
        char display_buf[32];
        snprintf(display_buf, sizeof(display_buf), "%.2f", display_score);
        if (fast_mode)
        {
            snprintf(info, sizeof(info),
                     "GA: %s  Mode: %s  %s\nGen: %d  Pop: %d\nBest ever: %.2f\nGen best: %.2f\nThr: %.2f\nTime left: %s",
                     ga.running ? "ON" : "OFF",
                     "FAST",
                     stage,
                     ga.generation,
                     ga.population_size,
                     champ,
                     ga.gen_best_fitness,
                     ga.upright_threshold,
                     time_left);
        }
        else
        {
            snprintf(info, sizeof(info),
                     "GA: %s  Mode: %s  %s\nGen: %d  Pop: %d\nDisplay score: %s\nBest ever: %.2f\nThr: %.2f\nTime left: %s",
                     ga.running ? "ON" : "OFF",
                     "DISPLAY",
                     stage,
                     ga.generation,
                     ga.population_size,
                     display_buf,
                     champ,
                     ga.upright_threshold,
                     time_left);
        }
        sfText_setString(info_text, info);
        sfRenderWindow_drawRectangleShape(window, button, NULL);
        sfRenderWindow_drawText(window, button_text, NULL);
        sfRenderWindow_drawText(window, info_text, NULL);
        sfRenderWindow_display(window);
    }

    ga_free(&ga);
    pendulum_destroy(&pendulum);
    sfClock_destroy(clock);
    sfText_destroy(info_text);
    sfText_destroy(button_text);
    sfRectangleShape_destroy(button);
    sfRectangleShape_destroy(agent_rod);
    sfCircleShape_destroy(agent_bob);
    sfRectangleShape_destroy(threshold_line);
    sfRectangleShape_destroy(chart_bg);
    sfVertexArray_destroy(chart_line);
    sfVertexArray_destroy(grad_ticks);
    sfText_destroy(grad_text);
    free(history);
    sfFont_destroy(font);
    sfRenderWindow_destroy(window);
    return EXIT_SUCCESS;
}

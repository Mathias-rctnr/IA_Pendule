#include "ga.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>

static float frand(float a, float b)
{
    return a + (b - a) * ((float)rand() / (float)RAND_MAX);
}

static void init_genome(Genome* g)
{
    g->hidden = 2 + rand() % (GA_MAX_HIDDEN - 1);
    for (int i = 0; i < GA_MAX_HIDDEN; ++i)
    {
        g->b_h[i] = frand(-0.5f, 0.5f);
        g->w_out[i] = frand(-1.f, 1.f);
        for (int j = 0; j < GA_INPUTS; ++j)
            g->w_in[i][j] = frand(-1.f, 1.f);
    }
    g->b_out = frand(-0.5f, 0.5f);
    g->fitness = 0.f;
}

static float eval_network(const Genome* g, const float in[GA_INPUTS])
{
    float h[GA_MAX_HIDDEN];
    for (int i = 0; i < g->hidden; ++i)
    {
        float sum = g->b_h[i];
        for (int j = 0; j < GA_INPUTS; ++j)
            sum += g->w_in[i][j] * in[j];
        h[i] = tanhf(sum);
    }
    float out = g->b_out;
    for (int i = 0; i < g->hidden; ++i)
        out += g->w_out[i] * h[i];
    return tanhf(out);
}

static void reset_agent(GAContext* ga, GAAgent* a)
{
    a->slider_value = 0.5f;
    a->pivot_x = ga->track_left + ga->track_width * 0.5f;
    a->pivot_v = 0.f;
    a->theta = -0.7f;
    a->omega = 0.f;
    a->bob_x = a->pivot_x + ga->length * sinf(a->theta);
    a->bob_y = ga->pivot_y + ga->length * cosf(a->theta);
    a->above_time = 0.f;
    a->last_control = 0.f;
    a->fitness = 0.f;
}

static int cmp_fitness_desc(const void* a, const void* b)
{
    const Genome* ga = (const Genome*)a;
    const Genome* gb = (const Genome*)b;
    return (ga->fitness < gb->fitness) - (ga->fitness > gb->fitness);
}

static void mutate_genome(Genome* g, float sigma, float prob)
{
    if (frand(0.f, 1.f) < 0.15f)
    {
        int delta = (rand() % 3) - 1;
        g->hidden += delta;
        if (g->hidden < 2)
            g->hidden = 2;
        if (g->hidden > GA_MAX_HIDDEN)
            g->hidden = GA_MAX_HIDDEN;
    }
    for (int i = 0; i < GA_MAX_HIDDEN; ++i)
    {
        if (frand(0.f, 1.f) < prob)
            g->b_h[i] += frand(-sigma, sigma);
        if (frand(0.f, 1.f) < prob)
            g->w_out[i] += frand(-sigma, sigma);
        for (int j = 0; j < GA_INPUTS; ++j)
        {
            if (frand(0.f, 1.f) < prob)
                g->w_in[i][j] += frand(-sigma, sigma);
        }
    }
    if (frand(0.f, 1.f) < prob)
        g->b_out += frand(-sigma, sigma);
}

static Genome crossover(const Genome* a, const Genome* b)
{
    Genome c = *a;
    c.hidden = (rand() % 2) ? a->hidden : b->hidden;
    for (int i = 0; i < GA_MAX_HIDDEN; ++i)
    {
        c.b_h[i]   = (rand() % 2) ? a->b_h[i] : b->b_h[i];
        c.w_out[i] = (rand() % 2) ? a->w_out[i] : b->w_out[i];
        for (int j = 0; j < GA_INPUTS; ++j)
            c.w_in[i][j] = (rand() % 2) ? a->w_in[i][j] : b->w_in[i][j];
    }
    c.b_out = (rand() % 2) ? a->b_out : b->b_out;
    c.fitness = 0.f;
    return c;
}

void ga_init(GAContext* ga, int population_size)
{
    if (!ga)
        return;
    srand((unsigned)time(NULL));
    ga->population_size = population_size;
    ga->generation      = 0;
    ga->eval_time       = 0.f;
    ga->eval_duration   = 12.0f;
    ga->running         = 0;
    ga->stage           = GA_STAGE_EVAL;
    ga->best_index      = 0;
    ga->best_fitness    = -1e9f;
    ga->gen_best_fitness = -1e9f;
    ga->max_base_speed  = 600.f;
    ga->upright_threshold = -0.7f;
    ga->population      = calloc((size_t)ga->population_size, sizeof(Genome));
    ga->agents          = calloc((size_t)ga->population_size, sizeof(GAAgent));
    for (int i = 0; i < ga->population_size; ++i)
        init_genome(&ga->population[i]);
}

void ga_set_env(GAContext* ga,
                float track_left,
                float track_width,
                float pivot_y,
                float length,
                float base_k,
                float base_d,
                float gravity,
                float damping,
                float max_speed_factor,
                float max_base_speed,
                float upright_threshold)
{
    if (!ga)
        return;
    ga->track_left = track_left;
    ga->track_width = track_width;
    ga->pivot_y = pivot_y;
    ga->length = length;
    ga->base_k = base_k;
    ga->base_d = base_d;
    ga->gravity = gravity;
    ga->damping = damping;
    ga->max_speed_factor = max_speed_factor;
    ga->max_base_speed = max_base_speed;
    ga->upright_threshold = upright_threshold;
}

void ga_start(GAContext* ga)
{
    if (!ga || !ga->population || !ga->agents)
        return;
    ga->running = 1;
    ga->generation = 0;
    ga->eval_time = 0.f;
    ga->best_fitness = -1e9f;
    ga->gen_best_fitness = -1e9f;
    ga->best_index = 0;
    ga->stage = GA_STAGE_EVAL;
    for (int i = 0; i < ga->population_size; ++i)
    {
        ga->population[i].fitness = 0.f;
        reset_agent(ga, &ga->agents[i]);
    }
}

void ga_update(GAContext* ga, float dt)
{
    if (!ga || !ga->running)
        return;

    if (ga->stage == GA_STAGE_EVAL)
    {
        ga->eval_time += dt;
        ga->best_index = 0;
        float best_now = -1e9f;

        for (int i = 0; i < ga->population_size; ++i)
        {
            GAAgent* a = &ga->agents[i];
            Genome* g = &ga->population[i];

            float inputs[GA_INPUTS];
            inputs[0] = a->slider_value * 2.f - 1.f; // position [-1,1]
            inputs[1] = sinf(a->theta);
            inputs[2] = cosf(a->theta);
            inputs[3] = a->omega;

            float out = eval_network(g, inputs);
            float control = out * ga->max_base_speed;
            a->last_control = control;

            // GA outputs base velocity -> update slider target
            a->slider_value += (control * dt) / ga->track_width;
            if (a->slider_value < 0.f)
                a->slider_value = 0.f;
            if (a->slider_value > 1.f)
                a->slider_value = 1.f;

            float pivot_target_x = ga->track_left + ga->track_width * a->slider_value;
            float dx = pivot_target_x - a->pivot_x;
            float pivot_acc = ga->base_k * dx - ga->base_d * a->pivot_v;
            a->pivot_v += pivot_acc * dt;
            a->pivot_x += a->pivot_v * dt;

            // clamp pivot
            if (a->pivot_x < ga->track_left)
            {
                a->pivot_x = ga->track_left;
                a->pivot_v = 0.f;
            }
            if (a->pivot_x > ga->track_left + ga->track_width)
            {
                a->pivot_x = ga->track_left + ga->track_width;
                a->pivot_v = 0.f;
            }

            float theta_dd = -(ga->gravity / ga->length) * sinf(a->theta)
                             - (pivot_acc / ga->length) * cosf(a->theta)
                             - ga->damping * a->omega;
            a->omega += theta_dd * dt;
            if (a->omega > ga->max_speed_factor)
                a->omega = ga->max_speed_factor;
            if (a->omega < -ga->max_speed_factor)
                a->omega = -ga->max_speed_factor;
            a->theta += a->omega * dt;

            a->bob_x = a->pivot_x + ga->length * sinf(a->theta);
            a->bob_y = ga->pivot_y + ga->length * cosf(a->theta);

            // success condition: above threshold for >1s, then +1 point each extra second (discrete)
            if (cosf(a->theta) < ga->upright_threshold)
            {
                a->above_time += dt;
                if (a->above_time > 1.f)
                {
                    float extra = a->above_time - 1.f;
                    int points = (int)floorf(extra);
                    if (points > 0)
                    {
                        a->fitness += (float)points;
                        a->above_time -= (float)points;
                    }
                }
            }
            else
            {
                a->above_time = 0.f;
            }

            g->fitness = a->fitness;
            if (g->fitness > best_now)
            {
                best_now = g->fitness;
                ga->best_index = i;
            }
        }

        if (ga->eval_time < ga->eval_duration)
            return;
        ga->stage = GA_STAGE_SELECT;
        return;
    }

    if (ga->stage == GA_STAGE_SELECT)
    {
        qsort(ga->population, (size_t)ga->population_size, sizeof(Genome), cmp_fitness_desc);
        ga->gen_best_fitness = ga->population[0].fitness;
        if (ga->gen_best_fitness > ga->best_fitness)
            ga->best_fitness = ga->gen_best_fitness;
        ga->stage = GA_STAGE_MUTATE;
        return;
    }

    if (ga->stage == GA_STAGE_MUTATE)
    {
        int elite = (int)(ga->population_size * 0.3f);
        if (elite < 1)
            elite = 1;
        for (int i = elite; i < ga->population_size; ++i)
            mutate_genome(&ga->population[i], 0.25f, 0.15f);

        // weaker agents get extra mutation
        int weak_start = (int)(ga->population_size * 0.8f);
        for (int i = weak_start; i < ga->population_size; ++i)
            mutate_genome(&ga->population[i], 0.05f, 0.5f);

        ga->generation++;
        ga->eval_time = 0.f;
        ga->stage = GA_STAGE_EVAL;

        for (int i = 0; i < ga->population_size; ++i)
        {
            ga->population[i].fitness = 0.f;
            reset_agent(ga, &ga->agents[i]);
        }
    }
}

const GAAgent* ga_get_agents(const GAContext* ga, int* count, int* best_index)
{
    if (!ga)
        return NULL;
    if (count)
        *count = ga->population_size;
    if (best_index)
        *best_index = ga->best_index;
    return ga->agents;
}

void ga_free(GAContext* ga)
{
    if (!ga)
        return;
    free(ga->population);
    free(ga->agents);
    ga->population = NULL;
    ga->agents = NULL;
}

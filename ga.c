#include "ga.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#define GA_THREAD_COUNT 14

typedef enum
{
    MUTATE_NONE = 0,
    MUTATE_NEW_CONN,
    MUTATE_NEW_NODE,
    MUTATE_WEIGHTS
} MutationKind;

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

static void ga_step_agent(GAContext* ga, GAAgent* a, Genome* g, float dt, int write_fitness)
{
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

    // reward: above threshold + bonus for staying near angle 0
    const float center_range = 0.35f; // radians where bonus is strongest
    const float center_bonus = 0.3f;  // weight of the bonus
    const float drop_penalty = 0.6f;  // penalty when leaving the threshold
    if (cosf(a->theta) < ga->upright_threshold)
    {
        float closeness = 1.f - (fabsf(a->theta) / center_range);
        if (closeness < 0.f)
            closeness = 0.f;
        if (closeness > 1.f)
            closeness = 1.f;

        a->fitness += dt; // base reward for being above threshold
        a->fitness += dt * center_bonus * closeness; // extra reward near 0
        a->above_time += dt;
    }
    else
    {
        if (a->above_time > 0.f)
            a->fitness -= drop_penalty;
        a->above_time = 0.f;
    }

    // penalties: base motion + angular speed (inertia)
    // keep base near center (0)
    {
        const float center_x = ga->track_left + ga->track_width * 0.5f;
        float base_dist = fabsf(a->pivot_x - center_x) / (ga->track_width * 0.5f);
        if (base_dist > 1.f)
            base_dist = 1.f;
        a->fitness -= dt * 0.15f * base_dist;
    }
    a->fitness -= dt * 0.05f * (fabsf(a->pivot_v) / ga->max_base_speed);
    a->fitness -= dt * 0.08f * fabsf(a->omega);
    if (a->fitness < 0.f)
        a->fitness = 0.f;

    if (write_fitness)
        g->fitness = a->fitness;
}

typedef struct
{
    GAContext* ga;
    int start;
    int end;
    float dt;
    int steps;
    float best_fitness;
    int best_index;
} GAWorker;

static void* eval_worker(void* arg)
{
    GAWorker* w = (GAWorker*)arg;
    GAContext* ga = w->ga;
    for (int s = 0; s < w->steps; ++s)
    {
        for (int i = w->start; i < w->end; ++i)
        {
            GAAgent* a = &ga->agents[i];
            Genome* g = &ga->population[i];
            ga_step_agent(ga, a, g, w->dt, 1);
        }
    }

    float best_now = -1e9f;
    int best_idx = w->start;
    for (int i = w->start; i < w->end; ++i)
    {
        float f = ga->population[i].fitness;
        if (f > best_now)
        {
            best_now = f;
            best_idx = i;
        }
    }

    w->best_fitness = best_now;
    w->best_index = best_idx;
    return NULL;
}

static void ga_eval_parallel(GAContext* ga, float dt, int steps)
{
    if (!ga || steps < 1)
        return;

    int thread_count = GA_THREAD_COUNT;
    if (thread_count > ga->population_size)
        thread_count = ga->population_size;
    if (thread_count < 1)
        thread_count = 1;

    pthread_t threads[GA_THREAD_COUNT];
    GAWorker workers[GA_THREAD_COUNT];

    int chunk = ga->population_size / thread_count;
    int remainder = ga->population_size % thread_count;
    int start = 0;
    for (int t = 0; t < thread_count; ++t)
    {
        int size = chunk + (t < remainder ? 1 : 0);
        workers[t].ga = ga;
        workers[t].start = start;
        workers[t].end = start + size;
        workers[t].dt = dt;
        workers[t].steps = steps;
        workers[t].best_fitness = -1e9f;
        workers[t].best_index = start;
        pthread_create(&threads[t], NULL, eval_worker, &workers[t]);
        start += size;
    }
    for (int t = 0; t < thread_count; ++t)
        pthread_join(threads[t], NULL);

    ga->best_index = 0;
    float best_now = -1e9f;
    for (int t = 0; t < thread_count; ++t)
    {
        if (workers[t].best_fitness > best_now)
        {
            best_now = workers[t].best_fitness;
            ga->best_index = workers[t].best_index;
        }
    }
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

static MutationKind pick_mutation_kind(void)
{
    float r = frand(0.f, 1.f);
    if (r < 0.10f)
        return MUTATE_NONE;
    if (r < 0.25f)
        return MUTATE_NEW_CONN;
    if (r < 0.35f)
        return MUTATE_NEW_NODE;
    return MUTATE_WEIGHTS;
}

static void mutate_weights(Genome* g, float sigma, float prob)
{
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

static void mutate_genome(Genome* g, MutationKind kind, float sigma, float prob)
{
    if (!g)
        return;
    switch (kind)
    {
        case MUTATE_NONE:
            break;
        case MUTATE_NEW_CONN:
        {
            int i = rand() % g->hidden;
            if (rand() % 2)
                g->w_out[i] = frand(-1.f, 1.f);
            else
                g->w_in[i][rand() % GA_INPUTS] = frand(-1.f, 1.f);
            break;
        }
        case MUTATE_NEW_NODE:
        {
            if (g->hidden < GA_MAX_HIDDEN)
            {
                int i = g->hidden;
                g->hidden++;
                g->b_h[i] = frand(-0.5f, 0.5f);
                g->w_out[i] = frand(-1.f, 1.f);
                for (int j = 0; j < GA_INPUTS; ++j)
                    g->w_in[i][j] = frand(-1.f, 1.f);
            }
            else
            {
                mutate_weights(g, sigma, prob);
            }
            break;
        }
        case MUTATE_WEIGHTS:
        default:
            mutate_weights(g, sigma, prob);
            break;
    }
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

static void ga_do_select(GAContext* ga)
{
    qsort(ga->population, (size_t)ga->population_size, sizeof(Genome), cmp_fitness_desc);
    ga->gen_best_fitness = ga->population[0].fitness;
    if (ga->gen_best_fitness > ga->best_fitness)
        ga->best_fitness = ga->gen_best_fitness;
    if (ga->gen_best_fitness > ga->champion_fitness)
    {
        ga->champion = ga->population[0];
        ga->champion_fitness = ga->gen_best_fitness;
        ga->has_champion = 1;
        ga->display_active = 0;
    }
}

static void ga_do_mutate(GAContext* ga)
{
    int elite = (int)(ga->population_size * 0.3f);
    if (elite < 1)
        elite = 1;
    for (int i = elite; i < ga->population_size; ++i)
    {
        int p1 = rand() % elite;
        int p2 = rand() % elite;
        Genome child = crossover(&ga->population[p1], &ga->population[p2]);
        ga->population[i] = child;
        MutationKind kind = pick_mutation_kind();
        mutate_genome(&ga->population[i], kind, 0.25f, 0.15f);
    }

    // weaker agents get extra (light) mutation
    int weak_start = (int)(ga->population_size * 0.8f);
    for (int i = weak_start; i < ga->population_size; ++i)
        mutate_genome(&ga->population[i], MUTATE_WEIGHTS, 0.05f, 0.5f);

    ga->generation++;
    ga->eval_time = 0.f;
    ga->stage = GA_STAGE_EVAL;

    for (int i = 0; i < ga->population_size; ++i)
    {
        ga->population[i].fitness = 0.f;
        reset_agent(ga, &ga->agents[i]);
    }
}

void ga_init(GAContext* ga, int population_size)
{
    if (!ga)
        return;
    srand((unsigned)time(NULL));
    ga->population_size = population_size;
    ga->generation      = 0;
    ga->eval_time       = 0.f;
    ga->eval_duration   = 15.0f;
    ga->running         = 0;
    ga->stage           = GA_STAGE_EVAL;
    ga->best_index      = 0;
    ga->best_fitness    = -1e9f;
    ga->gen_best_fitness = -1e9f;
    ga->has_champion    = 0;
    ga->champion_fitness = -1e9f;
    ga->display_active  = 0;
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
    ga->has_champion = 0;
    ga->champion_fitness = -1e9f;
    ga->display_active = 0;
    ga->best_index = 0;
    ga->stage = GA_STAGE_EVAL;
    for (int i = 0; i < ga->population_size; ++i)
    {
        ga->population[i].fitness = 0.f;
        reset_agent(ga, &ga->agents[i]);
    }
}

void ga_reset_agents(GAContext* ga)
{
    if (!ga || !ga->agents)
        return;
    for (int i = 0; i < ga->population_size; ++i)
    {
        ga->population[i].fitness = 0.f;
        reset_agent(ga, &ga->agents[i]);
    }
    ga->eval_time = 0.f;
    ga->stage = GA_STAGE_EVAL;
    ga->best_index = 0;
    ga->display_active = 0;
}

void ga_update(GAContext* ga, float dt)
{
    if (!ga || !ga->running)
        return;

    if (ga->stage == GA_STAGE_EVAL)
    {
        ga->eval_time += dt;
        ga_eval_parallel(ga, dt, 1);

        if (ga->eval_time < ga->eval_duration)
            return;
        ga->stage = GA_STAGE_SELECT;
        return;
    }

    if (ga->stage == GA_STAGE_SELECT)
    {
        ga_do_select(ga);
        ga->stage = GA_STAGE_MUTATE;
        return;
    }

    if (ga->stage == GA_STAGE_MUTATE)
    {
        ga_do_mutate(ga);
    }
}

void ga_run_generation(GAContext* ga, float dt)
{
    if (!ga || !ga->running)
        return;
    if (dt <= 0.f)
        dt = 1.f / 120.f;

    if (ga->stage != GA_STAGE_EVAL)
    {
        ga->stage = GA_STAGE_EVAL;
        ga->eval_time = 0.f;
    }

    int steps = (int)ceilf(ga->eval_duration / dt);
    if (steps < 1)
        steps = 1;

    ga->eval_time = 0.f;
    ga_eval_parallel(ga, dt, steps);
    ga->eval_time = ga->eval_duration;

    ga->stage = GA_STAGE_SELECT;
    ga_do_select(ga);
    ga->stage = GA_STAGE_MUTATE;
    ga_do_mutate(ga);
}

void ga_display_step(GAContext* ga, float dt)
{
    if (!ga || !ga->running)
        return;
    if (dt <= 0.f)
        return;
    if (!ga->has_champion)
        return;

    if (!ga->display_active)
    {
        reset_agent(ga, &ga->display_agent);
        ga->display_active = 1;
        ga->eval_time = 0.f;
    }

    ga->eval_time += dt;
    ga_step_agent(ga, &ga->display_agent, &ga->champion, dt, 0);
    if (ga->eval_time >= ga->eval_duration)
    {
        reset_agent(ga, &ga->display_agent);
        ga->eval_time = 0.f;
    }
}

const GAAgent* ga_get_display_agent(const GAContext* ga)
{
    if (!ga || !ga->has_champion)
        return NULL;
    return &ga->display_agent;
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

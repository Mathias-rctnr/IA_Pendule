#include "ga.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

// Simple physics sim for fitness (pendulum about moving pivot with control u = kp*theta + kd*theta_dot)
static float simulate_genome(float kp, float kd)
{
    const float L   = 2.0f;   // meters (scale independent)
    const float g   = 9.81f;
    const float dam = 0.05f;  // small angular damping
    float theta = -0.7f;      // rad (displaced)
    float omega = 0.f;
    float fitness = 0.f;
    const float dt = 0.01f;
    const int steps = (int)(8.0f / dt); // simulate 8 seconds

    for (int i = 0; i < steps; ++i)
    {
        float u = kp * theta + kd * omega; // pivot acceleration surrogate
        // equation for pendulum with moving pivot: theta'' = -(g/L) sin(theta) - (u/L) cos(theta) - dam*omega
        float alpha = -(g / L) * sinf(theta) - (u / L) * cosf(theta) - dam * omega;
        omega += alpha * dt;
        theta += omega * dt;

        // accumulate fitness (higher is better): negative of angle/velocity magnitude
        fitness -= fabsf(theta) + 0.1f * fabsf(omega) + 0.0005f * u * u;
    }
    return fitness;
}

static float frand(float a, float b)
{
    return a + (b - a) * ((float)rand() / (float)RAND_MAX);
}

void ga_init(GAContext* ga)
{
    if (!ga)
        return;
    srand((unsigned)time(NULL));
    ga->population_size = 30;
    ga->generation      = 0;
    ga->population      = calloc((size_t)ga->population_size, sizeof(Genome));
    for (int i = 0; i < ga->population_size; ++i)
    {
        ga->population[i].kp = frand(-20.f, 20.f);
        ga->population[i].kd = frand(-10.f, 10.f);
    }
    ga->best_kp = 0.f;
    ga->best_kd = 0.f;
}

void ga_step(GAContext* ga)
{
    if (!ga || !ga->population)
        return;
    // Evaluate
    for (int i = 0; i < ga->population_size; ++i)
    {
        ga->population[i].fitness = simulate_genome(ga->population[i].kp, ga->population[i].kd);
    }
    // Select best and second best
    int best = 0, second = 0;
    for (int i = 1; i < ga->population_size; ++i)
    {
        if (ga->population[i].fitness > ga->population[best].fitness)
        {
            second = best;
            best   = i;
        }
        else if (ga->population[i].fitness > ga->population[second].fitness)
        {
            second = i;
        }
    }
    ga->best_kp = ga->population[best].kp;
    ga->best_kd = ga->population[best].kd;

    // Create next generation (elitism + gaussian mutation + crossover)
    Genome* next = calloc((size_t)ga->population_size, sizeof(Genome));
    next[0] = ga->population[best];
    next[1] = ga->population[second];
    for (int i = 2; i < ga->population_size; ++i)
    {
        const Genome* p1 = &ga->population[rand() % ga->population_size];
        const Genome* p2 = &ga->population[rand() % ga->population_size];
        float alpha = frand(0.f, 1.f);
        next[i].kp = alpha * p1->kp + (1.f - alpha) * p2->kp;
        next[i].kd = alpha * p1->kd + (1.f - alpha) * p2->kd;
        // mutation
        next[i].kp += frand(-1.0f, 1.0f);
        next[i].kd += frand(-0.5f, 0.5f);
    }
    free(ga->population);
    ga->population = next;
    ga->generation++;
}

void ga_free(GAContext* ga)
{
    if (!ga)
        return;
    free(ga->population);
    ga->population = NULL;
}

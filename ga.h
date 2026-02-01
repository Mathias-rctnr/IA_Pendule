#pragma once

// Simple stub for a future genetic algorithm implementation.
// Add genome representation, fitness evaluation, mutation/crossover, etc.

typedef struct
{
    float kp;
    float kd;
    float fitness;
} Genome;

typedef struct
{
    int     population_size;
    int     generation;
    float   best_kp;
    float   best_kd;
    Genome* population;
} GAContext;

void ga_init(GAContext* ga);
void ga_step(GAContext* ga);
void ga_free(GAContext* ga);

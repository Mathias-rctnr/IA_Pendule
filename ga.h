#pragma once

#define GA_INPUTS 4
#define GA_MAX_HIDDEN 8
#define GA_STAGE_EVAL 0
#define GA_STAGE_SELECT 1
#define GA_STAGE_MUTATE 2

typedef struct
{
    int   hidden;
    float w_in[GA_MAX_HIDDEN][GA_INPUTS];
    float b_h[GA_MAX_HIDDEN];
    float w_out[GA_MAX_HIDDEN];
    float w_direct[GA_INPUTS];
    float b_out;
    float fitness;
} Genome;

typedef struct
{
    float slider_value;
    float pivot_x;
    float pivot_v;
    float theta;
    float omega;
    float bob_x;
    float bob_y;
    float above_time;
    float last_control;
    float fitness;
} GAAgent;

typedef struct
{
    int     population_size;
    int     generation;
    float   eval_time;
    float   eval_duration;
    int     running;
    int     stage;
    int     best_index;
    float   best_fitness;
    float   gen_best_fitness;

    // environment
    float   track_left;
    float   track_width;
    float   pivot_y;
    float   length;
    float   base_k;
    float   base_d;
    float   gravity;
    float   damping;
    float   max_speed_factor;
    float   max_base_speed;
    float   upright_threshold;
    int     allow_remove_nodes;

    Genome  champion;
    int     has_champion;
    float   champion_fitness;
    GAAgent display_agent;
    int     display_active;

    Genome* population;
    GAAgent* agents;
} GAContext;

void  ga_init(GAContext* ga, int population_size);
void  ga_set_env(GAContext* ga,
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
                 float upright_threshold);
void  ga_start(GAContext* ga);
void  ga_update(GAContext* ga, float dt);
void  ga_run_generation(GAContext* ga, float dt);
void  ga_display_step(GAContext* ga, float dt);
void  ga_reset_agents(GAContext* ga);
const GAAgent* ga_get_display_agent(const GAContext* ga);
const GAAgent* ga_get_agents(const GAContext* ga, int* count, int* best_index);
void  ga_free(GAContext* ga);

#include <CSFML/Graphics.h>
#include <CSFML/Window.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "pendulum.h"
#include "ga.h"

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
    ga_init(&ga); // GA not used yet

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
    while (running && sfRenderWindow_isOpen(window))
    {
        while (sfRenderWindow_pollEvent(window, &event))
        {
            if (event.type == sfEvtClosed)
                running = false;
            pendulum_handle_event(&pendulum, &event);
        }

        float dt = sfTime_asSeconds(sfClock_restart(clock));
        pendulum_update(&pendulum, dt);

        sfRenderWindow_clear(window, bg);
        pendulum_draw(&pendulum, window);
        sfRenderWindow_display(window);
    }

    ga_free(&ga);
    pendulum_destroy(&pendulum);
    sfClock_destroy(clock);
    sfRenderWindow_destroy(window);
    return EXIT_SUCCESS;
}

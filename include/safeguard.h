#ifndef SAFEGUARD_H
#define SAFEGUARD_H

#include "racetrack.h"

/**
     * runs the safeguard controller and returns 1 if a goal state is reached and 0 on failure
     */
int run_safeguard_controller(const Map *map, const State *initial_state,
                             const char *nn_model_directory, int step_limit, int look_ahead_steps, int safety_distance);

#endif
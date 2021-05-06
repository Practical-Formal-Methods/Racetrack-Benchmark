#ifndef RACETRACK_H
#define RACETRACK_H

typedef struct Map Map;

typedef struct Position Position;

typedef struct Velocity Velocity;

typedef struct Acceleration Acceleration;

typedef struct State State;

typedef struct Distance Distance;

/**
     * computes the state that would be reached by executing the specified acceleration
     */
State *simulate_acceleration(const Map *map, const State *state,
                             const Acceleration *acceleration);

/**
     * logs the execution of the specified acceleration on the command line and computes
     * the next state
     */
State *execute_acceleration(const Map *map, const State *state, const Acceleration *acceleration);

/**
     * creates an initial state with zero velocity and a position that corresponds to a starting
     * position
     */
State *get_intial_state(const Map *map);

/**
     * computes the next state by adding the given acceleration and computing the next position
     */
State *get_next_state(const Map *map, const State *state, const Acceleration *acceleration);

/**
     * checks if a states is a goal state, i.e., the position is a goal position and
     * the velocity is zero
     */
int is_goal_state(const Map *map, const State *state);

/**
     * prints a map with the current position to the command line
     */
void print_state(const Map *map, const State *state);

/**
     * computes the goal distance
     */
Distance *get_goal_distance(const Map *map, const Position *position);

/**
     * creates a negated acceleration
     */
Acceleration *get_negated_acceleration(const Acceleration *acceleration);

/** 
     * extracts the position of a state
     */
Position *get_position(const State *state);

/**
     * extracts the velocity of a state
     */
Velocity *get_velocity(const State *state);

/**
     * creates a new acceleration
     */
Acceleration *create_acceleration(int x, int y);

/**
     * computes the wall distance as the maximal number of steps with the specified velocity
     * before hitting the wall
     */
int get_wall_distance(const Map *map, const Position *position, const Velocity *velocity);

void delete_map(Map *map);

void delete_position(Position *position);

void delete_velocity(Velocity *velocity);

void delete_acceleration(Acceleration *acceleration);

void delete_distance(Distance *distance);

void delete_state(State *state);

Map *get_map();

float *get_feature_values(const Map *map, const State *state);

const int velocity_to_traversed_positions[6][6][6][6] =
{{{{1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 1, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}},
{{{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 1}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}},
{{{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 1, 1, 0}, {0, 0, 0, 0, 1, 1}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}},
{{{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 1, 1}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}},
{{{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 1, 1}, {0, 0, 0, 0, 0, 0}}},
{{{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 0, 0}},
     {{1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 1, 0}},
     {{1, 1, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 1, 1}, {0, 0, 0, 0, 0, 1}}}};

#endif